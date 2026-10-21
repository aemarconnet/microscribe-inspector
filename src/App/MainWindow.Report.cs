using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using MicroScribe.Export;
using MicroScribe.Model;

namespace MicroScribe.App
{
    /// <summary>HTML inspection report with a picture of the view, and the per-part results log (CSV).</summary>
    public sealed partial class MainWindow
    {
        /// <summary>Asks for part number, serial, operator (and company / logo); null when cancelled.</summary>
        ReportInfo AskReportInfo(string title)
        {
            var w = new Window { Title = title, Owner = this, Width = 440, SizeToContent = SizeToContent.Height, ResizeMode = ResizeMode.NoResize, WindowStartupLocation = WindowStartupLocation.CenterOwner };
            var sp = new StackPanel { Margin = new Thickness(14) };
            Func<string, string, TextBox> field = (label, value) =>
            {
                var tb = new TextBox { Text = value ?? "", Width = 260, Margin = new Thickness(2) };
                sp.Children.Add(Ui.Row(Lbl(label), tb));
                return tb;
            };
            var part = field("Part number", _proj.PartNumber);
            var serial = field("Serial number", _proj.PartSerial);
            var op = field("Operator", string.IsNullOrEmpty(_proj.Operator) ? _s.Author : _proj.Operator);
            var company = field("Company", _s.ReportCompany);
            var logoText = Ui.Text(string.IsNullOrEmpty(_s.ReportLogoPath) ? "No logo" : Path.GetFileName(_s.ReportLogoPath), 11.5, null, Ui.Muted);
            string logo = _s.ReportLogoPath;
            sp.Children.Add(Ui.Row(Lbl("Logo"), Ui.Btn("Choose…", () =>
            {
                var dlg = new Microsoft.Win32.OpenFileDialog { Filter = "Images (*.png;*.jpg;*.jpeg)|*.png;*.jpg;*.jpeg" };
                if (dlg.ShowDialog(w) == true) { logo = dlg.FileName; logoText.Text = Path.GetFileName(logo); }
            }), Ui.Btn("None", () => { logo = ""; logoText.Text = "No logo"; }), logoText));
            bool ok = false;
            var btns = new WrapPanel { HorizontalAlignment = HorizontalAlignment.Right, Margin = new Thickness(0, 10, 0, 0) };
            var okb = Ui.Btn("OK", () => { ok = true; w.Close(); }); okb.IsDefault = true;
            var cb = Ui.Btn("Cancel", () => w.Close()); cb.IsCancel = true;
            btns.Children.Add(okb); btns.Children.Add(cb);
            sp.Children.Add(btns);
            w.Content = sp;
            w.ShowDialog();
            if (!ok) return null;
            _proj.PartNumber = part.Text.Trim(); _proj.PartSerial = serial.Text.Trim(); _proj.Operator = op.Text.Trim();
            _s.ReportCompany = company.Text.Trim(); _s.ReportLogoPath = logo ?? "";
            _dirty = true;
            return ReportHeader();
        }

        ReportInfo ReportHeader()
        {
            var info = new ReportInfo
            {
                PartNumber = _proj.PartNumber, Serial = _proj.PartSerial, Operator = string.IsNullOrEmpty(_proj.Operator) ? _s.Author : _proj.Operator,
                Company = _s.ReportCompany, Capability = CapabilityLine(),
                Software = Installer.AppName + " " + typeof(MainWindow).Assembly.GetName().Version.ToString(3)
            };
            try
            {
                if (!string.IsNullOrEmpty(_s.ReportLogoPath) && File.Exists(_s.ReportLogoPath))
                {
                    string ext = Path.GetExtension(_s.ReportLogoPath).ToLowerInvariant();
                    info.LogoDataUri = "data:image/" + (ext == ".png" ? "png" : "jpeg") + ";base64," + Convert.ToBase64String(File.ReadAllBytes(_s.ReportLogoPath));
                }
            }
            catch { info.LogoDataUri = null; }
            return info;
        }

        /// <summary>PNG of the 3D view as it is on screen.</summary>
        byte[] SnapshotView()
        {
            _view.UpdateLayout();
            int w = (int)Math.Max(1, _view.ActualWidth), h = (int)Math.Max(1, _view.ActualHeight);
            var rtb = new RenderTargetBitmap(w, h, 96, 96, PixelFormats.Pbgra32);
            rtb.Render(_view);
            var enc = new PngBitmapEncoder();
            enc.Frames.Add(BitmapFrame.Create(rtb));
            using (var ms = new MemoryStream()) { enc.Save(ms); return ms.ToArray(); }
        }

        void ExportReport()
        {
            if (_active != null) FinishMeasure(true);
            string path = ExportPath(".html", "HTML report (*.html)|*.html|Text report (*.txt)|*.txt");
            if (path == null) return;
            try
            {
                if (path.EndsWith(".txt", StringComparison.OrdinalIgnoreCase))
                    File.WriteAllText(path, CadExport.Report(_proj, true, Fmt, CapabilityLine()));
                else
                {
                    var info = AskReportInfo("Report details");
                    if (info == null) return;
                    var shots = new List<KeyValuePair<string, byte[]>> { new KeyValuePair<string, byte[]>("3D view (" + _proj.AlignmentDescription + ")", SnapshotView()) };
                    File.WriteAllText(path, HtmlReport.Build(_proj, Fmt, info, shots), new System.Text.UTF8Encoding(false));
                }
                Status("Report written to " + path + (path.EndsWith(".html", StringComparison.OrdinalIgnoreCase) ? " — print it to PDF from the browser if needed" : ""));
                try { System.Diagnostics.Process.Start(path); } catch { }
            }
            catch (Exception ex) { MessageBox.Show(this, "Report failed: " + ex.Message, "Report", MessageBoxButton.OK, MessageBoxImage.Error); }
        }

        /// <summary>Adds this part's results to a CSV log that collects many parts.</summary>
        void AppendResultsLog()
        {
            if (_proj.Dimensions.Count == 0) { Status("No measurements to log yet (Measure tab)"); return; }
            var dlg = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "Results log (*.csv)|*.csv", DefaultExt = ".csv", OverwritePrompt = false, Title = "Append results to log",
                FileName = string.IsNullOrEmpty(_s.ResultsLogPath) ? "results-log.csv" : Path.GetFileName(_s.ResultsLogPath)
            };
            if (!string.IsNullOrEmpty(_s.ResultsLogPath) && Directory.Exists(Path.GetDirectoryName(_s.ResultsLogPath))) dlg.InitialDirectory = Path.GetDirectoryName(_s.ResultsLogPath);
            if (dlg.ShowDialog(this) != true) return;
            var info = AskReportInfo("Part details for the log");
            if (info == null) return;
            try
            {
                int n = Results.AppendCsv(dlg.FileName, _proj, info, Fmt);
                _s.ResultsLogPath = dlg.FileName;
                Sounds.Good();
                Status(string.Format("Appended {0} result rows for part {1} {2} to {3}", n, info.PartNumber, info.Serial, dlg.FileName));
            }
            catch (Exception ex) { MessageBox.Show(this, "Could not write the log: " + ex.Message, "Results log", MessageBoxButton.OK, MessageBoxImage.Error); }
        }
    }
}
