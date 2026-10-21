using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using Ellipse = System.Windows.Shapes.Ellipse;
using Line = System.Windows.Shapes.Line;
using Polyline = System.Windows.Shapes.Polyline;
using MicroScribe.Core;
using MicroScribe.Export;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    /// <summary>Inspection plans (learn once, replay on the next parts) and SPC from the results log (Plan tab).</summary>
    public sealed partial class MainWindow
    {
        Project _plan;          // the learned project, with its points, while a plan runs
        string _planName;
        int _planStep = -1;
        bool _planDone;
        StackPanel _planPanel;
        string _spcKey;

        StackPanel BuildPlanPanel() { _planPanel = new StackPanel(); RebuildPlanPanel(); return _planPanel; }

        bool PlanRunning { get { return _plan != null && !_planDone; } }

        List<Feature> PlanSteps { get { return _plan != null ? InspectionPlan.Steps(_plan) : new List<Feature>(); } }

        /// <summary>The run's feature for the current step.</summary>
        Feature StepFeature
        {
            get
            {
                var steps = PlanSteps;
                return PlanRunning && _planStep >= 0 && _planStep < steps.Count ? _proj.Find(steps[_planStep].Id) : null;
            }
        }

        // ------------------------------------------------------------------ running a plan

        void RunPlanFromFile()
        {
            var dlg = new Microsoft.Win32.OpenFileDialog { Filter = "Projects (*.msproj)|*.msproj|All files|*.*", Title = "Inspection plan (a project measured once)" };
            if (dlg.ShowDialog(this) != true) return;
            try
            {
                var plan = ProjectIO.Load(dlg.FileName);
                string mp = plan.ModelPath;
                if (!string.IsNullOrEmpty(mp))
                {
                    if (!File.Exists(mp)) mp = Path.Combine(Path.GetDirectoryName(dlg.FileName), Path.GetFileName(mp));
                    if (File.Exists(mp)) { plan.NominalModel = TriMesh.LoadStl(mp, plan.ModelScale); plan.ModelPath = mp; }
                    else Status("The plan's model " + Path.GetFileName(plan.ModelPath) + " was not found");
                }
                StartPlan(plan, Path.GetFileNameWithoutExtension(dlg.FileName));
            }
            catch (Exception ex) { MessageBox.Show(this, "Could not open the plan: " + ex.Message, "Inspection plan", MessageBoxButton.OK, MessageBoxImage.Warning); }
        }

        void RunCurrentAsPlan()
        {
            if (_active != null) FinishMeasure(true);
            if (!InspectionPlan.Steps(_proj).Any(f => f.Points.Count > 0)) { Status("Measure the first part (all features, in order) before using it as a plan"); return; }
            if (_dirty && MessageBox.Show(this, "Save the learned project first? It is the plan for the next parts.", "Inspection plan", MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
                if (!SaveProject(false)) return;
            StartPlan(ProjectIO.Clone(_proj), _projPath != null ? Path.GetFileNameWithoutExtension(_projPath) : _proj.Title);
        }

        void StartPlan(Project plan, string name, string serial = null)
        {
            if (_active != null) FinishMeasure(true);
            FeatureSolver.SolveAll(plan);
            var prev = _proj; var prevPath = _projPath; bool prevDirty = _dirty;
            var run = InspectionPlan.NewRun(plan, serial ?? "");
            _proj = run; _projPath = null;
            if (serial == null && AskReportInfo("Part to inspect — " + name) == null) { _proj = prev; _projPath = prevPath; _dirty = prevDirty; return; }
            _plan = plan; _planName = name; _planStep = -1; _planDone = false;
            _selected = null; _selectedDim = 0; _dirty = true;
            _tabs.SelectedIndex = 5;
            RebuildAll();
            NextPlanStep();
        }

        void NextPlanStep()
        {
            var steps = PlanSteps;
            _planStep++;
            if (_planStep >= steps.Count) { PlanComplete(); return; }
            var f = _proj.Find(steps[_planStep].Id);
            if (f == null) { NextPlanStep(); return; }
            ResumeMeasure(f);
            int need = InspectionPlan.PlannedPoints(steps[_planStep]);
            Status(string.Format("Step {0} of {1}: {2} ({3}){4}{5}", _planStep + 1, steps.Count, f.Name, f.Kind == FeatureKind.Surface ? "freeform" : f.Kind.ToString(),
                need > 0 ? " — " + need + " points" : " — trace, then pedal 2", string.IsNullOrWhiteSpace(f.Notes) ? "" : " — " + f.Notes.Trim()));
            Sounds.Blip();
            RebuildPlanPanel();
            RebuildToolLayer();
        }

        /// <summary>Called after each captured point: a step finishes by itself once its points are taken.</summary>
        void PlanAfterPoint()
        {
            var f = StepFeature;
            if (f == null || _active != f) return;
            int need = InspectionPlan.PlannedPoints(PlanSteps[_planStep]);
            if (need > 0 && f.ActivePoints.Count() >= need) FinishMeasure();
            else RebuildPlanPanel();
        }

        /// <summary>Called when the operator finishes a feature (pedal 2 / Enter).</summary>
        void PlanAfterFinish(Feature f)
        {
            if (PlanRunning && f != null && _planStep >= 0 && _planStep < PlanSteps.Count && f.Id == PlanSteps[_planStep].Id) NextPlanStep();
        }

        void SkipStep()
        {
            if (!PlanRunning) return;
            if (_active != null) FinishMeasure(true);
            NextPlanStep();
        }

        void RedoStep()
        {
            if (!PlanRunning) return;
            var f = StepFeature;
            if (f != null && f.Points.Count == 0 && _planStep > 0) { _planStep -= 2; if (_active != null) FinishMeasure(true); NextPlanStep(); f = StepFeature; }
            if (f == null) return;
            f.Points.Clear();
            if (_active != f) ResumeMeasure(f);
            Changed();
            Status("Step " + (_planStep + 1) + " cleared — measure " + f.Name + " again");
            RebuildPlanPanel();
        }

        void StopPlan()
        {
            if (_active != null) FinishMeasure(true);
            _plan = null; _planStep = -1; _planDone = false;
            Status("Plan stopped — the measured part stays open as a normal project");
            UpdatePrompt(); RebuildPlanPanel(); RebuildToolLayer();
        }

        void PlanComplete()
        {
            _planDone = true;
            _active = null;
            Changed();
            var rows = Results.Rows(_proj).Where(r => r.Status != TolStatus.None).ToList();
            int fail = rows.Count(r => r.Status == TolStatus.Fail);
            if (fail == 0) Sounds.Success(); else Sounds.Warn();
            Status(rows.Count == 0 ? "Plan complete" : fail == 0 ? "Plan complete — all " + rows.Count + " characteristics in tolerance"
                : string.Format("Plan complete — {0} of {1} characteristics OUT of tolerance", fail, rows.Count));
            _tabs.SelectedIndex = 5;
            UpdatePrompt();
            RebuildPlanPanel();
            RebuildToolLayer();
        }

        void NextPart()
        {
            if (_plan == null) return;
            var info = new { pn = _proj.PartNumber, op = _proj.Operator, serial = InspectionPlan.NextSerial(_proj.PartSerial) };
            var plan = _plan; string name = _planName;
            _plan = null;
            StartPlan(plan, name, info.serial);
            _proj.PartNumber = info.pn; _proj.Operator = info.op;
            RebuildPlanPanel();
        }

        /// <summary>Appends to the remembered results log without asking (asks the first time).</summary>
        void LogPartResults()
        {
            if (string.IsNullOrEmpty(_s.ResultsLogPath)) { AppendResultsLog(); RebuildPlanPanel(); return; }
            try
            {
                int n = Results.AppendCsv(_s.ResultsLogPath, _proj, ReportHeader(), Fmt);
                Sounds.Good();
                Status(string.Format("Logged {0} results of {1} {2} to {3}", n, _proj.PartNumber, _proj.PartSerial, Path.GetFileName(_s.ResultsLogPath)));
                RebuildPlanPanel();
            }
            catch (Exception ex) { MessageBox.Show(this, "Could not write the log: " + ex.Message, "Results log", MessageBoxButton.OK, MessageBoxImage.Error); }
        }

        /// <summary>Where the plan probed this step's feature (machine frame of this run).</summary>
        List<Vec3> GhostPoints()
        {
            var steps = PlanSteps;
            if (!PlanRunning || _planStep < 0 || _planStep >= steps.Count) return new List<Vec3>();
            var src = steps[_planStep];
            // once this run's alignment is established the plan's points follow the part; before, assume the same fixture
            bool aligned = string.IsNullOrEmpty(_proj.AlignmentError) && string.IsNullOrEmpty(_plan.AlignmentError) && _proj.AlignmentRecipe.Orientation != OrientationMode.Machine
                && new[] { _proj.AlignmentRecipe.PrimaryId, _proj.AlignmentRecipe.SecondaryId }.Where(id => id != 0).All(id => { var x = _proj.Find(id); return x != null && x.Geometry != null; });
            Mat4 g = aligned ? _proj.AlignmentMatrix.RigidInverse() * _plan.AlignmentMatrix : Mat4.Identity();
            return src.ActivePoints.Select(q => g.TransformPoint(q.Position)).ToList();
        }

        void AddGhost(Model3DGroup g, List<Label3D> labels, Mat4 M)
        {
            var pts = GhostPoints();
            if (pts.Count == 0) return;
            var mb = new MeshBuilder();
            foreach (var p in pts) mb.Sphere(M.TransformPoint(p), _marker * 1.3, 10, 6);
            g.Children.Add(SceneView.Model(mb, Materials.Translucent(Color.FromRgb(192, 132, 252), 0.55)));
            Vec3 c = Vec3.Zero; foreach (var p in pts) c += p; c = c / pts.Count;
            labels.Add(new Label3D { Position = M.TransformPoint(c), Text = "Step " + (_planStep + 1) + ": " + PlanSteps[_planStep].Name, Bold = true, Background = Ui.Frozen(Color.FromArgb(220, 124, 58, 237)) });
        }

        // ------------------------------------------------------------------ Plan tab

        void RebuildPlanPanel()
        {
            var p = _planPanel;
            if (p == null) return;
            p.Children.Clear();
            var c = new StackPanel();
            c.Children.Add(Ui.Header("INSPECTION PLAN"));
            if (_plan == null)
            {
                c.Children.Add(Ui.Text("Measure a first part completely — features in the order the operator should probe them (feature notes become " +
                    "prompts), measurements with nominals and tolerances, alignment — and save it. Then run it as a plan: every next part is " +
                    "measured step by step, with markers showing where the first part was probed.", 11.5, null, Ui.Muted));
                var b = new WrapPanel();
                b.Children.Add(Ui.Btn("Run this project as a plan", RunCurrentAsPlan));
                b.Children.Add(Ui.Btn("Run plan from file…", RunPlanFromFile));
                c.Children.Add(b);
            }
            else
            {
                c.Children.Add(Ui.Text(string.Format("{0} — part {1} {2}", _planName, _proj.PartNumber, _proj.PartSerial), 13, FontWeights.SemiBold));
                var steps = PlanSteps;
                var list = new StackPanel { Margin = new Thickness(0, 4, 0, 4) };
                for (int i = 0; i < steps.Count; i++)
                {
                    var f = _proj.Find(steps[i].Id);
                    int need = InspectionPlan.PlannedPoints(steps[i]);
                    int have = f != null ? f.ActivePoints.Count() : 0;
                    bool cur = PlanRunning && i == _planStep;
                    string mark = cur ? "▶" : f != null && f.Geometry != null ? "✓" : i < _planStep || _planDone ? "–" : "·";
                    var t = Ui.Text(string.Format("{0}  {1}. {2}  ({3}{4})", mark, i + 1, steps[i].Name, have, need > 0 ? "/" + need + " pts" : " pts"), 12, cur ? FontWeights.SemiBold : (FontWeight?)null,
                        cur ? Ui.Accent : f != null && f.Geometry != null ? PassBrush : Ui.Muted);
                    list.Children.Add(t);
                    if (cur && !string.IsNullOrWhiteSpace(steps[i].Notes)) list.Children.Add(Ui.Text("      " + steps[i].Notes.Trim(), 11.5, null, Brushes.DarkSlateGray));
                }
                c.Children.Add(list);
                var b = new WrapPanel();
                if (PlanRunning)
                {
                    b.Children.Add(Ui.Btn("Skip step", SkipStep));
                    b.Children.Add(Ui.Btn("Redo step", RedoStep));
                    b.Children.Add(Ui.Btn("Stop plan", StopPlan));
                }
                else
                {
                    var rows = Results.Rows(_proj).Where(r => r.Status != TolStatus.None).ToList();
                    int fail = rows.Count(r => r.Status == TolStatus.Fail);
                    c.Children.Add(Ui.Text(rows.Count == 0 ? "Done (no tolerances in the plan)." : fail == 0 ? "✓ PASS — all " + rows.Count + " characteristics in tolerance"
                        : string.Format("✗ FAIL — {0} of {1} out of tolerance: {2}", fail, rows.Count, string.Join(", ", rows.Where(r => r.Status == TolStatus.Fail).Select(r => r.Name).Distinct())),
                        13, FontWeights.SemiBold, rows.Count == 0 ? Ui.Muted : fail == 0 ? PassBrush : FailBrush));
                    b.Children.Add(Ui.Btn("Report…", ExportReport));
                    b.Children.Add(Ui.Btn(string.IsNullOrEmpty(_s.ResultsLogPath) ? "Log results…" : "Log results", LogPartResults,
                        string.IsNullOrEmpty(_s.ResultsLogPath) ? "Append to a CSV results log" : "Append to " + _s.ResultsLogPath));
                    b.Children.Add(Ui.Btn("Save part…", () => SaveProject(true)));
                    b.Children.Add(Ui.Btn("Next part ▶", NextPart));
                    b.Children.Add(Ui.Btn("Close plan", StopPlan));
                }
                c.Children.Add(b);
            }
            p.Children.Add(Ui.Card(c));
            p.Children.Add(Ui.Card(SpcCard()));
        }

        UIElement SpcCard()
        {
            var c = new StackPanel();
            c.Children.Add(Ui.Header("TRENDS (results log)"));
            string path = _s.ResultsLogPath;
            c.Children.Add(Ui.Row(Ui.Btn("Choose log…", () =>
            {
                var dlg = new Microsoft.Win32.OpenFileDialog { Filter = "Results log (*.csv)|*.csv" };
                if (dlg.ShowDialog(this) == true) { _s.ResultsLogPath = dlg.FileName; RebuildPlanPanel(); }
            }), Ui.Text("  " + (string.IsNullOrEmpty(path) ? "no log yet" : Path.GetFileName(path)), 11.5, null, Ui.Muted)));
            if (string.IsNullOrEmpty(path) || !File.Exists(path))
            {
                c.Children.Add(Ui.Text("Log each part's results (File → Append results to log, or after a plan run) to follow every characteristic over many parts.", 11.5, null, Ui.Muted));
                return c;
            }
            List<LogEntry> log;
            try { log = ResultsLog.Read(path); }
            catch (Exception ex) { c.Children.Add(Ui.Text("Could not read the log: " + ex.Message, 11.5, null, FailBrush)); return c; }
            var keys = log.Select(e => e.Key).Distinct().ToList();
            if (keys.Count == 0) { c.Children.Add(Ui.Text("The log is empty.", 11.5, null, Ui.Muted)); return c; }
            if (_spcKey == null || !keys.Contains(_spcKey)) _spcKey = keys[0];
            var pick = new ComboBox { Width = 300, Margin = new Thickness(0, 4, 0, 4) };
            foreach (var k in keys) pick.Items.Add(k);
            pick.SelectedItem = _spcKey;
            pick.SelectionChanged += (s, e) => { _spcKey = (string)pick.SelectedItem; RebuildPlanPanel(); };
            c.Children.Add(pick);
            var rows = log.Where(e => e.Key == _spcKey && !double.IsNaN(e.Actual)).OrderBy(e => e.Date).ToList();
            var last = rows.LastOrDefault();
            double lsl = last != null ? last.Lsl : double.NaN, usl = last != null ? last.Usl : double.NaN;
            var st = Spc.Compute(rows.Select(e => e.Actual).ToList(), lsl, usl);
            bool ang = last != null && last.Unit == "deg";
            Func<double, string> V = x => double.IsNaN(x) ? "—" : ang ? x.ToString("0.000", Ui.CI) + "°" : Ui.L(x);
            c.Children.Add(TrendChart(rows.Select(e => e.Actual).ToList(), lsl, usl, st.Mean));
            c.Children.Add(new TextBlock
            {
                FontFamily = Ui.Mono, FontSize = 11.5, TextWrapping = TextWrapping.Wrap,
                Text = string.Format(Ui.CI, "n {0}   mean {1}   σ {2}\nmin {3}   max {4}   limits {5} … {6}\nCp {7}   Cpk {8}   out of tolerance {9}",
                    st.N, V(st.Mean), V(st.Sigma), V(st.Min), V(st.Max), V(lsl), V(usl),
                    double.IsNaN(st.Cp) ? "—" : st.Cp.ToString("0.00", Ui.CI), double.IsNaN(st.Cpk) ? "—" : st.Cpk.ToString("0.00", Ui.CI), st.OutOfTolerance)
            });
            if (st.N > 0 && st.N < 20) c.Children.Add(Ui.Text("Capability numbers need about 20+ parts to mean much.", 11, null, Ui.Muted));
            return c;
        }

        /// <summary>Run chart: values in order, specification limits in red, mean in grey.</summary>
        static FrameworkElement TrendChart(List<double> v, double lsl, double usl, double mean)
        {
            const double W = 300, Hh = 140, pad = 8;
            var cv = new Canvas { Width = W, Height = Hh, Background = Ui.Frozen(Color.FromRgb(248, 250, 252)), Margin = new Thickness(0, 2, 0, 4), ClipToBounds = true };
            if (v.Count == 0) return cv;
            var all = v.Concat(new[] { lsl, usl, mean }.Where(x => !double.IsNaN(x))).ToList();
            double lo = all.Min(), hi = all.Max();
            if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }
            double margin = (hi - lo) * 0.08; lo -= margin; hi += margin;
            Func<int, double> X = i => pad + (v.Count == 1 ? (W - 2 * pad) / 2 : i * (W - 2 * pad) / (v.Count - 1));
            Func<double, double> Y = y => Hh - pad - (y - lo) / (hi - lo) * (Hh - 2 * pad);
            Action<double, Brush, bool> hline = (y, b, dash) =>
            {
                if (double.IsNaN(y)) return;
                var l = new Line { X1 = 0, X2 = W, Y1 = Y(y), Y2 = Y(y), Stroke = b, StrokeThickness = 1.2 };
                if (dash) l.StrokeDashArray = new DoubleCollection { 4, 3 };
                cv.Children.Add(l);
            };
            hline(lsl, Brushes.IndianRed, true); hline(usl, Brushes.IndianRed, true); hline(mean, Brushes.Gray, false);
            var pl = new Polyline { Stroke = Ui.Accent, StrokeThickness = 1.6 };
            for (int i = 0; i < v.Count; i++) pl.Points.Add(new Point(X(i), Y(v[i])));
            cv.Children.Add(pl);
            for (int i = 0; i < v.Count; i++)
            {
                bool bad = (!double.IsNaN(lsl) && v[i] < lsl) || (!double.IsNaN(usl) && v[i] > usl);
                var dot = new Ellipse { Width = 6, Height = 6, Fill = bad ? Brushes.Red : Ui.Accent };
                Canvas.SetLeft(dot, X(i) - 3); Canvas.SetTop(dot, Y(v[i]) - 3);
                cv.Children.Add(dot);
            }
            return cv;
        }
    }
}
