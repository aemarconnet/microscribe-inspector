using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using MicroScribe.Core;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    /// <summary>Probe qualification on a reference sphere and the arm checks (Arm &amp; tip tab).</summary>
    public sealed partial class MainWindow
    {
        enum CheckMode { None, SinglePoint, Length }

        CheckMode _check;
        readonly List<Vec3> _spatPts = new List<Vec3>();
        readonly List<double> _lengths = new List<double>();
        Vec3? _lengthA;
        TextBlock _checkText;

        // ------------------------------------------------------------------ probe qualification

        UIElement QualifyCard()
        {
            var c = new StackPanel();
            c.Children.Add(Ui.Header("PROBE QUALIFICATION (reference sphere)"));
            c.Children.Add(Ui.Text("Measure a precision ball of known diameter (bearing ball, tooling ball) as a Sphere feature with this tip — " +
                "10 or more points spread over its upper half — then qualify the tip from it. This finds the tip's effective ball radius.", 11.5, null, Ui.Muted));
            var spheres = _proj.Features.Where(f => f.Origin == FeatureOrigin.Measured && f.RawFit is SpherePrim).ToList();
            var pick = new ComboBox { Width = 160 };
            foreach (var f in spheres) pick.Items.Add(f.Name);
            if (spheres.Count > 0) pick.SelectedIndex = _selected != null && spheres.Contains(_selected) ? spheres.IndexOf(_selected) : spheres.Count - 1;
            c.Children.Add(Ui.Row(Lbl("Sphere feature"), pick));
            c.Children.Add(Ui.Row(Lbl("Reference Ø"), Ui.LenBox(_s.QualifyReference, v => _s.QualifyReference = Math.Max(0, v)), Ui.UnitLabel()));
            c.Children.Add(Ui.Row(Ui.Btn("Qualify tip", () => { if (pick.SelectedIndex >= 0) QualifyTip(spheres[pick.SelectedIndex]); else Status("Measure a Sphere feature on the reference ball first"); })));
            var tip = _s.CurrentTip;
            c.Children.Add(Ui.Text(tip.IsQualified
                ? string.Format("'{0}' qualified {1:yyyy-MM-dd}: ball radius {2} on Ø{3}, {4} points, form {5}, RMS {6}.",
                    tip.Name, tip.QualifiedOn, Ui.LU(tip.BallRadius), Ui.L(tip.QualifiedReference), tip.QualifiedPoints, Ui.LU(tip.QualifiedFormError), Ui.LU(tip.QualifiedRms))
                : string.Format("'{0}' has not been qualified yet.", tip.Name), 11.5));
            return c;
        }

        void QualifyTip(Feature f)
        {
            var raw = f.RawFit as SpherePrim;
            double refD = _s.QualifyReference;
            double rEff = Calibration.QualifiedBallRadius(raw, refD);
            if (double.IsNaN(rEff)) { Status("Enter the reference diameter and pick a solved sphere"); return; }
            var pts = f.ActivePoints.ToList();
            var tip = _s.CurrentTip;
            if (rEff < -0.05)
            {
                MessageBox.Show(this, string.Format("The ball centres lie on a sphere of Ø{0}, smaller than the reference Ø{1}. " +
                    "Was the ball probed from outside, and is the reference diameter right?", Ui.LU(2 * raw.Radius), Ui.LU(refD)), "Qualify tip");
                return;
            }
            double used = pts.Average(q => q.ProbeRadius);
            double diaErr = 2 * (raw.Radius - used) - refD;
            var warn = FitQuality.Warnings(FeatureKind.Sphere, raw, pts.Select(q => q.Position).ToList());
            string msg = string.Format("Effective ball radius: {0}   (current setting of '{1}': {2})\n\n" +
                "With the setting used for these points the ball measured Ø{3}: error {4}{5}.\nForm {6}, RMS {7}, {8} points.{9}\n\n" +
                "Use {0} as the ball radius of '{1}'? Points already measured keep the radius they were taken with.",
                Ui.LU(Math.Max(0, rEff)), tip.Name, Ui.LU(tip.BallRadius), Ui.LU(2 * (raw.Radius - used)), diaErr >= 0 ? "+" : "", Ui.LU(diaErr),
                Ui.LU(raw.Stats.Form), Ui.LU(raw.Stats.Rms), pts.Count, warn.Count > 0 ? "\n⚠ " + string.Join("\n⚠ ", warn) : "");
            if (MessageBox.Show(this, msg, "Qualify tip", MessageBoxButton.YesNo, MessageBoxImage.Question) != MessageBoxResult.Yes) return;
            tip.BallRadius = Math.Max(0, rEff);
            tip.QualifiedOn = DateTime.Now;
            tip.QualifiedReference = refD;
            tip.QualifiedRms = raw.Stats.Rms;
            tip.QualifiedFormError = raw.Stats.Form;
            tip.QualifiedPoints = pts.Count;
            ApplyTip();
            _s.Save();
            Sounds.Success();
            Status(string.Format("'{0}' qualified: ball radius {1}", tip.Name, Ui.LU(tip.BallRadius)));
        }

        // ------------------------------------------------------------------ arm check

        UIElement ArmCheckCard()
        {
            var c = new StackPanel();
            c.Children.Add(Ui.Header("ARM CHECK"));
            c.Children.Add(Ui.Text("Single-point test: seat the tip in a cone seat (or countersink) fixed to the table and capture 10+ readings " +
                "while moving the arm through very different poses, keeping the tip seated. Length test: capture seat A, then seat B, of a bar " +
                "with two seats a known distance apart; repeat at 5+ places and directions around the working volume.", 11.5, null, Ui.Muted));
            var modes = new WrapPanel();
            modes.Children.Add(Ui.Btn(_check == CheckMode.SinglePoint ? "■ Stop single-point test" : "Single-point test", () => ToggleCheck(CheckMode.SinglePoint)));
            modes.Children.Add(Ui.Btn(_check == CheckMode.Length ? "■ Stop length test" : "Length test", () => ToggleCheck(CheckMode.Length)));
            c.Children.Add(modes);
            c.Children.Add(Ui.Row(Lbl("Reference length"), Ui.LenBox(_s.ArmCheckReference, v => { _s.ArmCheckReference = Math.Max(0, v); UpdateCheckText(); }), Ui.UnitLabel()));
            c.Children.Add(Ui.Row(Lbl("Pass limit"), Ui.LenBox(_s.ArmCheckLimit, v => { _s.ArmCheckLimit = Math.Max(0, v); UpdateCheckText(); }), Ui.UnitLabel()));
            _checkText = new TextBlock { FontFamily = Ui.Mono, FontSize = 11.5, Margin = new Thickness(0, 4, 0, 4), TextWrapping = TextWrapping.Wrap };
            c.Children.Add(_checkText);
            var acts = new WrapPanel();
            acts.Children.Add(Ui.Btn("Remove last", () => CheckUndo()));
            acts.Children.Add(Ui.Btn("Save result", SaveCheck));
            c.Children.Add(acts);
            UpdateCheckText();

            c.Children.Add(Ui.Header("MEASURING CAPABILITY"));
            c.Children.Add(Ui.Text(CapabilityLine(), 11.5));
            var hist = _s.ArmChecks.OrderByDescending(h => h.Date).Take(8).ToList();
            if (hist.Count > 0)
                c.Children.Add(new TextBlock
                {
                    FontFamily = Ui.Mono, FontSize = 11, Margin = new Thickness(0, 4, 0, 0),
                    Text = string.Join("\n", hist.Select(h => string.Format(Ui.CI, "{0:yyyy-MM-dd}  {1,-12} n={2,-3} max {3}  2σ {4}  {5}",
                        h.Date, h.Kind, h.N, Ui.L(h.MaxDeviation), Ui.L(h.TwoSigma), h.Pass ? "PASS" : "FAIL")))
                });
            return c;
        }

        void ToggleCheck(CheckMode m)
        {
            if (_check == m) { _check = CheckMode.None; }
            else
            {
                if (_active != null) FinishMeasure(true);
                _pivotActive = false;
                _check = m;
                _spatPts.Clear(); _lengths.Clear(); _lengthA = null;
                Status(m == CheckMode.SinglePoint ? "Single-point test: seat the tip and capture from many different poses"
                                                  : "Length test: capture seat A, then seat B");
            }
            UpdatePrompt();
            RebuildArmPanel();
        }

        /// <summary>Receives averaged readings while a check runs.</summary>
        void CheckCapture(Vec3 tip, Vec3 dir)
        {
            if (_check == CheckMode.SinglePoint) _spatPts.Add(tip);
            else if (_check == CheckMode.Length)
            {
                if (!_lengthA.HasValue) _lengthA = tip;
                else { _lengths.Add((tip - _lengthA.Value).Length); _lengthA = null; }
            }
            Sounds.Blip();
            UpdateCheckText();
            UpdatePrompt();
        }

        bool CheckUndo()
        {
            if (_check == CheckMode.None) return false;
            if (_check == CheckMode.SinglePoint && _spatPts.Count > 0) _spatPts.RemoveAt(_spatPts.Count - 1);
            if (_check == CheckMode.Length)
            {
                if (_lengthA.HasValue) _lengthA = null;
                else if (_lengths.Count > 0) _lengths.RemoveAt(_lengths.Count - 1);
            }
            UpdateCheckText();
            UpdatePrompt();
            return true;
        }

        /// <summary>Enter / pedal 2 while a check runs: save it.</summary>
        bool CheckFinish()
        {
            if (_check == CheckMode.None) return false;
            SaveCheck();
            return true;
        }

        ArmCheckResult CurrentCheck()
        {
            if (_check == CheckMode.SinglePoint) return ArmCheck.SinglePoint(_spatPts);
            if (_check == CheckMode.Length) return ArmCheck.Length(_lengths, _s.ArmCheckReference);
            return null;
        }

        void UpdateCheckText()
        {
            if (_checkText == null) return;
            var r = CurrentCheck();
            if (r == null) { _checkText.Text = ""; return; }
            if (_check == CheckMode.SinglePoint)
                _checkText.Text = r.N < 2 ? r.N + " reading(s) — capture from different poses"
                    : string.Format("{0} readings   max from mean {1}   range {2}   2σ {3}   → {4}",
                        r.N, Ui.LU(r.MaxDeviation), Ui.L(r.Range), Ui.L(r.TwoSigma), r.MaxDeviation <= _s.ArmCheckLimit ? "PASS" : "FAIL");
            else
                _checkText.Text = (r.N == 0 ? "No lengths yet" : string.Format("{0} lengths   mean error {1}{2}   max error {3}   2σ {4}   → {5}",
                        r.N, r.MeanError >= 0 ? "+" : "", Ui.L(r.MeanError), Ui.LU(r.MaxDeviation), Ui.L(r.TwoSigma), r.MaxDeviation <= _s.ArmCheckLimit ? "PASS" : "FAIL"))
                    + (_lengthA.HasValue ? "\nSeat A captured — now capture seat B" : "")
                    + (_lengths.Count > 0 ? "\nLast: " + Ui.LU(_lengths[_lengths.Count - 1]) : "");
        }

        void SaveCheck()
        {
            var r = CurrentCheck();
            int need = _check == CheckMode.SinglePoint ? 5 : 3;
            if (r == null || r.N < need) { Status(string.Format("Need at least {0} {1} to save", need, _check == CheckMode.Length ? "lengths" : "readings")); return; }
            var rec = new ArmCheckRecord
            {
                Date = DateTime.Now,
                Kind = _check == CheckMode.SinglePoint ? "single-point" : "length",
                Arm = _arm != null && _arm.Info != null ? _arm.Info.ModelName + " " + _arm.Info.SerialNumber : "",
                Tip = _s.CurrentTip.Name,
                N = r.N, MaxDeviation = r.MaxDeviation, Range = r.Range, TwoSigma = r.TwoSigma, MeanError = r.MeanError,
                Reference = _check == CheckMode.Length ? _s.ArmCheckReference : 0,
                Limit = _s.ArmCheckLimit,
                Pass = r.MaxDeviation <= _s.ArmCheckLimit
            };
            _s.ArmChecks.Add(rec);
            while (_s.ArmChecks.Count > 200) _s.ArmChecks.RemoveAt(0);
            _s.Save();
            if (rec.Pass) Sounds.Success(); else Sounds.Warn();
            Status(string.Format("{0} test saved: max {1} — {2}", rec.Kind, Ui.LU(rec.MaxDeviation), rec.Pass ? "PASS" : "FAIL (limit " + Ui.LU(rec.Limit) + ")"));
            _check = CheckMode.None;
            UpdatePrompt();
            RebuildArmPanel();
        }

        string CheckPrompt()
        {
            if (_check == CheckMode.SinglePoint)
                return string.Format("Single-point test — {0} reading(s). Keep the tip seated, change the pose, Pedal 1/Space: capture · Pedal 2/Enter: save", _spatPts.Count);
            if (_check == CheckMode.Length)
                return string.Format("Length test — {0} length(s). Pedal 1/Space: capture seat {1} · Pedal 2/Enter: save", _lengths.Count, _lengthA.HasValue ? "B" : "A");
            return null;
        }

        /// <summary>The arm's demonstrated measuring error from the latest saved checks, in words.</summary>
        string CapabilityLine()
        {
            var a = _s.LatestCheck("single-point");
            var b = _s.LatestCheck("length");
            if (a == null && b == null) return "No arm check saved yet — run the single-point and length tests above to know what this arm can measure.";
            var parts = new List<string>();
            if (a != null) parts.Add(string.Format("single-point max {0} ({1:yyyy-MM-dd}, {2})", Ui.LU(a.MaxDeviation), a.Date, a.Pass ? "pass" : "FAIL"));
            if (b != null) parts.Add(string.Format("length error max {0} ({1:yyyy-MM-dd}, {2})", Ui.LU(b.MaxDeviation), b.Date, b.Pass ? "pass" : "FAIL"));
            double e = _s.MeasuringError;
            return "Arm check: " + string.Join("; ", parts) + string.Format(". Measuring error ≈ ±{0}: tolerances tighter than ±{1} cannot be judged reliably.",
                Ui.LU(e), Ui.LU(4 * e));
        }

        /// <summary>True when ± <paramref name="tolerance"/> is too tight for the arm's demonstrated error (4:1 rule).</summary>
        bool ToleranceTooTight(double tolerance)
        {
            double e = _s.MeasuringError;
            return !double.IsNaN(e) && e > 0 && Math.Abs(tolerance) < 4 * e;
        }

        /// <summary>On connect: remind when the last arm check is missing or older than the interval.</summary>
        void CheckReminder()
        {
            if (_s.ArmCheckIntervalDays <= 0) return;
            var last = _s.ArmChecks.OrderBy(c => c.Date).LastOrDefault();
            if (last == null) Status("Connected. Tip: run an arm check (Arm & tip tab) to know the arm's measuring error.");
            else if ((DateTime.Now - last.Date).TotalDays > _s.ArmCheckIntervalDays)
                Status(string.Format("Connected. The last arm check was {0:0} days ago — consider repeating it (Arm & tip tab).", (DateTime.Now - last.Date).TotalDays));
        }
    }
}
