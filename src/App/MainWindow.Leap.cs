using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using MicroScribe.Core;
using MicroScribe.Driver;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    /// <summary>
    /// Leapfrog: moving the arm to reach more of a large part. Cone seats measured as Point features before
    /// the move are touched again afterwards; the fit of the two sets maps the arm's new readings back onto
    /// its original position, so measuring simply continues.
    /// </summary>
    public sealed partial class MainWindow
    {
        bool _leapMode;
        readonly List<Vec3> _leapRaw = new List<Vec3>();

        /// <summary>The arm's latest reading in the project's frame (after any leapfrog).</summary>
        ArmSample Latest { get { return Tx(_arm != null ? _arm.Latest : null); } }

        /// <summary>
        /// Maps an arm reading into the project: from the arm's current position onto its original one
        /// (leapfrog), then scaled to 20 °C when the temperature correction is on.
        /// </summary>
        ArmSample Tx(ArmSample s)
        {
            if (s == null || (!_proj.ArmMoved && _proj.TempScale == 1)) return s;
            Mat4 sc = Mat4.Identity();
            sc[0, 0] = sc[1, 1] = sc[2, 2] = _proj.TempScale;
            Mat4 t = sc * _proj.ArmMatrix;
            return new ArmSample
            {
                Tip = t.TransformPoint(s.Tip), Direction = t.TransformVector(s.Direction),
                StylusFrame = s.StylusFrame != null ? t * s.StylusFrame : null,
                JointOrigins = s.JointOrigins != null ? s.JointOrigins.Select(t.TransformPoint).ToArray() : null,
                Buttons = s.Buttons, Encoders = s.Encoders, JointAngles = s.JointAngles, Timestamp = s.Timestamp
            };
        }

        List<Feature> LeapReferences
        {
            get { return _proj.LeapRefIds.Select(id => _proj.Find(id)).Where(f => f != null && f.RawFit is PointPrim).ToList(); }
        }

        void StartLeap()
        {
            if (LeapReferences.Count < 3) { Status("Choose 3 or more cone seats (Point features) as references first"); return; }
            if (_active != null) FinishMeasure(true);
            _leapRaw.Clear();
            _leapMode = true;
            Status("Arm moved? Seat the ball in each reference in turn and press pedal 1");
            UpdatePrompt(); RebuildWorkspacePanel(); RebuildToolLayer();
        }

        void CancelLeap() { _leapMode = false; _leapRaw.Clear(); UpdatePrompt(); RebuildWorkspacePanel(); RebuildToolLayer(); }

        void LeapCapture(Vec3 tip, Vec3 dir)
        {
            // back to the arm's own coordinates (the fit replaces any earlier move), at the project's temperature scale
            _leapRaw.Add(_proj.ArmMatrix.RigidInverse().TransformPoint(tip / _proj.TempScale) * _proj.TempScale);
            Sounds.Blip();
            if (_leapRaw.Count >= LeapReferences.Count) SolveLeap();
            UpdatePrompt(); RebuildToolLayer();
        }

        /// <summary>Fit of the re-touched seats (arm coordinates) onto the references: new arm reading → original position.</summary>
        Mat4 ComputeLeap(out double rms, out double max)
        {
            var refs = LeapReferences.Select(f => ((PointPrim)f.RawFit).Position).ToList();
            Mat4 t = ModelFit.FromPairs(_leapRaw, refs, out rms);
            max = _leapRaw.Select((p, i) => (t.TransformPoint(p) - refs[i]).Length).Max();
            return t;
        }

        void ApplyLeap(Mat4 t, double rms, double max)
        {
            _proj.ArmMatrix = t;
            _proj.ArmMoves++;
            _proj.ArmMoveNote = string.Format("arm moved {0}×, last fit RMS {1}, max {2} over {3} seats", _proj.ArmMoves, Ui.LU(rms), Ui.LU(max), _leapRaw.Count);
            _dirty = true;
        }

        void SolveLeap()
        {
            double rms, max;
            Mat4 t = ComputeLeap(out rms, out max);
            _leapMode = false;
            var r = MessageBox.Show(this, string.Format("The {0} references fit with RMS {1}, largest {2}.\n\nUse this arm position from now on?{3}",
                _leapRaw.Count, Ui.LU(rms), Ui.LU(max), max > 0.25 ? "\n\n⚠ A reference is off by more than 0.25 mm — check that the ball was seated and the order was right." : ""),
                "Leapfrog", MessageBoxButton.YesNo, max > 0.25 ? MessageBoxImage.Warning : MessageBoxImage.Question);
            if (r == MessageBoxResult.Yes)
            {
                ApplyLeap(t, rms, max);
                Sounds.Success();
                Status("Arm position updated — " + _proj.ArmMoveNote);
            }
            _leapRaw.Clear();
            UpdatePrompt(); RebuildWorkspacePanel(); RebuildToolLayer(); RebuildScene();
        }

        string LeapPrompt()
        {
            if (!_leapMode) return null;
            var refs = LeapReferences;
            int k = _leapRaw.Count;
            return k < refs.Count ? string.Format("Leapfrog — seat the ball in reference {0} of {1} ({2}) and press pedal 1 / Space. Backspace = undo", k + 1, refs.Count, refs[k].Name) : null;
        }

        bool LeapUndo()
        {
            if (!_leapMode) return false;
            if (_leapRaw.Count > 0) _leapRaw.RemoveAt(_leapRaw.Count - 1);
            UpdatePrompt(); RebuildToolLayer();
            return true;
        }

        static readonly KeyValuePair<string, double>[] PartMaterials =
        {
            new KeyValuePair<string, double>("Steel", 11.5), new KeyValuePair<string, double>("Stainless steel", 16), new KeyValuePair<string, double>("Cast iron", 10.5),
            new KeyValuePair<string, double>("Aluminium", 23), new KeyValuePair<string, double>("Brass", 19), new KeyValuePair<string, double>("Copper", 17),
            new KeyValuePair<string, double>("Titanium", 8.6), new KeyValuePair<string, double>("Glass", 9), new KeyValuePair<string, double>("ABS plastic", 80),
            new KeyValuePair<string, double>("Nylon", 90), new KeyValuePair<string, double>("Custom", double.NaN)
        };

        UIElement TemperatureCard()
        {
            var c = new StackPanel();
            c.Children.Add(Ui.Header("TEMPERATURE"));
            var on = new CheckBox { Content = "Correct lengths to 20 °C", IsChecked = _proj.TempCorrection, Margin = new Thickness(0, 2, 0, 2) };
            double temp = _proj.PartTemperature, cte = _proj.Cte;
            Action apply = () =>
            {
                _proj.ApplyTemperature(on.IsChecked == true, temp, cte);
                _dirty = true;
                Changed();
                if (_proj.AlignmentRecipe.Orientation == OrientationMode.Model) Status("Temperature changed — run the best fit to the model again (Alignment tab)");
                RebuildWorkspacePanel();
            };
            on.Click += (s, e) => apply();
            c.Children.Add(on);
            c.Children.Add(Ui.Row(Lbl("Part temperature"), Ui.NumBox(temp, v => { temp = v; if (_proj.TempCorrection) apply(); else _proj.PartTemperature = v; }, 70), Ui.Text(" °C", 12, null, Ui.Muted)));
            var mat = new ComboBox { Width = 140 };
            foreach (var m in PartMaterials) mat.Items.Add(m.Key);
            int mi = Array.FindIndex(PartMaterials, m => m.Key == _proj.Material);
            mat.SelectedIndex = mi >= 0 ? mi : PartMaterials.Length - 1;
            mat.SelectionChanged += (s, e) =>
            {
                var m = PartMaterials[mat.SelectedIndex];
                _proj.Material = m.Key;
                if (!double.IsNaN(m.Value)) { cte = m.Value; if (_proj.TempCorrection) apply(); else { _proj.Cte = cte; RebuildWorkspacePanel(); } }
            };
            c.Children.Add(Ui.Row(Lbl("Material"), mat, Ui.NumBox(cte, v => { cte = v; _proj.Material = "Custom"; if (_proj.TempCorrection) apply(); else _proj.Cte = v; }, 55), Ui.Text(" µm/m·K", 12, null, Ui.Muted)));
            double k = Project.TemperatureScale(temp, cte);
            c.Children.Add(Ui.Text(_proj.TempCorrection
                ? string.Format(Ui.CI, "Lengths × {0:0.000000} ({1:+0.000;−0.000} mm per metre). The arm itself is not corrected.", _proj.TempScale, (_proj.TempScale - 1) * 1000)
                : string.Format(Ui.CI, "At {0:0.#} °C the part is {1:0.000} mm per metre {2} than at 20 °C.", temp, Math.Abs(1 / k - 1) * 1000, 1 / k >= 1 ? "longer" : "shorter"), 11.5, null, Ui.Muted));
            return c;
        }

        UIElement LeapCard()
        {
            var c = new StackPanel();
            c.Children.Add(Ui.Header("LEAPFROG (move the arm)"));
            c.Children.Add(Ui.Text("To reach more of a large part: before moving the arm, probe 3+ cone seats (drilled countersinks on the table or fixture, " +
                "spread around the part) as Point features with the ball seated, and tick them here. After the move, touch them again — " +
                "measuring then continues in the same coordinates.", 11.5, null, Ui.Muted));
            var cands = _proj.Features.Where(f => f.Origin == FeatureOrigin.Measured && f.Kind == FeatureKind.Point && f.RawFit != null).ToList();
            if (cands.Count == 0) c.Children.Add(Ui.Text("No Point features yet.", 11.5, null, Ui.Muted));
            foreach (var f in cands)
            {
                var ff = f;
                var cb = new CheckBox { Content = f.Name, IsChecked = _proj.LeapRefIds.Contains(f.Id), Margin = new Thickness(0, 1, 0, 1) };
                cb.Click += (s, e) => { if (cb.IsChecked == true) { if (!_proj.LeapRefIds.Contains(ff.Id)) _proj.LeapRefIds.Add(ff.Id); } else _proj.LeapRefIds.Remove(ff.Id); _dirty = true; RebuildWorkspacePanel(); };
                c.Children.Add(cb);
            }
            var b = new WrapPanel { Margin = new Thickness(0, 4, 0, 0) };
            b.Children.Add(Ui.Btn(_leapMode ? "■ Stop" : "Arm moved — touch the references", () => { if (_leapMode) CancelLeap(); else StartLeap(); }));
            if (_proj.ArmMoved)
                b.Children.Add(Ui.Btn("Back to the original position", () =>
                {
                    if (MessageBox.Show(this, "Is the arm back where it was when the project started?", "Leapfrog", MessageBoxButton.YesNo) != MessageBoxResult.Yes) return;
                    _proj.ArmTransform = null; _proj.ArmMoveNote = ""; _dirty = true; RebuildWorkspacePanel(); RebuildScene();
                }));
            c.Children.Add(b);
            c.Children.Add(Ui.Text(_proj.ArmMoved ? "Arm position: " + _proj.ArmMoveNote : "Arm position: original", 11.5, System.Windows.FontWeights.SemiBold));
            return c;
        }
    }
}
