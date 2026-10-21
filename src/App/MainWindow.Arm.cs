using System;
using System.Collections.Generic;
using System.Linq;
using System.Media;
using System.Threading;
using System.Windows;
using System.Windows.Media;
using MicroScribe.Core;
using MicroScribe.Driver;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    public sealed partial class MainWindow
    {
        IArmDevice _arm;
        Feature _active;          // feature being measured
        bool _activeAuto;         // auto-detect type on finish
        Feature _selected;

        // recent samples for capture averaging (written by the device thread)
        readonly object _histLock = new object();
        readonly LinkedList<ArmSample> _hist = new LinkedList<ArmSample>();
        bool _scanning;           // curve mode with pedal held
        long _captureStart = -1;  // Stopwatch ticks of a pending averaged capture
        ArmSample _captureSample;
        Action<Vec3, Vec3> _captureSink;  // receives the averaged point (feature, arm check)
        int _buttons;
        // hands-free capture: armed until a point is taken, re-armed once the stylus moves away
        bool _dwellArmed = true;
        Vec3 _dwellAt, _dwellDir;

        // ------------------------------------------------------------------ connection

        void ToggleConnect()
        {
            if (_arm != null && (_arm.State == ArmState.Connected || _arm.State == ArmState.Connecting)) { DisconnectArm(); return; }
            ConnectArm();
        }

        void ConnectArm()
        {
            DisconnectArm();
            string port = (_portBox.Text ?? "").Trim().ToUpperInvariant();
            _s.Port = port;
            IArmDevice dev;
            if (_s.Driver == "simulator")
            {
                var sim = new SimulatedArm();
                sim.BallRadius = _s.CurrentTip.BallRadius;
                dev = sim;
            }
            else
            {
                var a = new MicroScribeArm(port, _s.Baud);
                a.Log = msg => Dispatcher.BeginInvoke(new Action(() => Status(msg)));
                dev = a;
            }
            _arm = dev;
            dev.SampleReceived += OnSample;
            dev.ButtonChanged += (b, down, s) => Dispatcher.BeginInvoke(new Action(() => OnButton(b, down, s)));
            dev.StateChanged += (st, msg) => Dispatcher.BeginInvoke(new Action(() => OnArmState(st, msg)));
            _connectBtn.IsEnabled = false;
            _statusText.Text = "Connecting...";
            ApplyTip();
            var t = new Thread(() =>
            {
                try { dev.Connect(); }
                catch (Exception ex)
                {
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        _connectBtn.IsEnabled = true;
                        _statusText.Text = "Connection failed";
                        _ledConn.Fill = Brushes.IndianRed;
                        MessageBox.Show(this, ex.Message + "\n\nCheck: arm powered on, cable, COM port (" + port + "), and that no other program holds the port.",
                            "Connect", MessageBoxButton.OK, MessageBoxImage.Warning);
                        _arm = null;
                        RebuildArmPanel();
                    }));
                }
            }) { IsBackground = true };
            t.Start();
        }

        void DisconnectArm()
        {
            var a = _arm;
            _arm = null;
            if (a != null) { try { a.Disconnect(); } catch { } a.Dispose(); }
            _ledConn.Fill = Brushes.Gray;
            _connectBtn.Content = "Connect";
            _connectBtn.IsEnabled = true;
            _statusText.Text = "Not connected";
            _view.SetLayer("live", null);
            RebuildArmPanel();
        }

        void OnArmState(ArmState st, string msg)
        {
            _connectBtn.IsEnabled = true;
            switch (st)
            {
                case ArmState.Connected:
                    _ledConn.Fill = Brushes.LimeGreen; _connectBtn.Content = "Disconnect"; _statusText.Text = msg;
                    Status("Connected: " + msg);
                    CheckReminder();
                    break;
                case ArmState.Connecting: _ledConn.Fill = Brushes.Gold; _statusText.Text = msg; break;
                case ArmState.Error: _ledConn.Fill = Brushes.OrangeRed; _statusText.Text = msg; Status(msg); break;
                default: _ledConn.Fill = Brushes.Gray; _connectBtn.Content = "Connect"; _statusText.Text = "Not connected"; break;
            }
            RebuildArmPanel();
        }

        void ApplyTip()
        {
            var tip = _s.CurrentTip;
            if (_arm != null) _arm.TipOffset = tip.Offset;
            var sim = _arm as SimulatedArm;
            if (sim != null) sim.BallRadius = tip.BallRadius;
            RebuildArmPanel();
        }

        void Rehome()
        {
            if (_arm == null || _arm.State != ArmState.Connected) { Status("Connect first"); return; }
            if (MessageBox.Show(this, "Place the arm in its HOME position (resting in the base cradle exactly as at power-up), then press OK to re-zero all encoders.",
                "Rehome", MessageBoxButton.OKCancel, MessageBoxImage.Information) != MessageBoxResult.OK) return;
            _arm.Rehome();
            Status("Encoders re-zeroed at the home position");
        }

        // ------------------------------------------------------------------ samples & pedals

        void OnSample(ArmSample s)
        {
            lock (_histLock)
            {
                _hist.AddLast(s);
                while (_hist.Count > 400) _hist.RemoveFirst();
            }
        }

        void OnButton(int b, bool down, ArmSample s)
        {
            if (b == 0) _buttons = down ? _buttons | 1 : _buttons & ~1;
            if (b == 1) _buttons = down ? _buttons | 2 : _buttons & ~2;
            PedalAction act = b == 0 ? _s.Pedal1 : b == 1 ? _s.Pedal2 : PedalAction.None;
            if (act == PedalAction.Capture)
            {
                if (down) CapturePressed(s);
                else if (_scanning)
                {
                    _scanning = false;
                    // the traced curve/section is complete: solve once and give feedback
                    if (_active != null && (_active.Kind == FeatureKind.Curve || _active.Kind == FeatureKind.Surface))
                    {
                        FeatureSolver.Solve(_proj, _active);
                        Sounds.Good();
                        if (_active.Kind == FeatureKind.Surface)
                            Status(string.Format("Curve {0} traced — hold pedal 1 to trace the next one, pedal 2 / Enter to build the surface", _section + 1));
                        RebuildFeatureList(); RebuildScene(); RebuildFeaturePanel(); UpdatePrompt();
                    }
                }
            }
            else if (down)
            {
                if (act == PedalAction.Finish) { if (!ToolFinish() && !CheckFinish() && !ModelPickFinish()) FinishMeasure(); }
                else if (act == PedalAction.Undo) { if (!ToolUndo() && !CheckUndo() && !ModelPickUndo()) UndoPoint(); }
            }
        }

        int _section; // current traced curve of a freeform surface

        /// <summary>
        /// Pedal 1 / Space: capture one point (averaged), or start tracing a curve / surface section.
        /// <paramref name="dwell"/>: hands-free capture, <paramref name="at"/> is already averaged.
        /// </summary>
        void CapturePressed(ArmSample at = null, bool dwell = false)
        {
            var s = at ?? (_arm != null ? _arm.Latest : null);
            if (s == null) { Status("No arm data — connect the arm or the simulator"); return; }
            if (_pivotActive) { CapturePivot(); return; }
            if (ToolCapture(s)) return;
            if (_check != CheckMode.None) { Capture(s, at, dwell, CheckCapture); return; }
            if (_pairProbing) { Capture(s, at, dwell, PairCapture); return; }
            if (_active == null)
            {
                // Quick start: capturing without an active feature starts an Auto feature.
                StartMeasure(null);
            }
            if (_active.Kind == FeatureKind.Curve || _active.Kind == FeatureKind.Surface)
            {
                if (dwell) return; // tracing needs the pedal held
                // pedal held = trace; each new press on a freeform surface starts a new curve
                if (_active.Kind == FeatureKind.Surface && at != null && _active.Points.Any(p => p.Section == _section)) _section++;
                _scanning = at != null; // keyboard: single point; pedal: hold to scan
                AddPoint(s.Tip, s.Direction, true);
                return;
            }
            Capture(s, at, dwell, (p, d) => AddPoint(p, d));
        }

        /// <summary>One point for <paramref name="sink"/>: at once (keyboard, hands-free) or averaged after a pedal press.</summary>
        void Capture(ArmSample s, ArmSample at, bool dwell, Action<Vec3, Vec3> sink)
        {
            if (dwell || _s.CaptureAverageMs <= 0 || at == null) { sink(s.Tip, s.Direction); return; }
            _captureStart = s.Timestamp;
            _captureSample = s;
            _captureSink = sink;
        }

        /// <summary>Freeform surface: start the next traced curve (keyboard N).</summary>
        void NextSection()
        {
            if (_active == null || _active.Kind != FeatureKind.Surface) return;
            if (_active.Points.Any(p => p.Section == _section)) _section++;
            Sounds.Blip();
            Status("Next curve (" + (_section + 1) + ") — Space adds points to it");
            UpdatePrompt();
        }

        /// <summary>Called from the UI timer: completes pending averaged captures.</summary>
        void CompleteCapture()
        {
            if (_captureStart < 0) return;
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            double ms = (now - _captureStart) * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
            if (ms < _s.CaptureAverageMs) return;
            List<ArmSample> win;
            lock (_histLock) win = _hist.Where(h => h.Timestamp >= _captureStart).ToList();
            _captureStart = -1;
            var sink = _captureSink ?? ((a, b) => AddPoint(a, b));
            _captureSink = null;
            if (win.Count == 0) win.Add(_captureSample);
            double moved = ArmCheck.Movement(win.Select(h => h.Tip).ToList());
            if (_s.CaptureMotionLimit > 0 && moved > _s.CaptureMotionLimit)
            {
                Sounds.Warn();
                Status(string.Format("Not captured: the stylus moved {0} during the capture — hold it still and press again (limit {1}, Arm & tip tab)",
                    Ui.LU(moved), Ui.LU(_s.CaptureMotionLimit)));
                return;
            }
            Vec3 p = Vec3.Zero, d = Vec3.Zero;
            foreach (var h in win) { p += h.Tip; d += h.Direction; }
            sink(p / win.Count, d.Normalized());
        }

        /// <summary>
        /// Hands-free capture: while measuring, holding the stylus still for DwellMs takes a point; the next
        /// one is taken only after the tip has moved away (or the stylus was swung round, for seat tests).
        /// </summary>
        void DwellTick(ArmSample s)
        {
            if (_s.DwellMs <= 0 || s == null || _scanning || _captureStart >= 0) return;
            bool wanted = _pivotActive || _check != CheckMode.None || _pairProbing || _tool == Tool.TraceArea || _tool == Tool.ProbeTable
                || (_active != null && _active.Kind != FeatureKind.Curve && _active.Kind != FeatureKind.Surface);
            if (!wanted) { _dwellArmed = true; return; }
            if (!_dwellArmed)
            {
                if ((s.Tip - _dwellAt).Length > _s.DwellRearm || Constructions.AngleBetween(s.Direction, _dwellDir, false) > 10 * Math.PI / 180)
                    _dwellArmed = true;
                return;
            }
            long from = s.Timestamp - (long)(_s.DwellMs / 1000.0 * System.Diagnostics.Stopwatch.Frequency);
            List<ArmSample> win;
            lock (_histLock)
            {
                if (_hist.Count == 0 || _hist.First.Value.Timestamp > from) return; // not held long enough yet
                win = _hist.Where(h => h.Timestamp >= from).ToList();
            }
            Vec3 tip, dir;
            if (win.Count < 3 || !ArmCheck.IsStill(win.Select(h => h.Tip).ToList(), win.Select(h => h.Direction).ToList(), _s.DwellRadius, 1.0, out tip, out dir)) return;
            _dwellArmed = false; _dwellAt = tip; _dwellDir = dir;
            CapturePressed(new ArmSample { Tip = tip, Direction = dir, StylusFrame = s.StylusFrame, Buttons = s.Buttons, Timestamp = s.Timestamp }, true);
        }

        void AddPoint(Vec3 tip, Vec3 dir, bool traced = false)
        {
            if (_active == null) return;
            var mp = new MeasuredPoint(tip, dir, _s.CurrentTip.BallRadius) { Section = _active.Kind == FeatureKind.Surface ? _section : 0 };
            _active.Points.Add(mp);
            _dirty = true;
            if (_activeAuto && _active.Points.Count >= 3)
            {
                // live classification so the user sees what the points look like
                Primitive best;
                var pts = _active.ActivePoints.ToList();
                _active.Kind = Calibration.Classify(pts.Select(p => p.Position).ToArray(), pts.Select(p => p.Direction).ToArray(), 0.05, out best);
            }
            if (!_scanning) FeatureSolver.Solve(_proj, _active);
            if (!_scanning) JudgePoint(_active, mp, traced);
            UpdatePrompt();
            RebuildFeatureList();
            RebuildScene();
            if (_active == _selected) RebuildFeaturePanel();
            PlanAfterPoint();
        }

        /// <summary>Plays the happy chime for a point that agrees with the fit, a low "hmm" for an outlier.</summary>
        void JudgePoint(Feature f, MeasuredPoint mp, bool traced)
        {
            if (traced || f.Kind == FeatureKind.Curve || f.Kind == FeatureKind.Surface) { Sounds.Blip(); return; }
            int n = f.ActivePoints.Count();
            int need = _activeAuto ? 3 : Fitter.MinPoints(f.Kind);
            if (f.RawFit == null || n < need) { Sounds.Blip(); return; }
            if (n == need) { Sounds.Good(); return; }   // feature just became defined
            double r = Math.Abs(f.RawFit is PointPrim ? (mp.Position - ((PointPrim)f.RawFit).Position).Length : f.RawFit.SignedDistance(mp.Position));
            double rms = f.RawFit.Stats != null ? f.RawFit.Stats.Rms : 0;
            if (r <= Math.Max(_s.GoodPointTolerance, 3 * rms))
            {
                Sounds.Good();
                Status(string.Format("{0}: point {1} OK ({2} from the fit)", f.Name, n, Ui.LU(r)));
            }
            else
            {
                Sounds.Warn();
                Status(string.Format("{0}: point {1} is {2} off the fit — Backspace to undo, or exclude it later", f.Name, n, Ui.LU(r)));
            }
        }

        void ScanTick(ArmSample s)
        {
            if (!_scanning || _active == null || s == null) return;
            if (_active.Kind != FeatureKind.Curve && _active.Kind != FeatureKind.Surface) return;
            var last = _active.Points.LastOrDefault(p => p.Section == (_active.Kind == FeatureKind.Surface ? _section : 0));
            if (last == null || (last.Position - s.Tip).Length >= Math.Max(0.05, _s.CurveSpacing))
            {
                _active.Points.Add(new MeasuredPoint(s.Tip, s.Direction, _s.CurrentTip.BallRadius) { Section = _active.Kind == FeatureKind.Surface ? _section : 0 });
                _dirty = true;
                _active.Geometry = _active.Kind == FeatureKind.Curve ? null : _active.Geometry;
                UpdatePrompt();
                RebuildScene();
                RebuildFeatureList();
            }
        }

        // ------------------------------------------------------------------ measure session

        void StartMeasure(FeatureKind? kind)
        {
            if (_active != null) FinishMeasure(true);
            if (_tool != Tool.Select) SetTool(Tool.Select);
            var f = _proj.NewFeature(kind ?? FeatureKind.Plane, kind.HasValue ? (kind.Value == FeatureKind.Surface ? "Freeform" : null) : "Feature");
            f.ExportMargin = _s.DefaultExportMargin;
            _active = f;
            _activeAuto = !kind.HasValue;
            _selected = f;
            _section = 0;
            UpdatePrompt();
            RebuildAll();
            if (_arm == null) Status("Not connected — connect the arm (or choose Simulator) to capture points. Space also captures.");
        }

        void ResumeMeasure(Feature f)
        {
            if (f.Origin == FeatureOrigin.Constructed) return;
            if (_active != null && _active != f) FinishMeasure(true);
            _active = f; _activeAuto = false; _selected = f;
            _section = f.Points.Count > 0 ? f.Points.Max(p => p.Section) : 0;
            if (f.Kind == FeatureKind.Surface && f.Points.Count > 0) _section++; // new curve when resuming a freeform surface
            UpdatePrompt();
            RebuildAll();
        }

        void FinishMeasure() { FinishMeasure(false); }

        void FinishMeasure(bool silent)
        {
            var f = _active;
            if (f == null) return;
            _scanning = false;
            _captureStart = -1;
            int n = f.ActivePoints.Count();
            if (_activeAuto && n > 0)
            {
                Primitive best;
                var pts = f.ActivePoints.ToList();
                FeatureKind k = Calibration.Classify(pts.Select(p => p.Position).ToArray(), pts.Select(p => p.Direction).ToArray(), 0.05, out best);
                f.Kind = k;
                // rename "Feature N" to "<Kind> N"
                f.Name = UniqueName(k.ToString());
                Status("Auto-detected: " + k);
            }
            if (n == 0 && _plan == null) _proj.Features.Remove(f);
            else if (n < Fitter.MinPoints(f.Kind) && !silent)
                Status(f.Name + " needs at least " + Fitter.MinPoints(f.Kind) + " points (kept, you can add more later)");
            _active = null;
            _activeAuto = false;
            UpdatePrompt();
            Changed();
            if (n > 0 && !silent)
            {
                bool ok = f.Geometry != null && FitQuality.Warnings(f.Kind, f.RawFit, f.ActivePoints.Select(q => q.Position).ToList()).Count == 0;
                if (ok) Sounds.Success(); else Sounds.Blip();
                if (f.Geometry == null && f.FitError != null) Status(f.Name + ": " + f.FitError);
            }
            if (!silent) PlanAfterFinish(f);
        }

        string UniqueName(string baseName)
        {
            int i = 1;
            while (_proj.Features.Any(x => x.Name == baseName + " " + i && x != _active)) i++;
            return baseName + " " + i;
        }

        void CancelMeasure()
        {
            if (_active == null) return;
            if (PlanRunning) { Status("A plan is running — use Skip step, Redo step or Stop plan (Plan tab)"); return; }
            var f = _active;
            _active = null; _scanning = false; _captureStart = -1;
            if (f.Points.Count == 0 || MessageBox.Show(this, "Discard " + f.Name + " and its " + f.Points.Count + " points?", "Cancel", MessageBoxButton.YesNo) == MessageBoxResult.Yes)
                _proj.Features.Remove(f);
            UpdatePrompt();
            Changed();
        }

        void UndoPoint()
        {
            var f = _active ?? _selected;
            if (f == null || f.Points.Count == 0) return;
            f.Points.RemoveAt(f.Points.Count - 1);
            Status("Removed last point of " + f.Name);
            Changed();
            UpdatePrompt();
        }

        void UpdatePrompt()
        {
            string tp = ToolPrompt() ?? CheckPrompt() ?? ModelPrompt();
            if (tp != null)
            {
                _prompt.Text = tp + "     (Esc = stop)";
                _promptBar.Visibility = Visibility.Visible;
                _promptButtons.Visibility = Visibility.Collapsed;
                return;
            }
            if (_active == null)
            {
                _promptBar.Visibility = Visibility.Collapsed;
                _promptButtons.Visibility = Visibility.Collapsed;
                return;
            }
            int n = _active.ActivePoints.Count();
            string kind = _activeAuto ? "Auto-detect" : _active.Kind == FeatureKind.Surface ? "freeform surface" : _active.Kind.ToString();
            int need = _activeAuto ? 3 : Fitter.MinPoints(_active.Kind);
            string hint;
            if (_active.Kind == FeatureKind.Curve) hint = "hold pedal 1 and trace";
            else if (_active.Kind == FeatureKind.Surface)
            {
                int curves = _active.Points.Select(p => p.Section).Distinct().Count();
                hint = string.Format("{0} curve{1} traced — hold pedal 1 to trace each curve across the surface (N = next curve), 2+ needed", curves, curves == 1 ? "" : "s");
            }
            else hint = n < need ? string.Format("{0} more point{1} needed", need - n, need - n == 1 ? "" : "s") : "add more points to improve the fit";
            if (PlanRunning && _active == StepFeature)
            {
                int planned = InspectionPlan.PlannedPoints(PlanSteps[_planStep]);
                hint = string.Format("plan step {0} of {1}", _planStep + 1, PlanSteps.Count) + (planned > 0 ? string.Format(", {0} of {1} points", n, planned) : ", trace then pedal 2")
                    + (string.IsNullOrWhiteSpace(_active.Notes) ? "" : " — " + _active.Notes.Trim());
            }
            _prompt.Text = string.Format("Measuring {0} ({1})  —  {2} point{3}, {4}.   Pedal 1/Space: capture · Pedal 2/Enter: finish",
                _active.Name, kind, n, n == 1 ? "" : "s", hint);
            _promptBar.Visibility = Visibility.Visible;
            _promptButtons.Visibility = Visibility.Visible;
        }

        // ------------------------------------------------------------------ UI timer (≈30 Hz)

        void UiTick()
        {
            CompleteCapture();
            var s = _arm != null ? _arm.Latest : null;
            ScanTick(s);
            DwellTick(s);
            bool inch = _s.InchDisplay;
            int buttons = s != null ? s.Buttons : _buttons;
            _led1.Fill = (buttons & 1) != 0 ? Brushes.OrangeRed : Brushes.LightGray;
            _led2.Fill = (buttons & 2) != 0 ? Brushes.OrangeRed : Brushes.LightGray;
            if (s == null)
            {
                _readout.Text = "X  —\nY  —\nZ  —";
                _deviation.Text = "";
            }
            else
            {
                Mat4 m = DisplayMatrix;
                Vec3 p = m.TransformPoint(s.Tip);
                string u = Ui.Unit(inch);
                _readout.Text = string.Format("X {0,11}\nY {1,11}\nZ {2,11}  {3}", Ui.Mm(p.X, inch), Ui.Mm(p.Y, inch), Ui.Mm(p.Z, inch), u);
                // live deviation from the selected feature (inspection mode)
                var sel = _selected;
                if (sel != null && sel.Geometry != null && sel != _active && !(sel.Geometry is PointPrim))
                {
                    // surface deviation of the probe ball's contact point (tip centre minus ball radius)
                    double d = sel.Geometry.SignedDistance(s.Tip);
                    double r = _s.CurrentTip.BallRadius;
                    if (r > 0 && !(sel.Geometry is LinePrim)) d = d >= 0 ? d - r : d + r;
                    _deviation.Text = string.Format("Δ {0}: {1}{2} {3}", sel.Name, d >= 0 ? "+" : "", Ui.Mm(d, inch), u);
                }
                else if (_proj.NominalModel != null)
                {
                    // stylus against the nominal model (ball surface)
                    double d = ModelFit.Deviation(_proj.NominalModel, _proj.AlignmentMatrix, s.Tip, _s.CurrentTip.BallRadius);
                    _deviation.Text = string.Format("Δ model: {0}{1} {2}", d >= 0 ? "+" : "", Ui.Mm(d, inch), u);
                }
                else _deviation.Text = _arm != null ? string.Format("{0:0} Hz", _arm.SampleRate) : "";
                UpdateLive(s);
            }
            _frameText.Text = "Frame: " + (_proj.AlignmentDescription ?? "machine") + (_proj.AlignmentError != null ? "  ⚠ " + _proj.AlignmentError : "");
            ArmTick(s);
        }
    }
}
