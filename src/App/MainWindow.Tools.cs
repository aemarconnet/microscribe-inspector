using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using MicroScribe.Core;
using MicroScribe.Driver;
using MicroScribe.Export;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    public enum Tool { Select, Distance, Angle, Diameter, Coordinate, SketchArea, TraceArea, ProbeTable, Form, Parallel, Perpendicular, Angularity, Position, Coaxial, Profile, ModelPick }

    public sealed partial class MainWindow
    {
        Tool _tool = Tool.Select;
        readonly List<EntityRef> _picks = new List<EntityRef>();
        readonly List<Vec3> _pickMarks = new List<Vec3>();
        readonly List<Vec3> _sketch = new List<Vec3>();      // machine frame, on the table plane
        Vec3? _sketchHover;
        readonly List<Vec3> _tablePts = new List<Vec3>();
        int _selectedDim;
        readonly Dictionary<Tool, ToggleButton> _toolButtons = new Dictionary<Tool, ToggleButton>();

        // plane resizing
        Vec3 _dragC, _dragU; double _dragW, _dragH;
        Feature _dragFeature;

        // SpaceMouse
        SpaceMouseInput _space;
        readonly Stopwatch _frameClock = Stopwatch.StartNew();
        double _lastFrame;

        // ------------------------------------------------------------------ tool strip

        FrameworkElement BuildToolStrip()
        {
            var sp = new StackPanel { Margin = new Thickness(10, 0, 0, 0), VerticalAlignment = VerticalAlignment.Center, HorizontalAlignment = HorizontalAlignment.Left };
            Action<Tool, string, string> add = (t, glyph, tip) =>
            {
                var b = new ToggleButton
                {
                    Content = new TextBlock { Text = glyph, FontSize = 15, FontWeight = FontWeights.SemiBold, HorizontalAlignment = HorizontalAlignment.Center },
                    Width = 38, Height = 34, Margin = new Thickness(0, 2, 0, 2), ToolTip = tip, Focusable = false,
                    Background = new SolidColorBrush(Color.FromArgb(170, 40, 46, 58)), Foreground = Brushes.White,
                    BorderBrush = new SolidColorBrush(Color.FromArgb(120, 255, 255, 255))
                };
                b.Click += (s, e) => SetTool(_tool == t && t != Tool.Select ? Tool.Select : t);
                _toolButtons[t] = b;
                sp.Children.Add(b);
            };
            add(Tool.Select, "⬉", "Select (Esc)");
            add(Tool.Distance, "↔", "Measure distance: click two entities (features or probed points)");
            add(Tool.Angle, "∠", "Measure angle: two planes/lines/axes, or three points (vertex in the middle)");
            add(Tool.Diameter, "Ø", "Measure diameter: click a circle, cylinder, sphere or cone");
            add(Tool.Coordinate, "XYZ", "Show coordinates: click a feature or probed point");
            add(Tool.SketchArea, "⬠", "Sketch the work area outline on the table");
            return sp;
        }

        void SetTool(Tool t)
        {
            if (_tool == Tool.SketchArea && t != Tool.SketchArea && _sketch.Count > 0 && _sketch.Count < 3) _sketch.Clear();
            _tool = t;
            _picks.Clear(); _pickMarks.Clear();
            _view.ClickMode = t == Tool.SketchArea;
            foreach (var kv in _toolButtons) kv.Value.IsChecked = kv.Key == t;
            if (t == Tool.SketchArea)
            {
                _sketch.Clear();
                _view.SetView("top");
                FitWorkOrFeatures();
            }
            UpdatePrompt();
            RebuildToolLayer();
            RebuildModelLayer();
        }

        string ToolPrompt()
        {
            switch (_tool)
            {
                case Tool.Distance: return _picks.Count == 0 ? "Distance — click the first entity (a feature or one of its probed points)" : "Distance — click the second entity  (from " + DimensionSolver.RefName(_proj, _picks[0]) + ")";
                case Tool.Angle:
                    {
                        int need = DimensionSolver.PicksNeeded(DimKind.Angle, _proj, _picks);
                        if (_picks.Count == 0) return "Angle — click a plane, line or axis… or a probed point to start a 3-point angle";
                        return need == 3 ? string.Format("Angle — 3-point: click point {0} of 3 (the 2nd is the vertex)", _picks.Count + 1) : "Angle — click the second plane/line/axis";
                    }
                case Tool.Diameter: return "Diameter — click a circle, cylinder, sphere or cone";
                case Tool.Coordinate: return "Coordinates — click a feature or a probed point";
                case Tool.SketchArea: return string.Format("Work area — click points on the table to outline it ({0} so far). Click the first point, double-click or press Enter to close. Backspace = undo, Shift = no snap", _sketch.Count);
                case Tool.TraceArea: return string.Format("Work area — touch the table along the outline and press pedal 1 / Space for each corner ({0} so far). Enter = done", _sketch.Count);
                case Tool.ProbeTable: return string.Format("Table plane — touch the table in 3+ spread-out places, pedal 1 / Space each ({0} so far). Enter = done", _tablePts.Count);
                case Tool.Form: return "Form — click a plane (flatness), line (straightness), circle (roundness), cylinder (cylindricity), cone or sphere";
                case Tool.Position: return "Position — click a hole, pin, circle, sphere or point; then enter its nominal position (part frame)";
                case Tool.Profile: return "Profile — click a feature: its probed points are compared with the nominal model";
                case Tool.Parallel:
                case Tool.Perpendicular:
                case Tool.Angularity:
                case Tool.Coaxial:
                    {
                        string n = _tool == Tool.Parallel ? "Parallelism" : _tool == Tool.Perpendicular ? "Perpendicularity" : _tool == Tool.Angularity ? "Angularity" : "Coaxiality";
                        string what = _tool == Tool.Coaxial ? "axis (cylinder, cone, line or circle)" : "plane or axis";
                        return _picks.Count == 0 ? n + " — click the toleranced " + what : n + " — click the datum " + what + "  (for " + DimensionSolver.RefName(_proj, _picks[0]) + ")";
                    }
            }
            return null;
        }

        /// <summary>The measurement a click tool makes.</summary>
        static DimKind ToolKind(Tool t)
        {
            switch (t)
            {
                case Tool.Angle: return DimKind.Angle;
                case Tool.Diameter: return DimKind.Diameter;
                case Tool.Coordinate: return DimKind.Coordinate;
                case Tool.Form: return DimKind.Form;
                case Tool.Parallel: return DimKind.Parallelism;
                case Tool.Perpendicular: return DimKind.Perpendicularity;
                case Tool.Angularity: return DimKind.Angularity;
                case Tool.Position: return DimKind.Position;
                case Tool.Coaxial: return DimKind.Coaxiality;
                case Tool.Profile: return DimKind.Profile;
            }
            return DimKind.Distance;
        }

        // ------------------------------------------------------------------ picking

        void OnScenePicked(PickTag tag, Vec3 hit, Point screen)
        {
            if (_tool == Tool.SketchArea) { SketchClick(screen); return; }
            if (tag == null) return;
            if (tag.Anchor != null)
            {
                var r = _proj.AlignmentRecipe;
                r.Origin = OriginMode.BoundingBox;
                r.AnchorX = (Anchor)tag.Anchor[0]; r.AnchorY = (Anchor)tag.Anchor[1]; r.AnchorZ = (Anchor)tag.Anchor[2];
                Status("Origin moved to bounding box " + r.AnchorX + "/" + r.AnchorY + "/" + r.AnchorZ);
                Changed();
                return;
            }
            if (tag.Model) { if (_tool == Tool.ModelPick) ModelPicked(hit); return; }
            if (tag.DimId != 0) { _selectedDim = tag.DimId; _tabs.SelectedIndex = 1; RebuildMeasurePanel(); RebuildDims(); return; }
            var f = _proj.Find(tag.FeatureId);
            if (f == null) return;
            int pi = tag.PointIndex;
            if (pi == -2)
            {
                // nearest probed point to the hit
                Mat4 inv = DisplayMatrix.RigidInverse();
                Vec3 hm = inv.TransformPoint(hit);
                double best = double.MaxValue; pi = -1;
                for (int i = 0; i < f.Points.Count; i++)
                {
                    double d = (f.Points[i].Position - hm).Length;
                    if (d < best) { best = d; pi = i; }
                }
            }
            if (_tool == Tool.Select)
            {
                SelectFeature(f);
                return;
            }
            // measuring tools: a point marker picks that probed point, the geometry picks the feature
            PickEntity(new EntityRef { FeatureId = f.Id, PointIndex = pi });
        }

        void PickEntity(EntityRef e)
        {
            DimKind kind = ToolKind(_tool);
            // features with no geometry (curves being traced) can only provide points
            var f = _proj.Find(e.FeatureId);
            if (e.PointIndex < 0 && (f == null || f.Geometry == null)) { Status("That feature has no solved geometry yet"); return; }
            if ((kind == DimKind.Diameter || Gdt.IsGdt(kind)) && e.PointIndex >= 0) e.PointIndex = -1;
            _picks.Add(e);
            string n;
            var g = DimensionSolver.Resolve(_proj, e, out n);
            if (g != null) _pickMarks.Add(Constructions.KeyPoint(g));
            int need = DimensionSolver.PicksNeeded(kind, _proj, _picks);
            if (_picks.Count >= need)
            {
                var d = new Dimension { Id = _proj.NextDimId++, Kind = kind, Refs = _picks.ToList() };
                InitNominal(d);
                var r = DimensionSolver.Evaluate(_proj, d);
                _picks.Clear(); _pickMarks.Clear();
                if (!r.Ok) { Status("Can't measure that: " + r.Error); Sounds.Warn(); }
                else
                {
                    d.Name = (Gdt.IsGdt(kind) ? Gdt.Name(kind, DimensionSolver.Resolve(_proj, d.Refs[0], out n)) : kind.ToString()) + " " + d.Id;
                    _proj.Dimensions.Add(d);
                    _selectedDim = d.Id;
                    _dirty = true;
                    Status(DimText(d, r));
                    Sounds.Good();
                    if (_tabs.SelectedIndex != 1) _tabs.SelectedIndex = 1;
                    RebuildMeasurePanel();
                    RebuildDims();
                }
            }
            UpdatePrompt();
            RebuildToolLayer();
        }

        /// <summary>Position and angularity need a nominal to be evaluated: start from the measured value, rounded.</summary>
        void InitNominal(Dimension d)
        {
            if (d.Kind == DimKind.Position)
            {
                string n;
                var g = DimensionSolver.Resolve(_proj, d.Refs[0], out n);
                if (g == null) return;
                Vec3 k = _proj.AlignmentMatrix.TransformPoint(Constructions.KeyPoint(g));
                d.NX = Math.Round(k.X, 1); d.NY = Math.Round(k.Y, 1); d.NZ = Math.Round(k.Z, 1);
            }
            else if (d.Kind == DimKind.Angularity)
            {
                d.Nominal = 45;
                var r = DimensionSolver.Evaluate(_proj, d);
                if (r.Ok) d.Nominal = Math.Round(r.Extra.First(x => x.Key.StartsWith("Angle")).Value * 2) / 2;
            }
        }

        string DimText(Dimension d, DimResult r)
        {
            if (!r.Ok) return r.Error;
            if (d.Kind == DimKind.Coordinate)
            {
                Vec3 p = DisplayMatrix.TransformPoint(r.A);
                return r.Title + ": X " + Ui.L(p.X) + "  Y " + Ui.L(p.Y) + "  Z " + Ui.L(p.Z) + " " + Ui.U;
            }
            return r.Title + " = " + (r.IsAngle ? r.Value.ToString("0.000", Ui.CI) + "°" : Ui.LU(r.Value));
        }

        void SelectFeature(Feature f)
        {
            _selected = f;
            RebuildFeatureList();
            RebuildScene();
            RebuildFeaturePanel();
            RebuildRelationsPanel();
            if (_tabs.SelectedIndex != 2 && _tabs.SelectedIndex != 3 && _tabs.SelectedIndex != 4) _tabs.SelectedIndex = 0;
        }

        // ------------------------------------------------------------------ work area sketch

        Vec3 TableOrigin { get { return _s.WorkArea.TableOrigin; } }
        Vec3 TableNormal { get { return _s.WorkArea.TableNormal; } }

        void TableAxes(out Vec3 x, out Vec3 y)
        {
            Vec3 n = TableNormal;
            x = Vec3.UnitX - n * Vec3.UnitX.Dot(n);
            if (x.Length < 0.3) x = n.AnyPerpendicular();
            x = x.Normalized(); y = n.Cross(x);
        }

        /// <summary>Screen point → point on the table plane (machine frame), snapped.</summary>
        bool ScreenToTable(Point screen, out Vec3 p)
        {
            Vec3 o, d;
            _view.Unproject(screen, out o, out d);
            Mat4 inv = DisplayMatrix.RigidInverse();
            o = inv.TransformPoint(o); d = inv.TransformVector(d);
            if (!SceneView.RayPlane(o, d, TableOrigin, TableNormal, out p)) return false;
            double snap = _s.SketchSnap;
            if (snap > 0 && (Keyboard.Modifiers & ModifierKeys.Shift) == 0)
            {
                Vec3 x, y; TableAxes(out x, out y);
                Vec3 w = p - TableOrigin;
                double sx = Math.Round(w.Dot(x) / snap) * snap, sy = Math.Round(w.Dot(y) / snap) * snap;
                p = TableOrigin + x * sx + y * sy;
            }
            return true;
        }

        void SketchClick(Point screen)
        {
            Vec3 p;
            if (!ScreenToTable(screen, out p)) return;
            if (_sketch.Count >= 3)
            {
                Point first;
                if (_view.Project(DisplayMatrix.TransformPoint(_sketch[0]), out first) && (first - screen).Length < 12) { FinishSketch(); return; }
            }
            if (_sketch.Count > 0 && (_sketch[_sketch.Count - 1] - p).Length < 1e-6) return;
            _sketch.Add(p);
            Sounds.Blip();
            UpdatePrompt();
            RebuildToolLayer();
        }

        void SketchHover(Point screen)
        {
            if (_tool != Tool.SketchArea) return;
            Vec3 p;
            _sketchHover = ScreenToTable(screen, out p) ? (Vec3?)p : null;
            RebuildToolLayer();
        }

        void FinishSketch()
        {
            if (_sketch.Count < 3) { Status("Need at least 3 points for the outline"); return; }
            var wa = _s.WorkArea;
            wa.Outline = _sketch.Select(q => new[] { q.X, q.Y, q.Z }).ToList();
            wa.Enabled = true;
            _sketch.Clear();
            _sketchHover = null;
            Sounds.Success();
            Status("Work area set: " + wa.Outline.Count + " corners, height " + Ui.LU(wa.Height) + " — Fit now frames this volume");
            _s.Save();
            SetTool(Tool.Select);
            _tabs.SelectedIndex = 4;
            RebuildWorkspacePanel();
            RebuildScene();
            _view.SetView("iso");
            FitView();
        }

        // stylus-traced outline and table plane probing (called from CapturePressed)
        bool ToolCapture(ArmSample s)
        {
            if (_tool == Tool.TraceArea)
            {
                Vec3 n = TableNormal;
                Vec3 p = s.Tip - n * (s.Tip - TableOrigin).Dot(n);
                _sketch.Add(p);
                Sounds.Blip();
                UpdatePrompt(); RebuildToolLayer();
                return true;
            }
            if (_tool == Tool.ProbeTable)
            {
                _tablePts.Add(s.Tip); // tip centre; the ball radius is removed when the plane is fitted
                Sounds.Blip();
                UpdatePrompt(); RebuildToolLayer();
                return true;
            }
            return false;
        }

        bool ToolFinish()
        {
            if (_tool == Tool.SketchArea || _tool == Tool.TraceArea) { FinishSketch(); return true; }
            if (_tool == Tool.ProbeTable)
            {
                if (_tablePts.Count < 3) { Status("Touch the table in at least 3 places"); return true; }
                var pl = Fitter.FitPlane(_tablePts.ToArray());
                Vec3 n = pl.Normal.Z < 0 ? -pl.Normal : pl.Normal;
                // tip centre sits one ball radius above the table
                Vec3 o = pl.Origin - n * _s.CurrentTip.BallRadius;
                _s.WorkArea.TableOrigin = o; _s.WorkArea.TableNormal = n;
                // re-project an existing outline onto the new plane
                _s.WorkArea.Outline = _s.WorkArea.OutlinePoints.Select(q => q - n * (q - o).Dot(n)).Select(q => new[] { q.X, q.Y, q.Z }).ToList();
                Status(string.Format(Ui.CI, "Table plane probed: flatness {0}, tilt {1:0.00}° from the arm's base", Ui.LU(pl.Stats.Form), Constructions.AngleBetween(n, Vec3.UnitZ, true) * 180 / Math.PI));
                Sounds.Success();
                _tablePts.Clear();
                _s.Save();
                SetTool(Tool.Select);
                RebuildWorkspacePanel();
                RebuildScene();
                return true;
            }
            if (_tool != Tool.Select) { SetTool(Tool.Select); return true; }
            return false;
        }

        bool ToolUndo()
        {
            if ((_tool == Tool.SketchArea || _tool == Tool.TraceArea) && _sketch.Count > 0) { _sketch.RemoveAt(_sketch.Count - 1); UpdatePrompt(); RebuildToolLayer(); return true; }
            if (_tool == Tool.ProbeTable && _tablePts.Count > 0) { _tablePts.RemoveAt(_tablePts.Count - 1); UpdatePrompt(); RebuildToolLayer(); return true; }
            if (_picks.Count > 0) { _picks.RemoveAt(_picks.Count - 1); if (_pickMarks.Count > 0) _pickMarks.RemoveAt(_pickMarks.Count - 1); UpdatePrompt(); RebuildToolLayer(); return true; }
            return false;
        }

        /// <summary>Temporary graphics for the active tool (picked entities, sketch rubber band, probed table points).</summary>
        void RebuildToolLayer()
        {
            if (_view == null) return;
            Mat4 M = DisplayMatrix;
            var g = new Model3DGroup();
            var labels = new List<Label3D>();
            var mb = new MeshBuilder();
            foreach (var p in _pickMarks) mb.Sphere(M.TransformPoint(p), _marker * 2.4, 12, 8);
            if (mb.VertexCount > 0) g.Children.Add(SceneView.Model(mb, Materials.Emissive(Color.FromRgb(250, 204, 21))));
            if (_tool == Tool.SketchArea || _tool == Tool.TraceArea)
            {
                var sk = new MeshBuilder();
                var pts = _sketch.Select(M.TransformPoint).ToList();
                foreach (var p in pts) sk.Sphere(p, _marker * 1.6, 10, 6);
                sk.Polyline(pts, _marker * 0.35);
                if (_sketchHover.HasValue && pts.Count > 0)
                {
                    Vec3 hv = M.TransformPoint(_sketchHover.Value);
                    sk.Tube(pts[pts.Count - 1], hv, _marker * 0.25, 6);
                    if (pts.Count >= 2) sk.Tube(hv, pts[0], _marker * 0.15, 6);
                    labels.Add(new Label3D { Position = hv, Text = Ui.LU((_sketchHover.Value - _sketch[_sketch.Count - 1]).Length), Background = Ui.Frozen(Color.FromArgb(200, 30, 64, 175)) });
                }
                if (_sketchHover.HasValue) sk.Sphere(M.TransformPoint(_sketchHover.Value), _marker * 1.2, 10, 6);
                if (sk.VertexCount > 0) g.Children.Add(SceneView.Model(sk, Materials.Emissive(Color.FromRgb(56, 189, 248))));
            }
            if (_tool == Tool.ProbeTable && _tablePts.Count > 0)
            {
                var tp = new MeshBuilder();
                foreach (var p in _tablePts) tp.Sphere(M.TransformPoint(p), _marker * 1.8, 10, 6);
                g.Children.Add(SceneView.Model(tp, Materials.Emissive(Color.FromRgb(56, 189, 248))));
            }
            AddGhost(g, labels, M);
            AddSketchOverlay(g, M);
            if (_modelPts.Count > 0)
            {
                // model points of a point-pair alignment, numbered; the probed partners in orange
                var mp = new MeshBuilder();
                Mat4 X = ModelToDisplay;
                for (int i = 0; i < _modelPts.Count; i++)
                {
                    Vec3 q = X.TransformPoint(_modelPts[i]);
                    mp.Sphere(q, _marker * 2, 12, 8);
                    labels.Add(new Label3D { Position = q, Text = (i + 1).ToString(), Bold = true, Background = Ui.Frozen(Color.FromArgb(220, 124, 58, 237)) });
                }
                g.Children.Add(SceneView.Model(mp, Materials.Emissive(Color.FromRgb(167, 139, 250))));
                var pp = new MeshBuilder();
                foreach (var q in _pairPts) pp.Sphere(M.TransformPoint(q), _marker * 1.8, 10, 6);
                if (pp.VertexCount > 0) g.Children.Add(SceneView.Model(pp, Materials.Emissive(Color.FromRgb(251, 146, 60))));
            }
            g.Freeze();
            _view.SetLayer("tool", g, null, labels);
        }

        // ------------------------------------------------------------------ plane resize handles

        void OnDragStarted(PickTag t)
        {
            if (t.Handle != "plane") return;
            var f = _proj.Find(t.FeatureId);
            var pl = f != null ? f.Geometry as PlanePrim : null;
            if (pl == null) return;
            PlaneBounds.Get(f, pl, out _dragC, out _dragU, out _dragW, out _dragH);
            _dragFeature = f;
        }

        void OnDragging(PickTag t, Vec3 o, Vec3 d)
        {
            var f = _dragFeature;
            if (f == null) return;
            var pl = f.Geometry as PlanePrim;
            if (pl == null) return;
            Mat4 inv = DisplayMatrix.RigidInverse();
            o = inv.TransformPoint(o); d = inv.TransformVector(d);
            Vec3 hit;
            if (!SceneView.RayPlane(o, d, pl.Origin, pl.Normal, out hit))
                if (!SceneView.RayPlane(o, d, pl.Origin, -pl.Normal, out hit)) return;
            Vec3 v = pl.Normal.Cross(_dragU);
            double s = (hit - _dragC).Dot(_dragU), q = (hit - _dragC).Dot(v);
            double step = Ui.Inch ? 25.4 / 16 : 1; // snap edges to 1 mm / 1/16"
            if ((Keyboard.Modifiers & ModifierKeys.Shift) == 0) { s = Math.Round(s / step) * step; q = Math.Round(q / step) * step; }
            double umin = -_dragW / 2, umax = _dragW / 2, vmin = -_dragH / 2, vmax = _dragH / 2;
            switch (t.HandleIndex)
            {
                case 0: umin = s; vmin = q; break;
                case 1: umax = s; vmin = q; break;
                case 2: umax = s; vmax = q; break;
                case 3: umin = s; vmax = q; break;
                case 4: vmin = q; break;
                case 5: umax = s; break;
                case 6: vmax = q; break;
                case 7: umin = s; break;
            }
            if (umax - umin < 1) return;
            if (vmax - vmin < 1) return;
            PlaneBounds.Set(f, _dragC + _dragU * ((umin + umax) / 2) + v * ((vmin + vmax) / 2), _dragU, umax - umin, vmax - vmin);
            _dirty = true;
            RebuildScene();
            Status(string.Format("{0}: {1} × {2} {3}  (Shift = no snap)", f.Name, Ui.L(umax - umin), Ui.L(vmax - vmin), Ui.U));
        }

        void OnDragEnded(PickTag t)
        {
            if (_dragFeature != null) { RebuildFeaturePanel(); RebuildFeatureList(); }
            _dragFeature = null;
        }

        // ------------------------------------------------------------------ SpaceMouse

        void InitSpaceMouse()
        {
            try
            {
                var src = (HwndSource)PresentationSource.FromVisual(this);
                _space = new SpaceMouseInput();
                if (!_space.Register(src)) { _space = null; return; }
                _space.ButtonChanged += (b, down) =>
                {
                    if (!down || !_s.SpaceMouse.Enabled) return;
                    if (b == 1) FitView();          // right button: Fit
                    else if (b == 0) { _view.SetView("iso"); FitView(); } // left button: isometric
                };
                CompositionTarget.Rendering += OnRenderFrame;
            }
            catch { _space = null; }
        }

        void OnRenderFrame(object sender, EventArgs e)
        {
            double now = _frameClock.Elapsed.TotalSeconds;
            double dt = Math.Min(0.05, now - _lastFrame);
            _lastFrame = now;
            if (_space == null || !_s.SpaceMouse.Enabled || !_space.Active || !IsActive) return;
            var cfg = _s.SpaceMouse;
            Func<int, double> ax = i =>
            {
                double v = _space.Axes[i];
                if (Math.Abs(v) < cfg.Deadzone) return 0;
                v = (v - Math.Sign(v) * cfg.Deadzone) / (350 - cfg.Deadzone);
                return Math.Max(-1.5, Math.Min(1.5, v)) * cfg.Sensitivity;
            };
            Func<bool, double> sign = rev => rev ? -1 : 1;
            // Raw HID axes 0..5 = Tx, Ty, Tz, Rx, Ry (roll, unused), Rz. Default signs as confirmed on the
            // user's SpaceNavigator / SpaceMouse; each motion can be reversed in the Workspace tab.
            _view.Navigate(
                -ax(0) * sign(cfg.ReversePanX),
                -ax(1) * sign(cfg.ReverseZoom),
                ax(2) * sign(cfg.ReversePanY),
                -ax(3) * sign(cfg.ReverseTilt),
                ax(5) * sign(cfg.ReverseSpin), dt);
        }
    }
}
