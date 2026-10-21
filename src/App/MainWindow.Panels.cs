using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Media;
using MicroScribe.Core;
using MicroScribe.Driver;
using MicroScribe.Export;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    public sealed partial class MainWindow
    {
        StackPanel _featurePanel, _relationsPanel, _alignPanel, _armPanel, _workPanel;
        TextBlock _armLive, _spaceLive;
        int _relA, _relB, _relC;
        bool _keepBoxExpanded;
        int _armTickCount;

        // pivot calibration state
        bool _pivotActive;
        readonly List<Mat4> _pivotFrames = new List<Mat4>();
        TextBlock _pivotText;

        StackPanel BuildFeaturePanel() { _featurePanel = new StackPanel(); return _featurePanel; }
        StackPanel BuildRelationsPanel() { _relationsPanel = new StackPanel(); return _relationsPanel; }
        StackPanel BuildAlignmentPanel() { _alignPanel = new StackPanel(); return _alignPanel; }
        StackPanel BuildArmPanel() { _armPanel = new StackPanel(); RebuildArmPanel(); return _armPanel; }
        StackPanel BuildWorkspacePanel() { _workPanel = new StackPanel(); RebuildWorkspacePanel(); return _workPanel; }

        UnitFmt Fmt { get { return new UnitFmt(Ui.Inch); } }

        static TextBlock Lbl(string t) { return new TextBlock { Text = t, Width = 130, VerticalAlignment = VerticalAlignment.Center }; }
        static TextBlock Short(string t) { return new TextBlock { Text = t, Width = 92, VerticalAlignment = VerticalAlignment.Center }; }

        static readonly Brush PassBrush = Ui.Frozen(Color.FromRgb(22, 163, 74)), FailBrush = Ui.Frozen(Color.FromRgb(220, 38, 38));

        TolStatus TolStatusOf(Dimension d) { return Tolerance.Evaluate(d, DimensionSolver.Evaluate(_proj, d), _proj.AlignmentMatrix).Status; }

        /// <summary>Nominal / tolerance editor shown under the selected measurement.</summary>
        UIElement ToleranceEditor(Dimension d, Action refresh)
        {
            var sp = new StackPanel { Margin = new Thickness(0, 4, 0, 4) };
            var r = DimensionSolver.Evaluate(_proj, d);
            if (!r.Ok) return sp;
            bool ang = r.IsAngle;
            Action changed = () => { _dirty = true; refresh(); RebuildDims(); };
            if (Gdt.IsGdt(d.Kind)) { GdtEditor(sp, d, changed); return sp; }
            Action fromMeasured = () =>
            {
                if (d.Kind == DimKind.Coordinate)
                {
                    Vec3 q = _proj.AlignmentMatrix.TransformPoint(r.A);
                    d.NX = Math.Round(q.X, 3); d.NY = Math.Round(q.Y, 3); d.NZ = Math.Round(q.Z, 3);
                }
                else d.Nominal = Math.Round(r.Value, 3);
            };
            var on = new CheckBox { Content = "Nominal value and tolerance", IsChecked = d.Toleranced, Margin = new Thickness(0, 2, 0, 2) };
            on.Click += (s, e) =>
            {
                d.Toleranced = on.IsChecked == true;
                if (d.Toleranced && d.Nominal == 0 && d.NX == 0 && d.NY == 0 && d.NZ == 0) { fromMeasured(); d.TolPlus = 0.5; d.TolMinus = -0.5; }
                _dirty = true; RebuildMeasurePanel(); RebuildDims();
            };
            sp.Children.Add(on);
            if (!d.Toleranced) return sp;
            Func<double, Action<double>, double, TextBox> box = (v, set, w) =>
                ang ? Ui.NumBox(v, x => { set(x); changed(); }, w) : Ui.LenBox(v, x => { set(x); changed(); }, w);
            Func<UIElement> unit = () => ang ? (UIElement)Ui.Text(" °", 12, null, Ui.Muted) : Ui.UnitLabel();
            if (d.Kind == DimKind.Coordinate)
                sp.Children.Add(Ui.Row(Short("Nominal XYZ"), box(d.NX, x => d.NX = x, 58), box(d.NY, x => d.NY = x, 58), box(d.NZ, x => d.NZ = x, 58), unit()));
            else sp.Children.Add(Ui.Row(Short("Nominal"), box(d.Nominal, x => d.Nominal = x, 80), unit()));
            sp.Children.Add(Ui.Row(Short("Tolerance + / −"), box(d.TolPlus, x => d.TolPlus = Math.Abs(x), 62),
                box(-d.TolMinus, x => d.TolMinus = -Math.Abs(x), 62), unit()));
            sp.Children.Add(Ui.Row(Ui.Btn("Measured → nominal", () => { fromMeasured(); _dirty = true; RebuildMeasurePanel(); RebuildDims(); })));
            if (!ang)
            {
                double hb = Tolerance.HalfBand(d), err = _s.MeasuringError;
                if (ToleranceTooTight(hb))
                    sp.Children.Add(Ui.Text(string.Format("⚠ ±{0} is tighter than 4× the arm's measuring error (±{1}) — a pass/fail here is not reliable.",
                        Ui.LU(hb), Ui.LU(err)), 11, null, Brushes.DarkOrange));
                else if (double.IsNaN(err))
                    sp.Children.Add(Ui.Text("Run an arm check (Arm & tip tab) to see whether this tolerance can be judged reliably.", 11, null, Ui.Muted));
            }
            return sp;
        }

        /// <summary>Geometric tolerance: nominal angle / position where needed, and the tolerance zone.</summary>
        void GdtEditor(StackPanel sp, Dimension d, Action changed)
        {
            if (d.Kind == DimKind.Angularity)
                sp.Children.Add(Ui.Row(Short("Nominal angle"), Ui.NumBox(d.Nominal, x => { d.Nominal = x; changed(); }, 80), Ui.Text(" °", 12, null, Ui.Muted)));
            if (d.Kind == DimKind.Position)
                sp.Children.Add(Ui.Row(Short("Nominal XYZ"), Ui.LenBox(d.NX, x => { d.NX = x; changed(); }, 58), Ui.LenBox(d.NY, x => { d.NY = x; changed(); }, 58),
                    Ui.LenBox(d.NZ, x => { d.NZ = x; changed(); }, 58), Ui.UnitLabel()));
            if (d.Kind == DimKind.Form)
            {
                var mz = new CheckBox { Content = "Minimum zone (narrowest zone instead of least squares)", IsChecked = d.MinZone, Margin = new Thickness(0, 2, 0, 2) };
                mz.Click += (s, e) => { d.MinZone = mz.IsChecked == true; changed(); };
                sp.Children.Add(mz);
            }
            var on = new CheckBox { Content = "Tolerance zone", IsChecked = d.Toleranced, Margin = new Thickness(0, 2, 0, 2) };
            on.Click += (s, e) =>
            {
                d.Toleranced = on.IsChecked == true;
                if (d.Toleranced && d.TolPlus <= 0.1 + 1e-9) d.TolPlus = 0.5;
                _dirty = true; RebuildMeasurePanel(); RebuildDims();
            };
            sp.Children.Add(on);
            if (!d.Toleranced) return;
            string z = d.Kind == DimKind.Position || d.Kind == DimKind.Coaxiality ? "Ø zone" : "Zone width";
            sp.Children.Add(Ui.Row(Short(z), Ui.LenBox(d.TolPlus, x => { d.TolPlus = Math.Abs(x); changed(); }, 80), Ui.UnitLabel()));
            double hb = Tolerance.HalfBand(d);
            if (ToleranceTooTight(hb))
                sp.Children.Add(Ui.Text(string.Format("⚠ A {0} zone is tighter than 4× the arm's measuring error (±{1}) — a pass/fail here is not reliable.",
                    Ui.LU(d.TolPlus), Ui.LU(_s.MeasuringError)), 11, null, Brushes.DarkOrange));
        }

        /// <summary>Fixed size and fixed direction for the fit.</summary>
        UIElement ConstraintCard(Feature f)
        {
            var sp = new StackPanel { Margin = new Thickness(0, 4, 0, 0) };
            if (ConstrainedFit.CanFixSize(f.Kind))
            {
                bool cone = f.Kind == FeatureKind.Cone;
                var fx = new CheckBox { Content = cone ? "Fixed included angle" : "Fixed diameter", IsChecked = f.FixSize, VerticalAlignment = VerticalAlignment.Center, Width = 130 };
                fx.Click += (s, e) =>
                {
                    f.FixSize = fx.IsChecked == true;
                    if (f.FixSize && f.FixedSize <= 0 && f.Geometry != null)
                    {
                        var g = f.Geometry;
                        f.FixedSize = cone ? Math.Round(2 * Math.Abs(((ConePrim)g).HalfAngle) * 180 / Math.PI, 1)
                            : Math.Round(2 * (g is CirclePrim ? ((CirclePrim)g).Radius : g is SpherePrim ? ((SpherePrim)g).Radius : ((CylinderPrim)g).Radius), 2);
                    }
                    Changed();
                };
                sp.Children.Add(Ui.Row(fx, cone ? (UIElement)Ui.NumBox(f.FixedSize, v => { f.FixedSize = Math.Max(0.01, v); Changed(); }, 80) : Ui.LenBox(f.FixedSize, v => { f.FixedSize = Math.Max(0.001, v); Changed(); }, 80),
                    cone ? (UIElement)Ui.Text(" °", 12, null, Ui.Muted) : Ui.UnitLabel()));
            }
            if (ConstrainedFit.CanFixDirection(f.Kind))
            {
                var rel = new ComboBox { Width = 130 };
                rel.Items.Add("Free direction"); rel.Items.Add("Parallel to"); rel.Items.Add("Perpendicular to");
                rel.SelectedIndex = f.DirectionRelation == "parallel" ? 1 : f.DirectionRelation == "perpendicular" ? 2 : 0;
                rel.SelectionChanged += (s, e) => { f.DirectionRelation = rel.SelectedIndex == 1 ? "parallel" : rel.SelectedIndex == 2 ? "perpendicular" : ""; Changed(); };
                Func<Feature, bool> datum = x => x != f && x.Geometry != null && (x.Geometry is PlanePrim || x.Geometry is LinePrim || x.Geometry is CylinderPrim || x.Geometry is ConePrim);
                sp.Children.Add(Ui.Row(rel, FeatureCombo(f.DirectionDatumId, id => { f.DirectionDatumId = id; Changed(); }, datum, "(datum)")));
                if (f.Kind == FeatureKind.Cone && f.DirectionRelation == "perpendicular") sp.Children.Add(Ui.Text("Cones: only an axis along the datum is supported.", 11, null, Brushes.DarkOrange));
            }
            sp.Children.Add(Ui.Text("Use when the part is known to have this size or direction (a dowel hole, a face square to the base). If the constraint does not suit the part, the residuals grow — check the form value.", 11, null, Ui.Muted));
            bool active = (f.FixSize && f.FixedSize > 0) || !string.IsNullOrEmpty(f.DirectionRelation);
            return new Expander { Header = active ? "Fit constraints (active)" : "Fit constraints", IsExpanded = active, Content = sp, FontWeight = active ? FontWeights.SemiBold : FontWeights.Normal };
        }

        /// <summary>How the ball radius was applied, in words.</summary>
        static string CompensationNote(Feature f)
        {
            switch (f.Kind)
            {
                case FeatureKind.Plane: return "plane offset";
                case FeatureKind.Point: return "along the stylus";
                case FeatureKind.Line: return "along the stylus, perpendicular to the line";
                case FeatureKind.Surface: return "along the surface normal, on the stylus side";
                case FeatureKind.Cloud: return "along the local surface normal, on the stylus side";
                case FeatureKind.Curve:
                    return f.Compensation == CompensationSide.Auto ? "along the stylus, perpendicular to the curve"
                         : "in the curve's plane, " + (f.Compensation == CompensationSide.Outside ? "toward" : "away from") + " the enclosed area";
                default:
                    return FeatureSolver.IsOutside(f.RawFit, f.ActivePoints.ToList(), f.Compensation) ? "outside / boss" : "inside / bore";
            }
        }

        // ================================================================== Feature tab

        void RebuildFeaturePanel()
        {
            var p = _featurePanel;
            if (p == null) return;
            p.Children.Clear();
            var f = _selected;
            if (f == null)
            {
                p.Children.Add(Ui.Card(Ui.Text("Select a feature in the list or the 3D view, or start measuring with the buttons on the left.\n\n" +
                    "Tip: press the capture pedal without choosing a type — the points are classified automatically when you finish.", 12.5, null, Ui.Muted)));
                return;
            }
            // header
            var name = new TextBox { Text = f.Name, FontSize = 15, FontWeight = FontWeights.SemiBold, BorderThickness = new Thickness(0, 0, 0, 1), Margin = new Thickness(0, 0, 0, 2) };
            Action rename = () => { if (name.Text.Trim().Length > 0 && name.Text != f.Name) { f.Name = name.Text.Trim(); _dirty = true; RebuildFeatureList(); RebuildScene(); RebuildMeasurePanel(); RebuildAlignmentPanel(); } };
            name.LostFocus += (s, e) => rename();
            name.KeyDown += (s, e) => { if (e.Key == System.Windows.Input.Key.Enter) rename(); };
            var hdr = new DockPanel();
            var ic = Ui.Icon(f.Kind, 24); ic.Margin = new Thickness(0, 0, 8, 0);
            DockPanel.SetDock(ic, Dock.Left);
            hdr.Children.Add(ic);
            hdr.Children.Add(name);
            p.Children.Add(hdr);
            p.Children.Add(Ui.Text((f.Kind == FeatureKind.Surface ? "Freeform surface" : f.Kind.ToString()) + (f.Origin == FeatureOrigin.Constructed ? " (constructed: " + f.Construction + ")" : "") +
                " · frame: " + (_showMachineFrame ? "machine" : "part") + " · " + Ui.U, 11.5, null, Ui.Muted));

            // results
            var g = GeoUtil.Transform(f.Geometry, DisplayMatrix);
            var res = new TextBlock { FontFamily = Ui.Mono, FontSize = 12, TextWrapping = TextWrapping.Wrap };
            res.Text = string.Join("\n", CadExport.Describe(g, f, Fmt));
            var resCard = new StackPanel();
            resCard.Children.Add(Ui.Header("RESULTS"));
            resCard.Children.Add(res);
            if (f.Origin == FeatureOrigin.Measured && !Fitter.IsTraced(f.Kind))
                foreach (var warn in FitQuality.Warnings(f.Kind, f.RawFit, f.ActivePoints.Select(q => q.Position).ToList()))
                    resCard.Children.Add(Ui.Text("⚠ " + warn, 11.5, null, Brushes.DarkOrange));
            if (f.Origin == FeatureOrigin.Measured && f.RawFit != null && f.Geometry != null && f.ActivePoints.Any(q => q.ProbeRadius > 0))
                resCard.Children.Add(Ui.Text(f.Compensation == CompensationSide.None
                    ? "Ball compensation off — results are at the ball centre."
                    : string.Format("Ball compensation {0} applied ({1}).", Ui.LU(f.ActivePoints.Average(q => q.ProbeRadius)), CompensationNote(f)), 11, null, Ui.Muted));
            p.Children.Add(Ui.Card(resCard, new Thickness(0, 8, 0, 8)));

            // kind-specific options
            if (f.Kind == FeatureKind.Plane && f.Geometry is PlanePrim) p.Children.Add(Ui.Card(PlaneSizeCard(f)));
            if (f.Kind == FeatureKind.Curve) { p.Children.Add(Ui.Card(CurveCard(f))); p.Children.Add(Ui.Card(SketchCard(f))); }
            if (f.Kind == FeatureKind.Surface) p.Children.Add(Ui.Card(SurfaceCard(f)));

            if (f.Origin == FeatureOrigin.Measured && (ConstrainedFit.CanFixSize(f.Kind) || ConstrainedFit.CanFixDirection(f.Kind)))
                p.Children.Add(Ui.Card(ConstraintCard(f)));

            // options
            var opt = new StackPanel();
            opt.Children.Add(Ui.Header("OPTIONS"));
            if (f.Origin == FeatureOrigin.Constructed && f.Construction == "offset")
                opt.Children.Add(Ui.Row(Lbl("Offset"), Ui.LenBox(f.ConstructValue, v => { f.ConstructValue = v; Changed(); }), Ui.UnitLabel()));
            if (f.Origin == FeatureOrigin.Measured)
            {
                // inside/outside only means something for closed shapes and for curves traced against a wall
                bool sided = f.Kind != FeatureKind.Point && f.Kind != FeatureKind.Line && f.Kind != FeatureKind.Surface && f.Kind != FeatureKind.Cloud;
                var comp = new ComboBox { Width = 160 };
                foreach (CompensationSide v in Enum.GetValues(typeof(CompensationSide)))
                    if (sided || v == CompensationSide.Auto || v == CompensationSide.None) comp.Items.Add(v);
                comp.SelectedItem = comp.Items.Contains(f.Compensation) ? f.Compensation : CompensationSide.Auto;
                comp.SelectionChanged += (s, e) => { f.Compensation = (CompensationSide)comp.SelectedItem; Changed(); };
                opt.Children.Add(Ui.Row(Lbl("Ball compensation"), comp));
                if (f.Kind == FeatureKind.Curve)
                    opt.Children.Add(Ui.Text("Auto: toward where the stylus points (tracing over a surface). Outside / Inside: within the curve's plane, " +
                        "for tracing around a boss or inside a pocket with the ball against the wall.", 11, null, Ui.Muted));
            }
            if (f.Kind == FeatureKind.Circle || ProfileFit.IsProfile(f.Kind))
            {
                var pb = new ComboBox { Width = 160 };
                pb.Items.Add("Own best-fit plane");
                var planes = _proj.Features.Where(x => x.Kind == FeatureKind.Plane && x.Geometry != null).ToList();
                foreach (var pl in planes) pb.Items.Add(pl.Name);
                var cur = planes.FirstOrDefault(x => x.Id == f.ProjectionPlaneId);
                pb.SelectedIndex = cur != null ? planes.IndexOf(cur) + 1 : 0;
                pb.SelectionChanged += (s, e) => { f.ProjectionPlaneId = pb.SelectedIndex > 0 ? planes[pb.SelectedIndex - 1].Id : 0; Changed(); };
                opt.Children.Add(Ui.Row(Lbl("Project onto"), pb));
            }
            if (f.Kind == FeatureKind.Cylinder || f.Kind == FeatureKind.Cone)
            {
                var full = new CheckBox { Content = "Export as full 360° surface", IsChecked = f.FullRevolution, Margin = new Thickness(0, 4, 0, 2) };
                full.Click += (s, e) => { f.FullRevolution = full.IsChecked == true; Changed(); };
                opt.Children.Add(full);
            }
            if (f.Kind != FeatureKind.Plane && !Fitter.IsTraced(f.Kind) && f.Kind != FeatureKind.Point && !ProfileFit.IsProfile(f.Kind))
                opt.Children.Add(Ui.Row(Lbl("Surface margin"), Ui.LenBox(f.ExportMargin, v => { f.ExportMargin = Math.Max(0, v); Changed(); }), Ui.UnitLabel()));
            var vis = new CheckBox { Content = "Visible", IsChecked = f.Visible, Margin = new Thickness(0, 4, 12, 0) };
            vis.Click += (s, e) => { f.Visible = vis.IsChecked == true; Changed(); };
            var ex = new CheckBox { Content = "Include in export", IsChecked = f.Export, Margin = new Thickness(0, 4, 0, 0) };
            ex.Click += (s, e) => { f.Export = ex.IsChecked == true; Changed(); };
            opt.Children.Add(Ui.Row(vis, ex));
            var notes = new TextBox { Text = f.Notes, AcceptsReturn = true, MinHeight = 38, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(0, 6, 0, 0) };
            notes.LostFocus += (s, e) => { if (notes.Text != f.Notes) { f.Notes = notes.Text; _dirty = true; } };
            opt.Children.Add(Ui.Text("Notes", 11, null, Ui.Muted));
            opt.Children.Add(notes);
            p.Children.Add(Ui.Card(opt));

            // actions
            var act = new WrapPanel();
            if (f.Origin == FeatureOrigin.Measured)
            {
                act.Children.Add(Ui.Btn(f == _active ? "Finish" : f.Kind == FeatureKind.Surface ? "Trace more curves" : f.Kind == FeatureKind.Cloud ? "Scan more" : "Add points", () => { if (f == _active) FinishMeasure(); else ResumeMeasure(f); }));
                if (f.Kind == FeatureKind.Cloud)
                {
                    act.Children.Add(Ui.Btn("Thin to spacing", () => ThinCloud(f), "Exclude points closer than the curve spacing to an earlier point"));
                    act.Children.Add(Ui.Btn("Remove strays", () => RemoveStrays(f), "Exclude isolated points (stray hits)"));
                }
                if (!Fitter.IsTraced(f.Kind))
                    act.Children.Add(Ui.Btn("Exclude outliers (>2.5σ)", () => ExcludeOutliers(f)));
                act.Children.Add(Ui.Btn("Include all", () => { foreach (var q in f.Points) q.Excluded = false; Changed(); }));
                act.Children.Add(Ui.Btn("Clear points", () =>
                {
                    if (MessageBox.Show(this, "Remove all " + f.Points.Count + " points of " + f.Name + "?", "Clear", MessageBoxButton.YesNo) == MessageBoxResult.Yes)
                    { f.Points.Clear(); Changed(); }
                }));
            }
            act.Children.Add(Ui.Btn("Delete", DeleteSelected));
            act.Children.Add(Ui.Btn("Zoom to", () =>
            {
                var pts = f.Points.Select(q => DisplayMatrix.TransformPoint(q.Position)).ToList();
                if (f.Geometry != null && !(f.Geometry is SurfacePrim) && !(f.Geometry is CurvePrim)) pts.Add(DisplayMatrix.TransformPoint(Constructions.KeyPoint(f.Geometry)));
                if (f.Geometry is PlanePrim) pts.AddRange(PlaneBounds.Corners(f, (PlanePrim)f.Geometry).Select(DisplayMatrix.TransformPoint));
                if (pts.Count == 0) return;
                if (pts.Count == 1) { pts.Add(pts[0] + new Vec3(20, 20, 20)); pts.Add(pts[0] - new Vec3(20, 20, 20)); }
                _view.Fit(pts);
            }));
            p.Children.Add(act);

            // points
            if (f.Origin == FeatureOrigin.Measured && f.Points.Count > 0) p.Children.Add(Ui.Card(PointsCard(f), new Thickness(0, 8, 0, 8)));
            else if (f.Origin == FeatureOrigin.Constructed)
                p.Children.Add(Ui.Card(Ui.Text("From: " + string.Join(", ", f.SourceIds.Select(id => { var s = _proj.Find(id); return s != null ? s.Name : "(deleted)"; })), 12)));
        }

        UIElement PlaneSizeCard(Feature f)
        {
            var sp = new StackPanel();
            sp.Children.Add(Ui.Header("PLANE SIZE" + (f.CustomBounds ? " (resized)" : " (fitted to the points + margin)")));
            var pl = (PlanePrim)f.Geometry;
            Vec3 c, u; double w, h;
            PlaneBounds.Get(f, pl, out c, out u, out w, out h);
            sp.Children.Add(Ui.Row(Lbl("Width × height"),
                Ui.LenBox(w, v => { Vec3 c2, u2; double w2, h2; PlaneBounds.Get(f, pl, out c2, out u2, out w2, out h2); PlaneBounds.Set(f, c2, u2, Math.Max(0.1, v), h2); Changed(); }, 70),
                new TextBlock { Text = "×", Margin = new Thickness(4, 0, 4, 0), VerticalAlignment = VerticalAlignment.Center },
                Ui.LenBox(h, v => { Vec3 c2, u2; double w2, h2; PlaneBounds.Get(f, pl, out c2, out u2, out w2, out h2); PlaneBounds.Set(f, c2, u2, w2, Math.Max(0.1, v)); Changed(); }, 70),
                Ui.UnitLabel()));
            if (!f.CustomBounds)
                sp.Children.Add(Ui.Row(Lbl("Margin around points"), Ui.LenBox(f.ExportMargin, v => { f.ExportMargin = Math.Max(0, v); Changed(); }), Ui.UnitLabel()));
            var grow = new WrapPanel();
            Action<double> growBy = d => { Vec3 c2, u2; double w2, h2; PlaneBounds.Get(f, pl, out c2, out u2, out w2, out h2); PlaneBounds.Set(f, c2, u2, Math.Max(0.1, w2 + 2 * d), Math.Max(0.1, h2 + 2 * d)); Changed(); };
            double stepMm = Ui.Inch ? 25.4 / 8 : 5;
            grow.Children.Add(Ui.Btn("Grow +" + Ui.L(stepMm), () => growBy(stepMm), "Enlarge every edge"));
            grow.Children.Add(Ui.Btn("Shrink −" + Ui.L(stepMm), () => growBy(-stepMm), "Shrink every edge"));
            grow.Children.Add(Ui.Btn("Fit to points", () => { f.CustomBounds = false; Changed(); }, "Back to the probed extent + margin"));
            sp.Children.Add(grow);
            // extend / trim to another plane
            var others = _proj.Features.Where(x => x != f && x.Geometry is PlanePrim).ToList();
            if (others.Count > 0)
            {
                var cb = new ComboBox { Width = 150 };
                foreach (var o in others) cb.Items.Add(o.Name);
                cb.SelectedIndex = 0;
                var go = Ui.Btn("Extend / trim to", () =>
                {
                    if (cb.SelectedIndex < 0) return;
                    if (PlaneBounds.ExtendTo(f, pl, (PlanePrim)others[cb.SelectedIndex].Geometry)) { Status(f.Name + " now meets " + others[cb.SelectedIndex].Name); Changed(); }
                    else Status("The planes are parallel — nothing to extend to");
                }, "Rotate the rectangle to the intersection line and move the nearest edge onto it (so the faces meet for knitting)");
                sp.Children.Add(Ui.Row(go, cb));
            }
            sp.Children.Add(Ui.Text("Or drag the yellow corner/edge handles in the 3D view (Shift = no snapping).", 11, null, Ui.Muted));
            return sp;
        }

        UIElement CurveCard(Feature f)
        {
            var sp = new StackPanel();
            sp.Children.Add(Ui.Header("CURVE"));
            var lin = new RadioButton { Content = "Linear (polyline through the points)", IsChecked = f.CurveMode == CurveMode.Linear, GroupName = "cm" + f.Id, Margin = new Thickness(0, 2, 0, 2) };
            var spl = new RadioButton { Content = "Spline (smooth B-spline)", IsChecked = f.CurveMode == CurveMode.Spline, GroupName = "cm" + f.Id, Margin = new Thickness(0, 2, 0, 2) };
            lin.Checked += (s, e) => { if (f.CurveMode != CurveMode.Linear) { f.CurveMode = CurveMode.Linear; Changed(); } };
            spl.Checked += (s, e) => { if (f.CurveMode != CurveMode.Spline) { f.CurveMode = CurveMode.Spline; Changed(); } };
            sp.Children.Add(spl); sp.Children.Add(lin);
            if (f.CurveMode == CurveMode.Spline)
            {
                sp.Children.Add(Ui.Row(Lbl("Smoothing tolerance"), Ui.LenBox(f.CurveTolerance, v => { f.CurveTolerance = Math.Max(0, v); Changed(); }), Ui.UnitLabel()));
                sp.Children.Add(Ui.Text("The spline stays within this distance of every traced point and uses as few control points as possible. 0 = pass exactly through every point.", 11, null, Ui.Muted));
            }
            var closed = new CheckBox { Content = "Closed (join the end to the start)", IsChecked = f.Closed, Margin = new Thickness(0, 6, 0, 0) };
            closed.Click += (s, e) => { f.Closed = closed.IsChecked == true; Changed(); };
            sp.Children.Add(closed);
            return sp;
        }

        UIElement SurfaceCard(Feature f)
        {
            var sp = new StackPanel();
            sp.Children.Add(Ui.Header("FREEFORM SURFACE"));
            int curves = f.Origin == FeatureOrigin.Constructed ? f.SourceIds.Count : f.Points.Select(q => q.Section).Distinct().Count();
            sp.Children.Add(Ui.Text(curves + " traced curve(s). The surface is skinned through them in the order they were traced" +
                (f.Origin == FeatureOrigin.Constructed ? " (selected)" : "") + "; trace them roughly side by side across the surface.", 11.5, null, Ui.Muted));
            var cp = Ui.NumBox(f.SurfaceCtrlU, v => { f.SurfaceCtrlU = (int)Math.Max(0, Math.Min(60, v)); Changed(); }, 50);
            sp.Children.Add(Ui.Row(Lbl("Control pts per curve"), cp, new TextBlock { Text = "0 = auto; fewer = smoother", Foreground = Ui.Muted, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(6, 0, 0, 0) }));
            var s0 = f.Geometry as SurfacePrim;
            if (s0 != null) sp.Children.Add(Ui.Text(string.Format("Deviation of the traced points: RMS {0}, max {1}", Ui.LU(s0.Stats.Rms), Ui.LU(s0.Stats.MaxAbs)), 11.5));
            if (f.Origin == FeatureOrigin.Measured && curves > 0)
            {
                var w = new WrapPanel();
                w.Children.Add(Ui.Btn("Delete last curve", () =>
                {
                    int last = f.Points.Max(q => q.Section);
                    f.Points.RemoveAll(q => q.Section == last);
                    Changed();
                }));
                sp.Children.Add(w);
            }
            return sp;
        }

        UIElement PointsCard(Feature f)
        {
            var pp = new StackPanel();
            pp.Children.Add(Ui.Header(string.Format("POINTS ({0} used of {1}) — X Y Z and residual, {2}", f.ActivePoints.Count(), f.Points.Count, Ui.U)));
            double worst = 0;
            var resid = new double[f.Points.Count];
            bool traced = Fitter.IsTraced(f.Kind);
            for (int i = 0; i < f.Points.Count; i++)
            {
                resid[i] = f.RawFit != null ? f.RawFit.SignedDistance(f.Points[i].Position) : double.NaN;
                if (f.RawFit is PointPrim) resid[i] = (f.Points[i].Position - ((PointPrim)f.RawFit).Position).Length;
                if (!f.Points[i].Excluded && !double.IsNaN(resid[i])) worst = Math.Max(worst, Math.Abs(resid[i]));
            }
            int max = traced ? 300 : 1000;
            for (int i = 0; i < Math.Min(f.Points.Count, max); i++)
            {
                var q = f.Points[i];
                int idx = i;
                Vec3 dp = DisplayMatrix.TransformPoint(q.Position);
                var cb = new CheckBox { IsChecked = !q.Excluded, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(0, 0, 6, 0) };
                cb.Click += (s, e) => { f.Points[idx].Excluded = cb.IsChecked != true; Changed(); };
                string r = double.IsNaN(resid[i]) ? "" : (resid[i] >= 0 ? "+" : "") + Ui.L(resid[i]);
                var tb = new TextBlock
                {
                    FontFamily = Ui.Mono, FontSize = 11.5,
                    Text = string.Format(Ui.CI, "{0,3}{1} {2,9} {3,9} {4,9}  {5}", i + 1, f.Kind == FeatureKind.Surface ? "c" + (q.Section + 1) : "", Ui.L(dp.X), Ui.L(dp.Y), Ui.L(dp.Z), r),
                    Foreground = q.Excluded ? Brushes.Gray : (worst > 0 && Math.Abs(resid[i]) >= worst * 0.999 ? Brushes.Firebrick : Brushes.Black)
                };
                pp.Children.Add(Ui.Row(cb, tb));
            }
            if (f.Points.Count > max) pp.Children.Add(Ui.Text("… " + (f.Points.Count - max) + " more points", 11, null, Ui.Muted));
            return pp;
        }

        void ExcludeOutliers(Feature f)
        {
            if (f.RawFit == null || f.RawFit.Stats == null) return;
            int removed = 0;
            for (int it = 0; it < 5; it++)
            {
                FeatureSolver.Solve(_proj, f);
                if (f.RawFit == null) break;
                double sigma = f.RawFit.Stats.Rms;
                if (sigma < 1e-6) break;
                var act = f.ActivePoints.ToList();
                if (act.Count <= Fitter.MinPoints(f.Kind) + 1) break;
                var worst = act.OrderByDescending(q => Math.Abs(f.RawFit.SignedDistance(q.Position))).First();
                if (Math.Abs(f.RawFit.SignedDistance(worst.Position)) <= 2.5 * sigma) break;
                worst.Excluded = true; removed++;
            }
            Status(removed > 0 ? "Excluded " + removed + " outlier point(s) from " + f.Name : "No outliers beyond 2.5σ in " + f.Name);
            Changed();
        }

        // ================================================================== Measure tab

        ComboBox FeatureCombo(int selectedId, Action<int> changed, Func<Feature, bool> filter = null, string none = null)
        {
            var cb = new ComboBox { Width = 210, Margin = new Thickness(2) };
            var list = _proj.Features.Where(x => x.Geometry != null && (filter == null || filter(x))).ToList();
            if (none != null) cb.Items.Add(none);
            foreach (var f in list) cb.Items.Add(f.Name + "  (" + f.Kind + ")");
            int idx = list.FindIndex(x => x.Id == selectedId);
            cb.SelectedIndex = idx >= 0 ? idx + (none != null ? 1 : 0) : (none != null ? 0 : -1);
            cb.SelectionChanged += (s, e) =>
            {
                int i = cb.SelectedIndex - (none != null ? 1 : 0);
                changed(i >= 0 && i < list.Count ? list[i].Id : 0);
            };
            return cb;
        }

        void RebuildRelationsPanel() { RebuildMeasurePanel(); }

        void RebuildMeasurePanel()
        {
            var p = _relationsPanel;
            if (p == null) return;
            p.Children.Clear();

            // click tools
            var tools = new StackPanel();
            tools.Children.Add(Ui.Header("MEASURE BY CLICKING"));
            var tw = new WrapPanel();
            Action<Tool, string, string> tb = (t, text, tip) =>
            {
                var b = Ui.Btn(text, () => SetTool(_tool == t ? Tool.Select : t), tip);
                if (_tool == t) { b.Background = Ui.Accent; b.Foreground = Brushes.White; }
                tw.Children.Add(b);
            };
            tb(Tool.Distance, "↔ Distance (D)", "Two entities: features or single probed points");
            tb(Tool.Angle, "∠ Angle (A)", "Two planes/lines/axes, or three probed points (vertex in the middle)");
            tb(Tool.Diameter, "Ø Diameter (R)", "A circle, cylinder, sphere or cone");
            tb(Tool.Coordinate, "XYZ Position (X)", "Coordinates of a point, centre or feature");
            tools.Children.Add(tw);
            tools.Children.Add(Ui.Text("Click entities in the 3D view — the geometry picks the feature, a point marker picks that probed point — or click features in the list. Results are kept below and drawn in the view.", 11, null, Ui.Muted));
            tools.Children.Add(Ui.Header("GEOMETRIC TOLERANCES"));
            tw = new WrapPanel();
            tb(Tool.Form, "Form", "Flatness, straightness, roundness or cylindricity of one feature");
            tb(Tool.Parallel, "∥ Parallel", "Parallelism of a plane or axis to a datum plane or axis");
            tb(Tool.Perpendicular, "⊥ Perpendicular", "Perpendicularity of a plane or axis to a datum plane or axis");
            tb(Tool.Angularity, "∠ Angularity", "Angularity of a plane or axis to a datum, at a nominal angle");
            tb(Tool.Position, "⌖ Position", "Position of a hole, pin or point against its nominal (part frame)");
            tb(Tool.Coaxial, "◎ Coaxial", "Coaxiality of an axis to a datum axis");
            tb(Tool.Profile, "Profile", "Deviation zone of a feature's points from the nominal model (Alignment tab)");
            tools.Children.Add(tw);
            tools.Children.Add(Ui.Text("Values are tolerance-zone widths from the least-squares fit of the compensated points. The part coordinate system (Alignment tab) is the datum reference frame for Position.", 11, null, Ui.Muted));
            p.Children.Add(Ui.Card(tools));

            // dimension list
            var dl = new StackPanel();
            var hdr = new DockPanel();
            if (_proj.Dimensions.Count > 0)
            {
                var clr = Ui.Btn("Clear all", () =>
                {
                    if (MessageBox.Show(this, "Delete all " + _proj.Dimensions.Count + " measurements?", "Measurements", MessageBoxButton.YesNo) != MessageBoxResult.Yes) return;
                    _proj.Dimensions.Clear(); _dirty = true; RebuildMeasurePanel(); RebuildDims();
                });
                DockPanel.SetDock(clr, Dock.Right);
                hdr.Children.Add(clr);
            }
            hdr.Children.Add(Ui.Header("MEASUREMENTS (" + _proj.Dimensions.Count + ")"));
            dl.Children.Add(hdr);
            var summary = Ui.Text("", 11.5, FontWeights.SemiBold);
            Action updateSummary = () =>
            {
                var st = _proj.Dimensions.Select(TolStatusOf).Where(s => s != TolStatus.None).ToList();
                int fail = st.Count(s => s == TolStatus.Fail);
                summary.Text = st.Count == 0 ? "" : fail == 0 ? string.Format("✓ All {0} toleranced measurements in tolerance", st.Count)
                    : string.Format("✗ {0} of {1} toleranced measurements OUT of tolerance", fail, st.Count);
                summary.Foreground = fail == 0 ? PassBrush : FailBrush;
                summary.Visibility = st.Count == 0 ? Visibility.Collapsed : Visibility.Visible;
            };
            updateSummary();
            dl.Children.Add(summary);
            if (_proj.Dimensions.Count == 0) dl.Children.Add(Ui.Text("None yet — use the tools above.", 11.5, null, Ui.Muted));
            else dl.Children.Add(Ui.Text("Click a measurement to give it a nominal value and tolerance.", 11, null, Ui.Muted));
            Mat4 M = DisplayMatrix;
            foreach (var d in _proj.Dimensions.ToList())
            {
                var dd = d;
                var row = new DockPanel { Margin = new Thickness(0, 3, 0, 3), Background = d.Id == _selectedDim ? Ui.Frozen(Color.FromRgb(254, 249, 195)) : Brushes.Transparent };
                var del = Ui.Btn("✕", () => DeleteDimension(dd.Id), "Delete this measurement");
                DockPanel.SetDock(del, Dock.Right);
                row.Children.Add(del);
                var bar = new Border { Width = 4, Margin = new Thickness(0, 0, 6, 0), CornerRadius = new CornerRadius(2) };
                DockPanel.SetDock(bar, Dock.Left);
                row.Children.Add(bar);
                var body = new StackPanel();
                var txt = new TextBlock { FontFamily = Ui.Mono, FontSize = 11.5, TextWrapping = TextWrapping.Wrap };
                Action refresh = () =>
                {
                    txt.Text = string.Join("\n", CadExport.DescribeDimension(_proj, dd, M, Fmt));
                    var st = TolStatusOf(dd);
                    bar.Background = st == TolStatus.Pass ? PassBrush : st == TolStatus.Fail ? FailBrush : Brushes.Transparent;
                    updateSummary();
                };
                refresh();
                txt.MouseLeftButtonDown += (s, e) => { _selectedDim = dd.Id; RebuildMeasurePanel(); RebuildDims(); };
                body.Children.Add(txt);
                if (dd.Id == _selectedDim) body.Children.Add(ToleranceEditor(dd, refresh));
                row.Children.Add(body);
                dl.Children.Add(row);
            }
            p.Children.Add(Ui.Card(dl));

            // freeform surface from curve features
            var curves = _proj.Features.Where(x => x.Kind == FeatureKind.Curve && x.Geometry != null).ToList();
            var lf = new StackPanel();
            lf.Children.Add(Ui.Header("FREEFORM SURFACE FROM TRACED CURVES"));
            if (curves.Count < 2) lf.Children.Add(Ui.Text("Trace 2+ Curve features across a surface (or use Measure → Freeform, which traces the curves in one go). Then select them in the list (Ctrl+click, in order) and create the surface here.", 11.5, null, Ui.Muted));
            else
            {
                var selCurves = SelectedFeatures().Where(x => x.Kind == FeatureKind.Curve && x.Geometry != null).ToList();
                lf.Children.Add(Ui.Text(selCurves.Count >= 2
                    ? "Selected (in order): " + string.Join(" → ", selCurves.Select(x => x.Name))
                    : "Select 2+ curves in the feature list (Ctrl+click in the order they lie on the surface). All curves are used if fewer are selected.", 11.5, null, Ui.Muted));
                lf.Children.Add(Ui.Btn("Create freeform surface", () =>
                {
                    var use = selCurves.Count >= 2 ? selCurves : curves;
                    var f = _proj.NewFeature(FeatureKind.Surface, "Loft");
                    f.Origin = FeatureOrigin.Constructed; f.Construction = "loft";
                    f.SourceIds.AddRange(use.Select(x => x.Id));
                    FeatureSolver.Solve(_proj, f);
                    if (f.Geometry == null) { _proj.Features.Remove(f); Status("Surface failed: " + f.FitError); Sounds.Warn(); return; }
                    _selected = f;
                    Sounds.Success();
                    Status("Created " + f.Name + " through " + use.Count + " curves");
                    Changed();
                }));
            }
            p.Children.Add(Ui.Card(lf));

            // constructions (A/B pickers)
            if (_relA == 0 && _selected != null) _relA = _selected.Id;
            var cons = new StackPanel();
            cons.Children.Add(Ui.Header("CONSTRUCT NEW FEATURE"));
            cons.Children.Add(Ui.Row(Lbl("Feature A"), FeatureCombo(_relA, id => { _relA = id; RebuildMeasurePanel(); })));
            cons.Children.Add(Ui.Row(Lbl("Feature B"), FeatureCombo(_relB, id => { _relB = id; RebuildMeasurePanel(); })));
            var a = _proj.Find(_relA); var b2 = _proj.Find(_relB);
            if (a != null && b2 != null && a.Geometry != null && b2.Geometry != null && a != b2 && !(a.Geometry is CurvePrim) && !(b2.Geometry is CurvePrim) && !(a.Geometry is SurfacePrim) && !(b2.Geometry is SurfacePrim))
            {
                var d = Constructions.Distance(a.Geometry, b2.Geometry);
                var ang = Constructions.Angle(a.Geometry, b2.Geometry);
                string s = d.Label + ": " + Ui.LU(d.Value) + (ang != null ? string.Format(Ui.CI, "   ·   {0}: {1:0.000}°", ang.Label, ang.Value) : "");
                cons.Children.Add(Ui.Text(s, 11.5, null, Ui.Muted));
            }
            var w = new WrapPanel();
            w.Children.Add(Ui.Btn("Intersection A ∩ B", () => Construct("intersect", _relA, _relB), "Plane∩plane → line, axis/line∩plane → point, line∩line → closest point"));
            w.Children.Add(Ui.Btn("Midpoint", () => Construct("midpoint", _relA, _relB)));
            w.Children.Add(Ui.Btn("Line A → B", () => Construct("line2", _relA, _relB), "Line through the centres of A and B"));
            w.Children.Add(Ui.Btn("Axis of A", () => Construct("axis", _relA)));
            w.Children.Add(Ui.Btn("Centre of A", () => Construct("center", _relA)));
            cons.Children.Add(w);
            cons.Children.Add(Ui.Header("PLANES AND MORE (B = a plane)"));
            var w2 = new WrapPanel();
            w2.Children.Add(Ui.Btn("Mid-plane of A, B", () => Construct("midplane", _relA, _relB), "Symmetry plane between two faces; shows the width between them"));
            w2.Children.Add(Ui.Btn("Project A onto B", () => Construct("project", _relA, _relB), "Point, centre or axis projected onto plane B"));
            w2.Children.Add(Ui.Btn("Plane through A ∥ B", () => Construct("parallel", _relA, _relB), "Plane parallel to B through A's centre"));
            w2.Children.Add(Ui.Btn("Plane through axis A ⟂ B", () => Construct("perpendicular", _relA, _relB), "Plane containing line/axis A, perpendicular to plane B"));
            w2.Children.Add(Ui.Btn("Mirror A in B", () => Construct("mirror", _relA, _relB), "Mirror image of A about plane B"));
            cons.Children.Add(w2);
            var offs = Ui.Row(Ui.Btn("Offset plane A by", () => Construct("offset", _relA)), Ui.LenBox(_consValue, v => _consValue = v, 70), Ui.UnitLabel());
            cons.Children.Add(offs);
            var w3 = new WrapPanel();
            w3.Children.Add(Ui.Btn("Bolt circle through selected", () => ConstructFromSelection("boltcircle"), "Circle through the centres of 3+ features selected in the list (Ctrl+click)"));
            w3.Children.Add(Ui.Btn("Plane through selected centres", () => ConstructFromSelection("centreplane"), "Plane through the centres of 3+ selected features"));
            cons.Children.Add(w3);
            cons.Children.Add(Ui.Row(Lbl("Plane C (corner)"), FeatureCombo(_relC, id => { _relC = id; }, x => x.Kind == FeatureKind.Plane)));
            cons.Children.Add(Ui.Btn("Corner of planes A, B, C", () => Construct("intersect", _relA, _relB, _relC)));
            p.Children.Add(Ui.Card(cons));
        }

        void DeleteDimension(int id)
        {
            _proj.Dimensions.RemoveAll(d => d.Id == id);
            if (_selectedDim == id) _selectedDim = 0;
            _dirty = true;
            RebuildMeasurePanel();
            RebuildDims();
        }

        double _consValue = 10;

        void ConstructFromSelection(string op)
        {
            var sel = SelectedFeatures().Where(x => x.Geometry != null && !(x.Geometry is CurvePrim) && !(x.Geometry is SurfacePrim) && !(x.Geometry is CloudPrim)).ToList();
            if (sel.Count < 3) { Status("Select 3 or more holes / centres in the feature list first (Ctrl+click)"); return; }
            Construct(op, sel.Select(x => x.Id).ToArray());
        }

        static string ConstructionName(string op)
        {
            switch (op)
            {
                case "axis": return "Axis";
                case "line2": return "Line";
                case "intersect": return "Intersection";
                case "center": return "Centre";
                case "offset": return "Offset plane";
                case "midplane": return "Mid-plane";
                case "project": return "Projection";
                case "parallel": return "Parallel plane";
                case "perpendicular": return "Perpendicular plane";
                case "mirror": return "Mirror";
                case "boltcircle": return "Bolt circle";
                case "centreplane": return "Centre plane";
            }
            return "Midpoint";
        }

        void Construct(string op, params int[] ids)
        {
            var src = ids.Where(i => i != 0).Select(i => _proj.Find(i)).ToList();
            bool multi = op == "boltcircle" || op == "centreplane";
            int need = op == "axis" || op == "center" || op == "offset" ? 1 : multi ? 3 : ids.Length;
            if (src.Count < need || src.Any(x => x == null)) { Status("Select the source feature(s) first"); return; }
            var f = _proj.NewFeature(FeatureKind.Point, ConstructionName(op));
            f.Origin = FeatureOrigin.Constructed;
            f.Construction = op;
            if (op == "offset") f.ConstructValue = _consValue;
            f.SourceIds.AddRange((multi ? src : src.Take(need)).Select(x => x.Id));
            FeatureSolver.Solve(_proj, f);
            if (f.Geometry == null)
            {
                _proj.Features.Remove(f);
                Status("Construction failed: " + f.FitError);
                return;
            }
            _selected = f;
            Status("Created " + f.Name);
            Changed();
        }

        // ================================================================== Alignment tab (centre / corner tool)

        void RebuildAlignmentPanel()
        {
            var p = _alignPanel;
            if (p == null) return;
            p.Children.Clear();
            var r = _proj.AlignmentRecipe;
            p.Children.Add(Ui.Card(ModelCard()));

            // --- orientation
            var o = new StackPanel();
            o.Children.Add(Ui.Header("1 · ORIENTATION (square the part up)"));
            Func<string, OrientationMode, RadioButton> orb = (t, m) =>
            {
                var rb = new RadioButton { Content = t, IsChecked = r.Orientation == m, Margin = new Thickness(0, 2, 0, 2), GroupName = "orient" };
                rb.Checked += (s, e) => { if (r.Orientation != m) { r.Orientation = m; Changed(); } };
                return rb;
            };
            o.Children.Add(orb("Machine axes (arm base)", OrientationMode.Machine));
            o.Children.Add(orb("From features (3-2-1: primary → Z, secondary → X)", OrientationMode.Features));
            o.Children.Add(orb("Principal axes of the measured points (auto)", OrientationMode.PrincipalAxes));
            if (r.ModelMatrix != null) o.Children.Add(orb("Fitted to the nominal model (see above)", OrientationMode.Model));
            if (r.Orientation == OrientationMode.Features)
            {
                Func<Feature, bool> hasDir = x => x.Geometry != null && !(x.Geometry is CurvePrim) && !(x.Geometry is SurfacePrim) && Constructions.Direction(x.Geometry).HasValue;
                var fp = new CheckBox { Content = "flip", IsChecked = r.FlipPrimary, VerticalAlignment = VerticalAlignment.Center };
                fp.Click += (s, e) => { r.FlipPrimary = fp.IsChecked == true; Changed(); };
                o.Children.Add(Ui.Row(Lbl("Primary (Z)"), FeatureCombo(r.PrimaryId, id => { r.PrimaryId = id; Changed(); }, hasDir), fp));
                var fs = new CheckBox { Content = "flip", IsChecked = r.FlipSecondary, VerticalAlignment = VerticalAlignment.Center };
                fs.Click += (s, e) => { r.FlipSecondary = fs.IsChecked == true; Changed(); };
                o.Children.Add(Ui.Row(Lbl("Secondary (X)"), FeatureCombo(r.SecondaryId, id => { r.SecondaryId = id; Changed(); }, x => x.Id != r.PrimaryId && !(x.Geometry is CurvePrim) && !(x.Geometry is SurfacePrim), "(none)"), fs));
                o.Children.Add(Ui.Text("Typical: top plane as primary, a side edge line (or plane ∩ plane) as secondary.", 11, null, Ui.Muted));
            }
            else if (r.Orientation == OrientationMode.PrincipalAxes)
            {
                var fp = new CheckBox { Content = "Flip Z (upside down)", IsChecked = r.FlipPrimary, Margin = new Thickness(0, 2, 0, 0) };
                fp.Click += (s, e) => { r.FlipPrimary = fp.IsChecked == true; Changed(); };
                o.Children.Add(fp);
                o.Children.Add(Ui.Text("Longest extent → X, thinnest → Z. Good for quickly squaring up a block.", 11, null, Ui.Muted));
            }
            var turns = new WrapPanel { Margin = new Thickness(0, 6, 0, 0) };
            turns.Children.Add(new TextBlock { Text = "Rotate 90°:", VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(0, 0, 6, 0) });
            foreach (var ax in new[] { "X", "Y", "Z" })
            {
                string a = ax;
                turns.Children.Add(Ui.Btn(a + "−", () => Turn(a, -1)));
                turns.Children.Add(Ui.Btn(a + "+", () => Turn(a, +1)));
            }
            o.Children.Add(turns);
            p.Children.Add(Ui.Card(o));

            // --- origin
            var g = new StackPanel();
            g.Children.Add(Ui.Header("2 · ORIGIN (centre or corner the part)"));
            if (r.Orientation == OrientationMode.Model)
                g.Children.Add(Ui.Text("Not used while the part frame is fitted to the model: the model's own origin and axes apply (rotations too).", 11.5, null, Brushes.DarkOrange));
            Func<string, OriginMode, RadioButton> grb = (t, m) =>
            {
                var rb = new RadioButton { Content = t, IsChecked = r.Origin == m, Margin = new Thickness(0, 2, 0, 2), GroupName = "origin" };
                rb.Checked += (s, e) => { if (r.Origin != m) { r.Origin = m; Changed(); } };
                return rb;
            };
            g.Children.Add(grb(r.Orientation == OrientationMode.Machine ? "Keep machine origin" : "Keep (from orientation features)", OriginMode.Keep));
            g.Children.Add(grb("At a feature (centre of circle / sphere, point, intersection...)", OriginMode.Feature));
            g.Children.Add(grb("Bounding box: corner / face centre / centre", OriginMode.BoundingBox));
            if (r.Origin == OriginMode.Feature)
                g.Children.Add(Ui.Row(Lbl("Origin feature"), FeatureCombo(r.OriginFeatureId, id => { r.OriginFeatureId = id; Changed(); }, x => !(x.Geometry is CurvePrim) && !(x.Geometry is SurfacePrim))));
            if (r.Origin == OriginMode.BoundingBox)
            {
                var grid = new Grid { Margin = new Thickness(0, 6, 0, 4) };
                grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(30) });
                for (int c = 0; c < 3; c++) grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
                var axes = new[] { "X", "Y", "Z" };
                for (int row = 0; row < 3; row++)
                {
                    grid.RowDefinitions.Add(new RowDefinition());
                    var lab = new TextBlock { Text = axes[row], FontWeight = FontWeights.SemiBold, VerticalAlignment = VerticalAlignment.Center };
                    Grid.SetRow(lab, row); grid.Children.Add(lab);
                    Anchor cur = row == 0 ? r.AnchorX : row == 1 ? r.AnchorY : r.AnchorZ;
                    for (int c = 0; c < 3; c++)
                    {
                        int rr = row; Anchor an = (Anchor)c;
                        var t = new ToggleButton { Content = new[] { "Min", "Centre", "Max" }[c], IsChecked = cur == an, Margin = new Thickness(2), Padding = new Thickness(4, 3, 4, 3), Focusable = false };
                        t.Click += (s, e) => { if (rr == 0) r.AnchorX = an; else if (rr == 1) r.AnchorY = an; else r.AnchorZ = an; Changed(); };
                        Grid.SetRow(t, row); Grid.SetColumn(t, c + 1);
                        grid.Children.Add(t);
                    }
                }
                g.Children.Add(grid);
                var presets = new WrapPanel();
                Action<string, Anchor, Anchor, Anchor> preset = (t, x, y, z) => presets.Children.Add(Ui.Btn(t, () => { r.AnchorX = x; r.AnchorY = y; r.AnchorZ = z; Changed(); }));
                preset("Centre", Anchor.Center, Anchor.Center, Anchor.Center);
                preset("Bottom centre", Anchor.Center, Anchor.Center, Anchor.Min);
                preset("Top centre", Anchor.Center, Anchor.Center, Anchor.Max);
                preset("Corner min", Anchor.Min, Anchor.Min, Anchor.Min);
                preset("Corner max", Anchor.Max, Anchor.Max, Anchor.Max);
                g.Children.Add(presets);
                g.Children.Add(Ui.Text("…or click a yellow handle on the bounding box in the 3D view.", 11, null, Ui.Muted));
                var exp = new Expander { Header = r.BoxFeatureIds.Count == 0 ? "Box from: all measured features" : "Box from: " + r.BoxFeatureIds.Count + " selected feature(s)", Margin = new Thickness(0, 6, 0, 0) };
                var list = new StackPanel { Margin = new Thickness(8, 2, 0, 2) };
                foreach (var feat in _proj.Features.Where(x => x.Origin == FeatureOrigin.Measured))
                {
                    var ff = feat;
                    bool on = r.BoxFeatureIds.Count == 0 || r.BoxFeatureIds.Contains(ff.Id);
                    var cb = new CheckBox { Content = ff.Name, IsChecked = on, Margin = new Thickness(0, 1, 0, 1) };
                    cb.Click += (s, e) =>
                    {
                        if (r.BoxFeatureIds.Count == 0) r.BoxFeatureIds.AddRange(_proj.Features.Where(x => x.Origin == FeatureOrigin.Measured).Select(x => x.Id));
                        if (cb.IsChecked == true) { if (!r.BoxFeatureIds.Contains(ff.Id)) r.BoxFeatureIds.Add(ff.Id); }
                        else r.BoxFeatureIds.Remove(ff.Id);
                        if (_proj.Features.Where(x => x.Origin == FeatureOrigin.Measured).All(x => r.BoxFeatureIds.Contains(x.Id))) r.BoxFeatureIds.Clear();
                        _keepBoxExpanded = true;
                        Changed();
                    };
                    list.Children.Add(cb);
                }
                exp.Content = list;
                exp.IsExpanded = _keepBoxExpanded;
                exp.Expanded += (s, e) => _keepBoxExpanded = true;
                exp.Collapsed += (s, e) => _keepBoxExpanded = false;
                g.Children.Add(exp);
            }
            g.Children.Add(Ui.Header("Fine offset (" + Ui.U + ", applied after the anchor)"));
            g.Children.Add(Ui.Row(new TextBlock { Text = "X", Width = 14, VerticalAlignment = VerticalAlignment.Center }, Ui.LenBox(r.OffsetX, v => { r.OffsetX = v; Changed(); }, 60),
                                  new TextBlock { Text = "Y", Width = 14, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(6, 0, 0, 0) }, Ui.LenBox(r.OffsetY, v => { r.OffsetY = v; Changed(); }, 60),
                                  new TextBlock { Text = "Z", Width = 14, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(6, 0, 0, 0) }, Ui.LenBox(r.OffsetZ, v => { r.OffsetZ = v; Changed(); }, 60)));
            p.Children.Add(Ui.Card(g));

            // --- info
            var info = new StackPanel();
            info.Children.Add(Ui.Header("PART FRAME"));
            Vec3 a0, b0; string err;
            AlignmentSolver.Solve(_proj, r, out a0, out b0, out err);
            Vec3 size = b0 - a0;
            info.Children.Add(Ui.Text(_proj.AlignmentDescription ?? "", 12));
            if (err != null) info.Children.Add(Ui.Text("⚠ " + err, 12, null, Brushes.Firebrick));
            if (AlignmentSolver.BoxPoints(_proj, r).Count > 0)
            {
                info.Children.Add(new TextBlock
                {
                    FontFamily = Ui.Mono, Margin = new Thickness(0, 6, 0, 0),
                    Text = string.Format("Box size  {0} × {1} × {2} {3}\nBox min   {4}, {5}, {6}\nBox max   {7}, {8}, {9}",
                        Ui.L(size.X), Ui.L(size.Y), Ui.L(size.Z), Ui.U, Ui.L(a0.X), Ui.L(a0.Y), Ui.L(a0.Z), Ui.L(b0.X), Ui.L(b0.Y), Ui.L(b0.Z))
                });
            }
            var mf = new CheckBox { Content = "Show machine coordinates instead", IsChecked = _showMachineFrame, Margin = new Thickness(0, 8, 0, 0) };
            mf.Click += (s, e) => { _showMachineFrame = mf.IsChecked == true; RebuildAll(); FitView(); };
            info.Children.Add(mf);
            var sb = new CheckBox { Content = "Always show bounding box", IsChecked = _showBox, Margin = new Thickness(0, 2, 0, 0) };
            sb.Click += (s, e) => { _showBox = sb.IsChecked == true; RebuildOverlay(); };
            info.Children.Add(sb);
            info.Children.Add(Ui.Btn("Reset to machine frame", () => { _proj.AlignmentRecipe = new AlignmentRecipe(); Changed(); FitView(); }));
            info.Children.Add(Ui.Text("Exports (STEP, CSV, report) use this part frame, so the part lands at the CAD origin.", 11, null, Ui.Muted));
            p.Children.Add(Ui.Card(info));
        }

        void Turn(string axis, int dir)
        {
            var r = _proj.AlignmentRecipe;
            if (axis == "X") r.TurnsX = (r.TurnsX + dir + 4) % 4;
            if (axis == "Y") r.TurnsY = (r.TurnsY + dir + 4) % 4;
            if (axis == "Z") r.TurnsZ = (r.TurnsZ + dir + 4) % 4;
            Changed();
        }

        // ================================================================== Workspace tab (work area, SpaceMouse, units)

        void RebuildWorkspacePanel()
        {
            var p = _workPanel;
            if (p == null) return;
            p.Children.Clear();
            var wa = _s.WorkArea;

            var c = new StackPanel();
            c.Children.Add(Ui.Header("WORK AREA (what Fit shows)"));
            c.Children.Add(Ui.Text(wa.IsDefined
                ? string.Format("{0} corners, max height {1}{2}", wa.Outline.Count, Ui.LU(wa.Height), wa.Enabled ? "" : " — disabled")
                : "Not set. Sketch the usable area of the table, then give a maximum height. Fit (F, the Fit button, or the SpaceMouse Fit button) then always frames this volume.", 12, wa.IsDefined ? FontWeights.SemiBold : FontWeights.Normal, wa.IsDefined ? Brushes.Black : Ui.Muted));
            var w1 = new WrapPanel { Margin = new Thickness(0, 6, 0, 0) };
            w1.Children.Add(Ui.Btn("⬠ Sketch outline on the table", () => SetTool(Tool.SketchArea), "Click the corners in the 3D view (top view, snaps to the grid)"));
            w1.Children.Add(Ui.Btn("Trace with the stylus", () => { _sketch.Clear(); SetTool(Tool.TraceArea); }, "Touch each corner of the area on the table and press pedal 1 / Space"));
            if (wa.IsDefined) w1.Children.Add(Ui.Btn("Clear", () => { wa.Outline.Clear(); wa.Enabled = false; _s.Save(); RebuildWorkspacePanel(); RebuildScene(); }));
            c.Children.Add(w1);
            var hb = Ui.LenBox(wa.Height, v => { wa.Height = Math.Max(1, v); _s.Save(); RebuildScene(); RebuildWorkspacePanel(); });
            var useTip = Ui.Btn("Use stylus height", () =>
            {
                var s = _arm != null ? _arm.Latest : null;
                if (s == null) { Status("Connect the arm first"); return; }
                wa.Height = Math.Max(1, (s.Tip - wa.TableOrigin).Dot(wa.TableNormal));
                _s.Save(); RebuildScene(); RebuildWorkspacePanel();
            }, "Hold the stylus at the highest point you will measure, then click");
            c.Children.Add(Ui.Row(Lbl("Maximum height"), hb, Ui.UnitLabel(), useTip));
            var en = new CheckBox { Content = "Fit frames the work area (otherwise all features)", IsChecked = wa.Enabled, IsEnabled = wa.IsDefined, Margin = new Thickness(0, 4, 0, 0) };
            en.Click += (s, e) => { wa.Enabled = en.IsChecked == true; _s.Save(); RebuildScene(); FitView(); };
            c.Children.Add(en);
            c.Children.Add(Ui.Row(Lbl("Sketch snap"), Ui.LenBox(_s.SketchSnap, v => _s.SketchSnap = Math.Max(0, v)), Ui.UnitLabel()));
            c.Children.Add(Ui.Header("TABLE PLANE"));
            bool probed = Math.Abs(wa.TableNormal.Z - 1) > 1e-9 || wa.TableOrigin.Length > 1e-9;
            c.Children.Add(Ui.Text(probed ? string.Format("Probed: tilt {0:0.00}° from the arm base, height {1}", Constructions.AngleBetween(wa.TableNormal, Vec3.UnitZ, true) * 180 / Math.PI, Ui.LU(wa.TableOrigin.Z))
                                           : "Assumed: the arm's base plane (z = 0, where the base sits on the table).", 11.5, null, Ui.Muted));
            var w2 = new WrapPanel();
            w2.Children.Add(Ui.Btn("Probe table plane", () => { _tablePts.Clear(); SetTool(Tool.ProbeTable); }, "Touch the table in 3+ spread-out spots with pedal 1 / Space, then Enter"));
            if (probed) w2.Children.Add(Ui.Btn("Reset to base plane", () => { wa.TableOrigin = Vec3.Zero; wa.TableNormal = Vec3.UnitZ; _s.Save(); RebuildWorkspacePanel(); RebuildScene(); }));
            c.Children.Add(w2);
            p.Children.Add(Ui.Card(c));

            // SpaceMouse
            var sm = new StackPanel();
            sm.Children.Add(Ui.Header("3DCONNEXION SPACEMOUSE"));
            var cfg = _s.SpaceMouse;
            sm.Children.Add(Ui.Text(_space == null ? "Raw input not available." :
                _space.Reports > 0 ? "Receiving input from " + _space.LastDevice : "Ready — move the cap to test (the window must be active).", 11.5, null, Ui.Muted));
            var enm = new CheckBox { Content = "Use the SpaceMouse to navigate the 3D view", IsChecked = cfg.Enabled, Margin = new Thickness(0, 4, 0, 2) };
            enm.Click += (s, e) => cfg.Enabled = enm.IsChecked == true;
            sm.Children.Add(enm);
            var sens = new Slider { Minimum = 0.2, Maximum = 3, Value = cfg.Sensitivity, Width = 160, VerticalAlignment = VerticalAlignment.Center };
            sens.ValueChanged += (s, e) => cfg.Sensitivity = sens.Value;
            sm.Children.Add(Ui.Row(Lbl("Speed"), sens));
            sm.Children.Add(Ui.Text("Reverse a direction if it moves the wrong way:", 11.5, null, Ui.Muted));
            Func<string, Func<bool>, Action<bool>, CheckBox> rev = (label, get, set) =>
            {
                var cb = new CheckBox { Content = label, IsChecked = get(), Margin = new Thickness(0, 2, 12, 2) };
                cb.Click += (s, e) => { set(cb.IsChecked == true); _s.Save(); };
                return cb;
            };
            sm.Children.Add(Ui.Row(
                rev("Left / right", () => cfg.ReversePanX, v => cfg.ReversePanX = v),
                rev("Up / down", () => cfg.ReversePanY, v => cfg.ReversePanY = v),
                rev("Zoom", () => cfg.ReverseZoom, v => cfg.ReverseZoom = v)));
            sm.Children.Add(Ui.Row(
                rev("Tilt", () => cfg.ReverseTilt, v => cfg.ReverseTilt = v),
                rev("Spin (twist)", () => cfg.ReverseSpin, v => cfg.ReverseSpin = v),
                Ui.Btn("Reverse all", () =>
                {
                    cfg.ReversePanX = !cfg.ReversePanX; cfg.ReversePanY = !cfg.ReversePanY; cfg.ReverseZoom = !cfg.ReverseZoom;
                    cfg.ReverseTilt = !cfg.ReverseTilt; cfg.ReverseSpin = !cfg.ReverseSpin;
                    _s.Save(); RebuildWorkspacePanel();
                }, "Switch between 'model follows the cap' and 'view follows the cap'")));
            _spaceLive = new TextBlock { FontFamily = Ui.Mono, FontSize = 11, Margin = new Thickness(0, 4, 0, 0), TextWrapping = TextWrapping.Wrap };
            sm.Children.Add(_spaceLive);
            sm.Children.Add(Ui.Text("Object mode: slide the cap to pan, pull/push to zoom, tilt and twist to rotate. Right button = Fit, left button = isometric view. If 3DxWare also moves the view, set its profile for MicroScribeInspector to 'no action'.", 11, null, Ui.Muted));
            p.Children.Add(Ui.Card(sm));

            // units + sounds
            var us = new StackPanel();
            us.Children.Add(Ui.Header("UNITS & SOUNDS"));
            var um = new RadioButton { Content = "Millimetres", IsChecked = !Ui.Inch, GroupName = "units", Margin = new Thickness(0, 2, 12, 2) };
            var ui2 = new RadioButton { Content = "Inches", IsChecked = Ui.Inch, GroupName = "units", Margin = new Thickness(0, 2, 0, 2) };
            um.Checked += (s, e) => SetUnits(false);
            ui2.Checked += (s, e) => SetUnits(true);
            us.Children.Add(Ui.Row(um, ui2));
            var snd = new CheckBox { Content = "Sounds (chime = good point, low tone = point off the fit)", IsChecked = _s.BeepOnCapture, Margin = new Thickness(0, 4, 0, 2) };
            snd.Click += (s, e) => { _s.BeepOnCapture = snd.IsChecked == true; Sounds.Enabled = _s.BeepOnCapture; if (Sounds.Enabled) Sounds.Good(); };
            us.Children.Add(snd);
            us.Children.Add(Ui.Row(Lbl("'Good point' limit"), Ui.LenBox(_s.GoodPointTolerance, v => _s.GoodPointTolerance = Math.Max(0.001, v)), Ui.UnitLabel(),
                Ui.Btn("▶", () => Sounds.Good(), "Play the good-point chime"), Ui.Btn("▶ done", () => Sounds.Success(), "Play the feature-finished chime"), Ui.Btn("▶ off", () => Sounds.Warn(), "Play the outlier tone")));
            us.Children.Add(Ui.Text("A point is 'good' when it lies within this limit (or 3σ, whichever is larger) of the fit through the other points.", 11, null, Ui.Muted));
            p.Children.Add(Ui.Card(us));
        }

        // ================================================================== Arm & tip tab

        void RebuildArmPanel()
        {
            var p = _armPanel;
            if (p == null) return;
            p.Children.Clear();
            var d = new StackPanel();
            d.Children.Add(Ui.Header("DEVICE"));
            if (_arm == null || _arm.State != ArmState.Connected) d.Children.Add(Ui.Text("Not connected. Choose the driver and COM port in the toolbar and press Connect. The arm must be powered on in its home position.", 12, null, Ui.Muted));
            else
            {
                d.Children.Add(Ui.Text(_arm.Description, 12, FontWeights.SemiBold));
                if (_arm.Info != null) d.Children.Add(new TextBlock { FontFamily = Ui.Mono, FontSize = 11.5, Text = _arm.Info.ToString().TrimEnd() });
                var ms = _arm as MicroScribeArm;
                if (ms != null && ms.Model != null)
                {
                    d.Children.Add(Ui.Header("KINEMATIC MODEL (from the arm's EEPROM, mm)"));
                    d.Children.Add(new TextBlock { FontFamily = Ui.Mono, FontSize = 11, Text = ms.Model.Describe().TrimEnd() });
                }
                _armLive = new TextBlock { FontFamily = Ui.Mono, FontSize = 11, Margin = new Thickness(0, 6, 0, 0) };
                d.Children.Add(_armLive);
                var sim = _arm as SimulatedArm;
                if (sim != null)
                {
                    d.Children.Add(Ui.Header("SIMULATOR — surface under the virtual stylus"));
                    var tc = new ComboBox { Width = 220 };
                    foreach (var t in SimulatedArm.Targets) tc.Items.Add(t);
                    tc.SelectedIndex = sim.Target;
                    tc.SelectionChanged += (s, e) => sim.Target = tc.SelectedIndex;
                    d.Children.Add(tc);
                    var pr = new WrapPanel { Margin = new Thickness(0, 4, 0, 0) };
                    var pd = Ui.Btn("Hold pedal 1", () => { });
                    pd.PreviewMouseDown += (s, e) => sim.SetButton(0, true);
                    pd.PreviewMouseUp += (s, e) => sim.SetButton(0, false);
                    pr.Children.Add(pd);
                    pr.Children.Add(Ui.Btn("Pedal 2", () => { sim.SetButton(1, true); sim.SetButton(1, false); }));
                    d.Children.Add(pr);
                    d.Children.Add(Ui.Text("Space captures, Enter finishes — same as the pedals.", 11, null, Ui.Muted));
                }
            }
            p.Children.Add(Ui.Card(d));

            var tp = new StackPanel();
            tp.Children.Add(Ui.Header("TIP / STYLUS"));
            var tip = _s.CurrentTip;
            var nameBox = new TextBox { Text = tip.Name, Width = 200, Margin = new Thickness(2) };
            nameBox.LostFocus += (s, e) => { if (nameBox.Text.Trim() != "" && nameBox.Text != tip.Name) { tip.Name = nameBox.Text.Trim(); _s.ActiveTip = tip.Name; RefreshTips(); } };
            tp.Children.Add(Ui.Row(Lbl("Name"), nameBox));
            tp.Children.Add(Ui.Row(Lbl("Ball radius"), Ui.LenBox(tip.BallRadius, v => { tip.BallRadius = Math.Max(0, v); ApplyTip(); }), Ui.UnitLabel()));
            tp.Children.Add(Ui.Row(Lbl("Offset X / Y / Z"),
                Ui.LenBox(tip.OffsetX, v => { tip.OffsetX = v; ApplyTip(); }, 55), Ui.LenBox(tip.OffsetY, v => { tip.OffsetY = v; ApplyTip(); }, 55), Ui.LenBox(tip.OffsetZ, v => { tip.OffsetZ = v; ApplyTip(); }, 55), Ui.UnitLabel()));
            tp.Children.Add(Ui.Text("Offset is in the stylus frame (Z along the stylus) relative to the factory tip. Leave 0 for the standard point tip; use pivot calibration for custom styli.", 11, null, Ui.Muted));
            var tb = new WrapPanel();
            tb.Children.Add(Ui.Btn("New tip", () => { var t = new TipProfile { Name = "Tip " + (_s.Tips.Count + 1) }; _s.Tips.Add(t); _s.ActiveTip = t.Name; RefreshTips(); ApplyTip(); }));
            tb.Children.Add(Ui.Btn("Delete tip", () => { if (_s.Tips.Count > 1) { _s.Tips.Remove(tip); _s.ActiveTip = _s.Tips[0].Name; RefreshTips(); ApplyTip(); } }));
            tp.Children.Add(tb);
            p.Children.Add(Ui.Card(tp));

            var pc = new StackPanel();
            pc.Children.Add(Ui.Header("PIVOT CALIBRATION (custom tips)"));
            pc.Children.Add(Ui.Text("Put the tip in a small conical divot, then capture 8–20 poses while swinging the arm around it (keep the tip seated). The tip offset and ball centre are solved by least squares.", 11.5, null, Ui.Muted));
            var pw = new WrapPanel();
            pw.Children.Add(Ui.Btn(_pivotActive ? "Stop" : "Start", () => { _pivotActive = !_pivotActive; if (_pivotActive) _pivotFrames.Clear(); RebuildArmPanel(); }));
            pw.Children.Add(Ui.Btn("Capture pose", CapturePivot));
            pw.Children.Add(Ui.Btn("Solve & apply", SolvePivot));
            pc.Children.Add(pw);
            _pivotText = Ui.Text(_pivotActive ? _pivotFrames.Count + " poses captured (pedal 1 / Space captures while active)" : "", 12);
            pc.Children.Add(_pivotText);
            p.Children.Add(Ui.Card(pc));
            p.Children.Add(Ui.Card(QualifyCard()));
            p.Children.Add(Ui.Card(ArmCheckCard()));

            var cs = new StackPanel();
            cs.Children.Add(Ui.Header("CAPTURE & PEDALS"));
            cs.Children.Add(Ui.Row(Lbl("Average after press (ms)"), Ui.NumBox(_s.CaptureAverageMs, v => _s.CaptureAverageMs = (int)Math.Max(0, Math.Min(1000, v)))));
            cs.Children.Add(Ui.Row(Lbl("Reject if moved over"), Ui.LenBox(_s.CaptureMotionLimit, v => _s.CaptureMotionLimit = Math.Max(0, v)), Ui.UnitLabel(),
                Ui.Text("  during the average (0 = off)", 11, null, Ui.Muted)));
            var dwell = new CheckBox { Content = "Hands-free: capture when the stylus is held still", IsChecked = _s.DwellMs > 0, Margin = new Thickness(0, 6, 0, 2) };
            dwell.Click += (s, e) => { _s.DwellMs = dwell.IsChecked == true ? 800 : 0; RebuildArmPanel(); };
            cs.Children.Add(dwell);
            if (_s.DwellMs > 0)
            {
                cs.Children.Add(Ui.Row(Lbl("Hold still for (ms)"), Ui.NumBox(_s.DwellMs, v => _s.DwellMs = (int)Math.Max(200, Math.Min(5000, v)))));
                cs.Children.Add(Ui.Row(Lbl("Still within"), Ui.LenBox(_s.DwellRadius, v => _s.DwellRadius = Math.Max(0.02, v)), Ui.UnitLabel()));
                cs.Children.Add(Ui.Row(Lbl("Next after moving"), Ui.LenBox(_s.DwellRearm, v => _s.DwellRearm = Math.Max(0.1, v)), Ui.UnitLabel()));
                cs.Children.Add(Ui.Text("Only while measuring a feature, calibrating or checking. Traced curves still use the pedal.", 11, null, Ui.Muted));
            }
            var sm = new ComboBox { Width = 160 };
            sm.Items.Add("By distance"); sm.Items.Add("By time"); sm.Items.Add("At section planes");
            sm.SelectedIndex = (int)_s.ScanMode;
            sm.SelectionChanged += (s, e) => { _s.ScanMode = (ScanMode)sm.SelectedIndex; RebuildArmPanel(); UpdatePrompt(); };
            cs.Children.Add(Ui.Row(Lbl("Scan (pedal held)"), sm));
            cs.Children.Add(Ui.Row(Lbl("Curve point spacing"), Ui.LenBox(_s.CurveSpacing, v => _s.CurveSpacing = Math.Max(0.05, v)), Ui.UnitLabel()));
            if (_s.ScanMode == ScanMode.Time)
                cs.Children.Add(Ui.Row(Lbl("Scan interval (ms)"), Ui.NumBox(_s.ScanIntervalMs, v => _s.ScanIntervalMs = (int)Math.Max(10, Math.Min(5000, v)))));
            if (_s.ScanMode == ScanMode.Sections)
            {
                var ax = new ComboBox { Width = 60 };
                foreach (var a in new[] { "X", "Y", "Z" }) ax.Items.Add(a);
                ax.SelectedIndex = _s.SectionAxis;
                ax.SelectionChanged += (s, e) => _s.SectionAxis = ax.SelectedIndex;
                cs.Children.Add(Ui.Row(Lbl("Section planes ⟂ part"), ax, Ui.Text("  every ", 12, null, Ui.Muted), Ui.LenBox(_s.SectionSpacing, v => _s.SectionSpacing = Math.Max(0.1, v)), Ui.UnitLabel()));
                cs.Children.Add(Ui.Text("Sweep back and forth along that axis (across the planes) while moving slowly sideways: a point is kept wherever the ball centre crosses a plane. Freeform surfaces loft through the sections; curves always use the distance.", 11, null, Ui.Muted));
            }
            cs.Children.Add(Ui.Row(Lbl("Default margin"), Ui.LenBox(_s.DefaultExportMargin, v => _s.DefaultExportMargin = Math.Max(0, v)), Ui.UnitLabel()));
            Func<PedalAction, Action<PedalAction>, ComboBox> pedal = (cur, set) =>
            {
                var c = new ComboBox { Width = 120 };
                foreach (var v in Enum.GetValues(typeof(PedalAction))) c.Items.Add(v);
                c.SelectedItem = cur;
                c.SelectionChanged += (s, e) => set((PedalAction)c.SelectedItem);
                return c;
            };
            cs.Children.Add(Ui.Row(Lbl("Pedal / button 1"), pedal(_s.Pedal1, v => _s.Pedal1 = v)));
            cs.Children.Add(Ui.Row(Lbl("Pedal / button 2"), pedal(_s.Pedal2, v => _s.Pedal2 = v)));
            cs.Children.Add(Ui.Header("CONNECTION"));
            var baud = new ComboBox { Width = 120, IsEditable = true, Text = _s.Baud == 0 ? "Auto" : _s.Baud.ToString() };
            foreach (var bd in new[] { "Auto", "9600", "19200", "38400", "57600", "115200" }) baud.Items.Add(bd);
            baud.LostFocus += (s, e) => { int v; _s.Baud = int.TryParse(baud.Text, out v) ? v : 0; };
            baud.SelectionChanged += (s, e) => { int v; _s.Baud = baud.SelectedItem != null && int.TryParse(baud.SelectedItem.ToString(), out v) ? v : 0; };
            cs.Children.Add(Ui.Row(Lbl("Baud rate"), baud));
            p.Children.Add(Ui.Card(cs));
        }

        void ArmTick(ArmSample s)
        {
            if (++_armTickCount % 6 != 0) return;
            if (_spaceLive != null && _space != null && _tabs.SelectedIndex == 4)
                _spaceLive.Text = string.Format(Ui.CI, "T {0,5:0} {1,5:0} {2,5:0}   R {3,5:0} {4,5:0} {5,5:0}   buttons 0x{6:X}",
                    _space.Axes[0], _space.Axes[1], _space.Axes[2], _space.Axes[3], _space.Axes[4], _space.Axes[5], _space.Buttons);
            if (_armLive == null || s == null || _tabs.SelectedIndex != 3) return;
            string enc = s.Encoders != null ? string.Join(" ", s.Encoders.Select(e => e.ToString().PadLeft(6))) : "";
            string ang = s.JointAngles != null ? string.Join(" ", s.JointAngles.Take(6).Select(a => (a * 180 / Math.PI).ToString("0.00", Ui.CI).PadLeft(7))) : "";
            _armLive.Text = string.Format(Ui.CI, "Encoders {0}\nJoints°  {1}\nTip (machine) {2}, {3}, {4} {5}\nStylus dir    {6:0.0000}, {7:0.0000}, {8:0.0000}\nButtons 0x{9:X2}   {10:0} Hz",
                enc, ang, Ui.L(s.Tip.X), Ui.L(s.Tip.Y), Ui.L(s.Tip.Z), Ui.U, s.Direction.X, s.Direction.Y, s.Direction.Z, s.Buttons, _arm != null ? _arm.SampleRate : 0);
        }

        void CapturePivot()
        {
            var s = _arm != null ? _arm.Latest : null;
            if (s == null || s.StylusFrame == null) { Status("No arm data"); return; }
            if (!_pivotActive) { _pivotActive = true; _pivotFrames.Clear(); }
            _pivotFrames.Add(s.StylusFrame.Clone());
            if (_pivotText != null) _pivotText.Text = _pivotFrames.Count + " poses captured";
            Sounds.Blip();
        }

        void SolvePivot()
        {
            Vec3 t, c; double rms;
            if (!Calibration.Pivot(_pivotFrames, out t, out c, out rms)) { Status("Need at least 4 well-spread poses"); return; }
            var tip = _s.CurrentTip;
            string msg = string.Format("Tip offset (stylus frame): {0}, {1}, {2} {3}\nPivot point: {4}, {5}, {6}\nRMS: {7} over {8} poses\n\nApply this offset to '{9}'?",
                Ui.L(t.X), Ui.L(t.Y), Ui.L(t.Z), Ui.U, Ui.L(c.X), Ui.L(c.Y), Ui.L(c.Z), Ui.LU(rms), _pivotFrames.Count, tip.Name);
            if (MessageBox.Show(this, msg, "Pivot calibration", MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
            {
                tip.Offset = t;
                ApplyTip();
                _s.Save();
                Sounds.Success();
            }
            _pivotActive = false;
            RebuildArmPanel();
        }

        // ================================================================== files & commands

        void NewProject()
        {
            if (_dirty && _proj.Features.Count > 0 && MessageBox.Show(this, "Discard the current project?", "New", MessageBoxButton.YesNo) != MessageBoxResult.Yes) return;
            _proj = new Project(); _projPath = null; _dirty = false; _active = null; _selected = null; _selectedDim = 0;
            SetTool(Tool.Select);
            UpdatePrompt();
            RebuildAll();
        }

        void OpenProject(string path)
        {
            if (path == null)
            {
                var dlg = new Microsoft.Win32.OpenFileDialog { Filter = "MicroScribe projects (*.msproj)|*.msproj|All files|*.*" };
                if (dlg.ShowDialog(this) != true) return;
                path = dlg.FileName;
            }
            try
            {
                _proj = ProjectIO.Load(path);
                _projPath = path; _dirty = false; _active = null; _selected = null; _selectedDim = 0;
                _s.AddRecent(path);
                LoadProjectModel();
                _dirty = false;
                SetTool(Tool.Select);
                UpdatePrompt();
                RebuildAll();
                FitView();
                Status("Opened " + path);
            }
            catch (Exception ex) { MessageBox.Show(this, "Could not open " + path + ":\n" + ex.Message, "Open", MessageBoxButton.OK, MessageBoxImage.Error); }
        }

        bool SaveProject(bool saveAs)
        {
            if (_active != null) FinishMeasure(true);
            string path = _projPath;
            if (saveAs || path == null)
            {
                var dlg = new Microsoft.Win32.SaveFileDialog { Filter = "MicroScribe projects (*.msproj)|*.msproj", FileName = (_proj.Title ?? "Untitled") + ".msproj", DefaultExt = ".msproj" };
                if (dlg.ShowDialog(this) != true) return false;
                path = dlg.FileName;
                _proj.Title = System.IO.Path.GetFileNameWithoutExtension(path);
            }
            try
            {
                _proj.Units = Ui.U;
                ProjectIO.Save(_proj, path);
                _projPath = path; _dirty = false;
                _s.AddRecent(path);
                UpdateTitle();
                Status("Saved " + path);
                return true;
            }
            catch (Exception ex) { MessageBox.Show(this, "Save failed: " + ex.Message, "Save", MessageBoxButton.OK, MessageBoxImage.Error); return false; }
        }

        string ExportPath(string ext, string filter)
        {
            var dlg = new Microsoft.Win32.SaveFileDialog
            {
                Filter = filter, DefaultExt = ext,
                FileName = (_projPath != null ? System.IO.Path.GetFileNameWithoutExtension(_projPath) : (_proj.Title ?? "measurement")) + ext
            };
            if (Directory.Exists(_s.LastExportFolder)) dlg.InitialDirectory = _s.LastExportFolder;
            if (dlg.ShowDialog(this) != true) return null;
            _s.LastExportFolder = System.IO.Path.GetDirectoryName(dlg.FileName);
            return dlg.FileName;
        }

        void ExportStep()
        {
            if (_active != null) FinishMeasure(true);
            if (!_proj.Features.Any(f => f.Export && f.Geometry != null)) { Status("Nothing to export yet"); return; }
            string path = ExportPath(".step", "STEP AP214 (*.step;*.stp)|*.step;*.stp");
            if (path == null) return;
            try
            {
                string summary = CadExport.WriteStep(_proj, path, new ExportOptions { Author = _s.Author, UsePartAlignment = true });
                Status("Exported " + path + " — " + summary);
                MessageBox.Show(this, "Wrote " + path + "\n" + summary + "\nCoordinate system: " + _proj.AlignmentDescription +
                    "\n\nThe STEP file is written in millimetres; SolidWorks converts it to your document units (inch parts are fine)." +
                    "\n\nSolidWorks: File > Open (STEP). In Options, enable 'Import free point/curve entities' to get axes, centres and curves; surfaces arrive as surface bodies (Knit/Thicken them as needed).",
                    "Export STEP", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex) { MessageBox.Show(this, "Export failed: " + ex.Message, "Export", MessageBoxButton.OK, MessageBoxImage.Error); }
        }

        void ExportIges()
        {
            if (_active != null) FinishMeasure(true);
            if (!_proj.Features.Any(f => f.Export && f.Geometry != null)) { Status("Nothing to export yet"); return; }
            string path = ExportPath(".igs", "IGES (*.igs;*.iges)|*.igs;*.iges");
            if (path == null) return;
            try
            {
                string summary = CadExport.WriteIges(_proj, path, new ExportOptions { Author = _s.Author, UsePartAlignment = true },
                    Installer.AppName + " " + typeof(MainWindow).Assembly.GetName().Version.ToString(3));
                Status("Exported " + path + " — " + summary + " (millimetres; surfaces as untrimmed B-spline surfaces)");
            }
            catch (Exception ex) { MessageBox.Show(this, "Export failed: " + ex.Message, "Export", MessageBoxButton.OK, MessageBoxImage.Error); }
        }

        void ExportStl()
        {
            if (_active != null) FinishMeasure(true);
            string path = ExportPath(".stl", "STL mesh (*.stl)|*.stl");
            if (path == null) return;
            try
            {
                int n = CadExport.WriteStl(_proj, path, new ExportOptions { UsePartAlignment = true });
                Status(n == 0 ? "No surfaces to export" : string.Format("Exported {0}: {1:N0} triangles (millimetres, part frame)", System.IO.Path.GetFileName(path), n));
            }
            catch (Exception ex) { MessageBox.Show(this, "Export failed: " + ex.Message, "Export", MessageBoxButton.OK, MessageBoxImage.Error); }
        }

        void ExportPoints(string kind)
        {
            string path = kind == "csv" ? ExportPath(".csv", "CSV (*.csv)|*.csv") : ExportPath(".xyz", "XYZ point cloud (*.xyz;*.txt)|*.xyz;*.txt");
            if (path == null) return;
            if (kind == "csv") CadExport.WriteCsv(_proj, path, true, Fmt);
            else CadExport.WriteXyz(_proj, path, true, Fmt, _selected != null && _selected.Kind == FeatureKind.Curve ? new[] { _selected } : null);
            Status("Exported " + path + " (" + (Ui.Inch ? "inches" : "millimetres") + ")");
        }

        void DeleteSelected()
        {
            var f = _selected;
            if (f == null) return;
            var users = _proj.Features.Where(x => x.SourceIds.Contains(f.Id)).ToList();
            int dims = _proj.Dimensions.Count(d => d.Refs.Any(r => r.FeatureId == f.Id));
            string extra = (users.Count > 0 ? "\n\nUsed by: " + string.Join(", ", users.Select(u => u.Name)) : "") + (dims > 0 ? "\n" + dims + " measurement(s) will be removed." : "");
            if (MessageBox.Show(this, "Delete " + f.Name + "?" + extra, "Delete", MessageBoxButton.YesNo, MessageBoxImage.Question) != MessageBoxResult.Yes) return;
            if (f == _active) { _active = null; UpdatePrompt(); }
            _proj.Features.Remove(f);
            _proj.Dimensions.RemoveAll(d => d.Refs.Any(r => r.FeatureId == f.Id));
            _selected = null;
            Changed();
        }

        void RenameSelected()
        {
            _tabs.SelectedIndex = 0;
            RebuildFeaturePanel();
        }

        void ShowSettings() { _tabs.SelectedIndex = 3; }

        void DoInstall()
        {
            var r = MessageBox.Show(this, "Install " + Installer.AppName + " for the current user?\n\n" +
                "• Program copied to " + Installer.InstallDir + "\n• Start-menu shortcut (and a desktop shortcut if you choose Yes below)\n• .msproj files open with this program\n• Listed in Apps & features for uninstall\n\nNo administrator rights needed. The portable copy keeps working.\n\nAlso create a desktop shortcut?",
                "Install", MessageBoxButton.YesNoCancel, MessageBoxImage.Question);
            if (r == MessageBoxResult.Cancel) return;
            try
            {
                string log = Installer.Install(r == MessageBoxResult.Yes, true);
                MessageBox.Show(this, log, "Installed", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex) { MessageBox.Show(this, "Install failed: " + ex.Message, "Install", MessageBoxButton.OK, MessageBoxImage.Error); }
        }

        void DoUninstall()
        {
            if (!Installer.IsInstalled()) { MessageBox.Show(this, "Not installed (running as a portable copy).", "Uninstall"); return; }
            if (MessageBox.Show(this, "Remove the installed copy, shortcuts and file association?", "Uninstall", MessageBoxButton.YesNo, MessageBoxImage.Question) != MessageBoxResult.Yes) return;
            string log = Installer.Uninstall();
            MessageBox.Show(this, log, "Uninstalled", MessageBoxButton.OK, MessageBoxImage.Information);
            if (Installer.IsInstalledCopy()) Close();
        }
    }
}
