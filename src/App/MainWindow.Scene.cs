using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using MicroScribe.Core;
using MicroScribe.Driver;
using MicroScribe.Export;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    public sealed partial class MainWindow
    {
        bool _showMachineFrame;
        bool _showBox;
        double _marker = 1.0;   // point marker radius in mm (scaled to the scene)

        Mat4 DisplayMatrix { get { return _showMachineFrame ? Mat4.Identity() : _proj.AlignmentMatrix; } }

        List<Vec3> DisplayPoints()
        {
            Mat4 m = DisplayMatrix;
            var pts = new List<Vec3>();
            foreach (var f in _proj.Features.Where(x => x.Visible))
            {
                foreach (var p in f.Points) pts.Add(m.TransformPoint(p.Position));
                if (f.Geometry != null && !(f.Geometry is CurvePrim) && !(f.Geometry is SurfacePrim))
                    pts.Add(m.TransformPoint(Constructions.KeyPoint(f.Geometry)));
            }
            var mesh = _proj.NominalModel;
            if (mesh != null && _s.ShowModel)
            {
                Mat4 x = ModelToDisplay;
                foreach (var cx in new[] { mesh.Min.X, mesh.Max.X }) foreach (var cy in new[] { mesh.Min.Y, mesh.Max.Y }) foreach (var cz in new[] { mesh.Min.Z, mesh.Max.Z })
                    pts.Add(x.TransformPoint(new Vec3(cx, cy, cz)));
            }
            return pts;
        }

        bool WorkAreaActive { get { return _s.WorkArea.Enabled && _s.WorkArea.IsDefined; } }

        /// <summary>Fit: the work area volume when defined, otherwise the measured features.</summary>
        void FitView()
        {
            if (WorkAreaActive)
            {
                _view.Fit(_s.WorkArea.Corners().Select(DisplayMatrix.TransformPoint).ToList());
                return;
            }
            FitFeatures();
        }

        void FitWorkOrFeatures() { FitView(); }

        void FitFeatures()
        {
            var pts = DisplayPoints();
            var s = _arm != null ? _arm.Latest : null;
            if (s != null) pts.Add(DisplayMatrix.TransformPoint(s.Tip));
            if (pts.Count == 0) { pts.Add(new Vec3(-150, -150, -50)); pts.Add(new Vec3(350, 150, 150)); }
            if (pts.Count == 1) { pts.Add(pts[0] + new Vec3(40, 40, 40)); pts.Add(pts[0] - new Vec3(40, 40, 40)); }
            _view.Fit(pts);
        }

        void RebuildScene()
        {
            if (_view == null) return;
            Mat4 M = DisplayMatrix;
            var pts = DisplayPoints();
            if (WorkAreaActive) pts.AddRange(_s.WorkArea.Corners().Select(M.TransformPoint));
            Vec3 bmin, bmax;
            if (pts.Count > 0) AlignmentSolver.Bounds(pts, out bmin, out bmax);
            else { bmin = new Vec3(-100, -100, 0); bmax = new Vec3(100, 100, 0); }
            double diag = Math.Max(60, (bmax - bmin).Length);
            _marker = Math.Max(0.25, Math.Min(4, diag / 260));
            _view.SceneScale = diag;

            BuildGrid(M, bmin, bmax, diag);
            BuildWorkArea(M);

            var group = new Model3DGroup();
            var alpha = new Model3DGroup();
            var tags = new Dictionary<Model3D, PickTag>();
            var alphaTags = new Dictionary<Model3D, PickTag>();
            var labels = new List<Label3D>();
            foreach (var f in _proj.Features.Where(x => x.Visible))
                AddFeatureModels(f, M, group, alpha, tags, alphaTags, labels);
            group.Freeze(); alpha.Freeze();
            _view.SetLayer("features", group, tags, labels);
            _view.SetLayer("surfaces", alpha, alphaTags);
            RebuildPlaneHandles();
            RebuildDims();
            RebuildOverlay();
            RebuildToolLayer();
            RebuildModelLayer();
        }

        void BuildGrid(Mat4 M, Vec3 bmin, Vec3 bmax, double diag)
        {
            var g = new Model3DGroup();
            double step = Ui.Inch
                ? (diag > 1500 ? 101.6 : diag > 400 ? 25.4 : diag > 150 ? 12.7 : 6.35)
                : (diag > 1500 ? 100 : diag > 400 ? 50 : diag > 150 ? 10 : 5);
            double ext = Math.Ceiling(Math.Max(Math.Max(Math.Abs(bmin.X), Math.Abs(bmax.X)), Math.Max(Math.Abs(bmin.Y), Math.Abs(bmax.Y))) / step + 1) * step;
            ext = Math.Max(ext, step * 4);
            var minor = new MeshBuilder(); var major = new MeshBuilder();
            double rr = _marker * 0.08;
            int n = (int)(ext / step);
            for (int i = -n; i <= n; i++)
            {
                var mb = i % (Ui.Inch ? 4 : 5) == 0 ? major : minor;
                mb.Tube(new Vec3(i * step, -ext, 0), new Vec3(i * step, ext, 0), i == 0 ? rr * 1.6 : rr, 4);
                mb.Tube(new Vec3(-ext, i * step, 0), new Vec3(ext, i * step, 0), i == 0 ? rr * 1.6 : rr, 4);
            }
            g.Children.Add(SceneView.Model(minor, Materials.Emissive(Color.FromRgb(70, 78, 92))));
            g.Children.Add(SceneView.Model(major, Materials.Emissive(Color.FromRgb(105, 115, 132))));
            double L = Math.Max(20, Math.Min(150, diag * 0.12));
            Action<Vec3, Color> arrow = (d, c) => { var mb = new MeshBuilder(); mb.Arrow(Vec3.Zero, d * L, _marker * 0.35); g.Children.Add(SceneView.Model(mb, Materials.Solid(c))); };
            arrow(Vec3.UnitX, Color.FromRgb(239, 68, 68));
            arrow(Vec3.UnitY, Color.FromRgb(34, 197, 94));
            arrow(Vec3.UnitZ, Color.FromRgb(59, 130, 246));
            Vec3 b0 = M.TransformPoint(Vec3.Zero), bz = M.TransformVector(Vec3.UnitZ);
            var bm = new MeshBuilder();
            bm.Revolved(b0, bz, bz.AnyPerpendicular(), h => 45, 0, 0, 2 * Math.PI, 0, 40, 40, 1);
            g.Freeze();
            _view.SetLayer("grid", g);
            var ga = new Model3DGroup();
            ga.Children.Add(SceneView.Model(bm, Materials.Translucent(Color.FromRgb(150, 155, 165), 0.35), true));
            ga.Freeze();
            _view.SetLayer("gridalpha", ga);
        }

        /// <summary>Work area: table outline, top outline and verticals.</summary>
        void BuildWorkArea(Mat4 M)
        {
            var wa = _s.WorkArea;
            if (!wa.IsDefined || (!wa.Enabled && _tabs.SelectedIndex != 4)) { _view.SetLayer("work", null); _view.SetLayer("workfill", null); return; }
            var g = new Model3DGroup();
            Vec3 n = M.TransformVector(wa.TableNormal);
            var bottom = wa.OutlinePoints.Select(M.TransformPoint).ToList();
            var top = bottom.Select(p => p + n * wa.Height).ToList();
            var fill = new MeshBuilder();
            fill.Polygon(bottom, n);
            var fg = new Model3DGroup();
            fg.Children.Add(SceneView.Model(fill, Materials.Translucent(Color.FromRgb(56, 189, 248), wa.Enabled ? 0.10 : 0.05), true));
            fg.Freeze();
            _view.SetLayer("workfill", fg);
            var lines = new MeshBuilder();
            lines.Polyline(bottom, _marker * 0.22, true);
            lines.Polyline(top, _marker * 0.12, true);
            for (int i = 0; i < bottom.Count; i++) lines.Tube(bottom[i], top[i], _marker * 0.1, 4);
            g.Children.Add(SceneView.Model(lines, Materials.Emissive(wa.Enabled ? Color.FromRgb(56, 189, 248) : Color.FromRgb(90, 110, 130))));
            g.Freeze();
            _view.SetLayer("work", g);
        }

        void AddFeatureModels(Feature f, Mat4 M, Model3DGroup group, Model3DGroup alpha, Dictionary<Model3D, PickTag> tags,
                              Dictionary<Model3D, PickTag> alphaTags, List<Label3D> labels)
        {
            bool sel = f == _selected || f == _active;
            Color c = Ui.KindColor(f.Kind);
            var tag = new PickTag { FeatureId = f.Id };
            var ptTag = new PickTag { FeatureId = f.Id, PointIndex = -2 };

            // probed points
            var inc = new MeshBuilder(); var exc = new MeshBuilder();
            bool traced = Fitter.IsTraced(f.Kind);
            double pr = _marker * (sel ? 1.15 : 0.85) * (traced ? 0.6 : 1);
            var pts = f.Points.Select(p => M.TransformPoint(p.Position)).ToList();
            bool byDev = _s.ColorByDeviation && _proj.NominalModel != null && f.Origin == FeatureOrigin.Measured;
            var bins = byDev ? Enumerable.Range(0, 5).Select(i => new MeshBuilder()).ToArray() : null;
            for (int i = 0; i < f.Points.Count; i++)
            {
                if (f.Points[i].Excluded) exc.Sphere(pts[i], pr * 0.8, 8, 6);
                else if (byDev) bins[DevBin(PointDeviation(f, f.Points[i]))].Sphere(pts[i], pr, 8, 6);
                else inc.Sphere(pts[i], pr, 8, 6);
            }
            if (byDev)
                for (int b = 0; b < 5; b++)
                    if (bins[b].VertexCount > 0) { var m = SceneView.Model(bins[b], Materials.Solid(DevColors[b])); group.Children.Add(m); tags[m] = ptTag; }
            if (inc.VertexCount > 0) { var m = SceneView.Model(inc, Materials.Solid(sel ? Lighter(c, 0.35) : c)); group.Children.Add(m); tags[m] = ptTag; }
            if (exc.VertexCount > 0) { var m = SceneView.Model(exc, Materials.Solid(Color.FromRgb(120, 120, 120))); group.Children.Add(m); tags[m] = ptTag; }

            var surf = new MeshBuilder(); var wire = new MeshBuilder();
            double tube = _marker * (sel ? 0.45 : 0.3);
            double margin = f.ExportMargin;

            // raw traces: always for freeform sections, for curves while not solved yet (e.g. while scanning)
            if ((f.Geometry == null || f.Kind == FeatureKind.Surface) && traced && f.Kind != FeatureKind.Cloud && !f.SectionScan && pts.Count > 1)
            {
                foreach (var grp in f.Points.Select((p, i) => new { p, i }).GroupBy(x => x.p.Section))
                    wire.Polyline(grp.Select(x => pts[x.i]).ToList(), _marker * 0.25);
            }

            var g = GeoUtil.Transform(f.Geometry, M);
            if (g == null)
            {
                if (wire.VertexCount > 0) { var m = SceneView.Model(wire, Materials.Solid(c)); group.Children.Add(m); tags[m] = tag; }
                return;
            }
            var activePts = f.ActivePoints.Select(p => M.TransformPoint(p.Position)).ToList();
            Vec3 labelAt = Constructions.KeyPoint(g);
            if (g is PlanePrim)
            {
                var poly = PlaneBounds.Corners(f, (PlanePrim)f.Geometry).Select(M.TransformPoint).ToList();
                surf.Polygon(poly, ((PlanePrim)g).Normal);
                wire.Polyline(poly, tube * (f.CustomBounds ? 1.2 : 1), true);
                Vec3 cen = (poly[0] + poly[2]) * 0.5;
                wire.Arrow(cen, cen + ((PlanePrim)g).Normal * Math.Max(8, _marker * 12), tube);
                labelAt = cen;
            }
            else if (g is CylinderPrim)
            {
                var cy = (CylinderPrim)g;
                Vec3 xr; double t0, t1, h0, h1;
                CadExport.RevolvedExtent(cy.AxisPoint, cy.Axis, activePts, margin, f.FullRevolution, out xr, out t0, out t1, out h0, out h1, cy.Radius);
                surf.Revolved(cy.AxisPoint, cy.Axis, xr, h => cy.Radius, 0, t0, t1, h0, h1);
                wire.Tube(cy.AxisPoint + cy.Axis * (h0 - margin * 2), cy.AxisPoint + cy.Axis * (h1 + margin * 2), tube * 0.8, 6);
            }
            else if (g is ConePrim)
            {
                var co = (ConePrim)g;
                Vec3 xr; double t0, t1, h0, h1;
                CadExport.RevolvedExtent(co.AxisPoint, co.Axis, activePts, margin, f.FullRevolution, out xr, out t0, out t1, out h0, out h1, co.RadiusAtOrigin);
                double hApex = co.HalfAngle > 1e-4 ? -co.RadiusAtOrigin / Math.Tan(co.HalfAngle) : double.MinValue;
                h0 = Math.Max(h0, hApex);
                surf.Revolved(co.AxisPoint, co.Axis, xr, h => Math.Max(0, co.RadiusAt(h)), Math.Tan(co.HalfAngle), t0, t1, h0, h1);
                wire.Tube(co.AxisPoint + co.Axis * Math.Max(hApex, h0 - margin * 2), co.AxisPoint + co.Axis * (h1 + margin * 2), tube * 0.8, 6);
            }
            else if (g is SpherePrim)
            {
                var sp = (SpherePrim)g;
                Vec3 pole; double cap;
                CadExport.SphereCap(sp, activePts, margin, out pole, out cap);
                surf.SphereCap(sp.Center, sp.Radius, pole, cap);
                wire.Sphere(sp.Center, tube * 2, 8, 6);
            }
            else if (g is ProfilePrim)
            {
                var pp = (ProfilePrim)g;
                wire.Polyline(pp.Outline().Select(M.TransformPoint).ToList(), tube);
                wire.Sphere(M.TransformPoint(pp.Center), tube * 2, 8, 6);
            }
            else if (g is CirclePrim)
            {
                var ci = (CirclePrim)g;
                wire.Ring(ci.Center, ci.Normal, ci.Radius, tube);
                wire.Sphere(ci.Center, tube * 2, 8, 6);
                wire.Arrow(ci.Center, ci.Center + ci.Normal * Math.Max(6, ci.Radius * 0.6), tube * 0.7);
            }
            else if (g is LinePrim)
            {
                var l = (LinePrim)g;
                wire.Arrow(l.Origin + l.Direction * (l.TMin - margin), l.Origin + l.Direction * (l.TMax + margin), tube);
            }
            else if (g is PointPrim)
            {
                wire.Sphere(((PointPrim)g).Position, _marker * 1.8, 12, 8);
            }
            else if (g is CurvePrim)
            {
                var cu = (CurvePrim)g;
                wire.Polyline(cu.Samples, tube * 0.9);
                labelAt = cu.Samples[cu.Samples.Length / 2];
            }
            else if (g is SurfacePrim)
            {
                var su = (SurfacePrim)g;
                AddSurfaceMesh(su, surf, wire, tube * 0.5);
                labelAt = su.Evaluate(0.5, 0.5);
            }
            if (surf.VertexCount > 0)
            {
                var m = SceneView.Model(surf, Materials.Translucent(c, sel ? 0.55 : 0.32), true);
                alpha.Children.Add(m); alphaTags[m] = tag;
            }
            if (wire.VertexCount > 0)
            {
                var m = SceneView.Model(wire, Materials.Solid(sel ? Lighter(c, 0.3) : c));
                group.Children.Add(m); tags[m] = tag;
            }
            labels.Add(new Label3D { Position = labelAt, Text = f.Name, Brush = new SolidColorBrush(Lighter(c, 0.45)) });
        }

        static void AddSurfaceMesh(SurfacePrim s, MeshBuilder surf, MeshBuilder wire, double tube)
        {
            var grid = s.Grid; var nrm = s.GridNormals;
            int n = SurfacePrim.GridN;
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                {
                    surf.Polygon(new[] { grid[i, j], grid[i + 1, j], grid[i + 1, j + 1] }, (nrm[i, j] + nrm[i + 1, j + 1]).Normalized());
                    surf.Polygon(new[] { grid[i, j], grid[i + 1, j + 1], grid[i, j + 1] }, (nrm[i, j] + nrm[i + 1, j + 1]).Normalized());
                }
            // iso-lines and boundary
            for (int k = 0; k <= n; k += n / 8)
            {
                wire.Polyline(Enumerable.Range(0, n + 1).Select(i => grid[i, k]).ToList(), k == 0 || k == n ? tube * 1.6 : tube);
                wire.Polyline(Enumerable.Range(0, n + 1).Select(j => grid[k, j]).ToList(), k == 0 || k == n ? tube * 1.6 : tube);
            }
        }

        static Color Lighter(Color c, double k)
        {
            return Color.FromRgb((byte)(c.R + (255 - c.R) * k), (byte)(c.G + (255 - c.G) * k), (byte)(c.B + (255 - c.B) * k));
        }

        /// <summary>Corner/edge handles for resizing the selected plane.</summary>
        void RebuildPlaneHandles()
        {
            var f = _selected;
            var pl = f != null && f.Visible ? f.Geometry as PlanePrim : null;
            if (pl == null || _tool != Tool.Select) { _view.SetLayer("handles", null); return; }
            Mat4 M = DisplayMatrix;
            var k = PlaneBounds.Corners(f, pl).Select(M.TransformPoint).ToList();
            var pos = new List<Vec3> { k[0], k[1], k[2], k[3], (k[0] + k[1]) * 0.5, (k[1] + k[2]) * 0.5, (k[2] + k[3]) * 0.5, (k[3] + k[0]) * 0.5 };
            var g = new Model3DGroup();
            var tags = new Dictionary<Model3D, PickTag>();
            for (int i = 0; i < 8; i++)
            {
                var mb = new MeshBuilder();
                double r = _marker * (i < 4 ? 1.7 : 1.4);
                if (i < 4) mb.Sphere(pos[i], r, 10, 7);
                else
                {
                    Vec3 d = (i == 4 || i == 6 ? k[1] - k[0] : k[2] - k[1]).Normalized();
                    mb.Tube(pos[i] - d * r * 2, pos[i] + d * r * 2, r * 0.8, 10);
                }
                var m = SceneView.Model(mb, Materials.Emissive(i < 4 ? Color.FromRgb(250, 204, 21) : Color.FromRgb(253, 230, 138)));
                g.Children.Add(m);
                tags[m] = new PickTag { FeatureId = f.Id, Handle = "plane", HandleIndex = i };
            }
            g.Freeze();
            _view.SetLayer("handles", g, tags);
        }

        /// <summary>Dimension annotations (click-measure results).</summary>
        void RebuildDims()
        {
            if (_view == null) return;
            Mat4 M = DisplayMatrix;
            var g = new Model3DGroup();
            var tags = new Dictionary<Model3D, PickTag>();
            var labels = new List<Label3D>();
            foreach (var d in _proj.Dimensions)
            {
                var r = DimensionSolver.Evaluate(_proj, d);
                if (!r.Ok) continue;
                bool sel = d.Id == _selectedDim;
                var st = Tolerance.Evaluate(d, r, _proj.AlignmentMatrix).Status;
                Color col = st == TolStatus.Pass ? (sel ? Color.FromRgb(74, 222, 128) : Color.FromRgb(34, 197, 94))
                          : st == TolStatus.Fail ? (sel ? Color.FromRgb(248, 113, 113) : Color.FromRgb(239, 68, 68))
                          : sel ? Color.FromRgb(253, 224, 71) : Color.FromRgb(250, 204, 21);
                var mb = new MeshBuilder();
                Vec3 a = M.TransformPoint(r.A), b = M.TransformPoint(r.B);
                double t = _marker * (sel ? 0.35 : 0.22);
                if (r.HasLine && (b - a).Length > 1e-6)
                {
                    mb.Tube(a, b, t, 6);
                    mb.Sphere(a, t * 3, 8, 6); mb.Sphere(b, t * 3, 8, 6);
                }
                else if (r.Arc != null) mb.Polyline(r.Arc.Select(M.TransformPoint).ToList(), t);
                else mb.Sphere(a, t * 4, 8, 6);
                if (mb.VertexCount > 0)
                {
                    var m = SceneView.Model(mb, Materials.Emissive(col));
                    g.Children.Add(m);
                    tags[m] = new PickTag { DimId = d.Id };
                }
                string text;
                if (d.Kind == DimKind.Coordinate)
                {
                    Vec3 p = M.TransformPoint(r.A);
                    text = "X " + Ui.L(p.X) + "  Y " + Ui.L(p.Y) + "  Z " + Ui.L(p.Z);
                }
                else if (Gdt.IsGdt(d.Kind)) { string n; text = Gdt.Symbol(d.Kind, DimensionSolver.Resolve(_proj, d.Refs[0], out n)) + " " + Ui.LU(r.Value); }
                else if (r.IsAngle) text = r.Value.ToString("0.00", Ui.CI) + "°";
                else text = (d.Kind == DimKind.Diameter ? "Ø " : "") + Ui.LU(r.Value);
                if (st != TolStatus.None) text += st == TolStatus.Pass ? "  ✓" : "  ✗";
                labels.Add(new Label3D
                {
                    Position = M.TransformPoint(r.LabelAt), Text = text, Bold = true,
                    Brush = Brushes.Black, Background = new SolidColorBrush(Color.FromArgb(sel ? (byte)245 : (byte)215, col.R, col.G, col.B))
                });
            }
            g.Freeze();
            _view.SetLayer("dims", g, tags, labels);
        }

        /// <summary>Bounding box + clickable anchor handles (centre/corner origin tool).</summary>
        void RebuildOverlay()
        {
            if (_view == null) return;
            bool show = _showBox || (_tabs != null && _tabs.SelectedIndex == 2);
            if (!show) { _view.SetLayer("overlay", null); return; }
            Vec3 a, b; string err;
            var r = _proj.AlignmentRecipe;
            Mat4 full = AlignmentSolver.Solve(_proj, r, out a, out b, out err);
            if (AlignmentSolver.BoxPoints(_proj, r).Count == 0) { _view.SetLayer("overlay", null); return; }
            Mat4 toDisplay = DisplayMatrix * full.RigidInverse();
            var g = new Model3DGroup();
            var tags = new Dictionary<Model3D, PickTag>();
            var edges = new MeshBuilder();
            edges.BoxEdges(a, b, toDisplay, _marker * 0.18);
            g.Children.Add(SceneView.Model(edges, Materials.Emissive(Color.FromRgb(250, 204, 21))));
            int ax = (int)r.AnchorX, ay = (int)r.AnchorY, az = (int)r.AnchorZ;
            bool boxOrigin = r.Origin == OriginMode.BoundingBox;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                    {
                        int mids = (i == 1 ? 1 : 0) + (j == 1 ? 1 : 0) + (k == 1 ? 1 : 0);
                        if (mids == 1) continue; // edge midpoints: available in the panel, hidden here to keep the view tidy
                        Vec3 p = new Vec3(Pick3(i, a.X, b.X), Pick3(j, a.Y, b.Y), Pick3(k, a.Z, b.Z));
                        bool current = boxOrigin && i == ax && j == ay && k == az;
                        double rad = _marker * (mids == 0 ? 1.9 : mids == 3 ? 2.3 : 1.5) * (current ? 1.5 : 1);
                        var mb = new MeshBuilder();
                        mb.Sphere(toDisplay.TransformPoint(p), rad, 12, 8);
                        Color col = current ? Color.FromRgb(250, 204, 21) : mids == 3 ? Color.FromRgb(255, 255, 255) : mids == 0 ? Color.FromRgb(253, 224, 71) : Color.FromRgb(202, 138, 4);
                        var m = SceneView.Model(mb, current ? Materials.Emissive(col) : Materials.Solid(col));
                        g.Children.Add(m);
                        tags[m] = new PickTag { Anchor = new[] { i, j, k } };
                    }
            g.Freeze();
            _view.SetLayer("overlay", g, tags);
        }

        static double Pick3(int i, double a, double b) { return i == 0 ? a : i == 2 ? b : 0.5 * (a + b); }

        void UpdateLive(ArmSample s)
        {
            Mat4 M = DisplayMatrix;
            var g = new Model3DGroup();
            Vec3 tip = M.TransformPoint(s.Tip), dir = M.TransformVector(s.Direction);
            bool pressed = (s.Buttons & 3) != 0;
            Color col = pressed ? Color.FromRgb(239, 68, 68) : _active != null ? Color.FromRgb(251, 146, 60) : Color.FromRgb(74, 222, 128);
            var mb = new MeshBuilder();
            double r = Math.Max(_marker * 1.4, _s.CurrentTip.BallRadius);
            mb.Sphere(tip, r, 16, 10);
            g.Children.Add(SceneView.Model(mb, Materials.Emissive(col)));
            var st = new MeshBuilder();
            st.Tube(tip - dir * r, tip - dir * 70, _marker * 0.7, 10);
            g.Children.Add(SceneView.Model(st, Materials.Solid(Color.FromRgb(200, 205, 215))));
            if (_s.ShowArm && s.JointOrigins != null)
            {
                var arm = new MeshBuilder();
                var jp = s.JointOrigins.Select(p => M.TransformPoint(p)).ToList();
                for (int i = 0; i + 1 < jp.Count; i++)
                {
                    if ((jp[i + 1] - jp[i]).Length < 1e-6) continue;
                    arm.Tube(jp[i], jp[i + 1], Math.Max(2, _marker * 2.2), 10);
                }
                foreach (var p in jp) arm.Sphere(p, Math.Max(3, _marker * 3.2), 10, 7);
                g.Children.Add(SceneView.Model(arm, Materials.Translucent(Color.FromRgb(170, 176, 190), 0.28)));
            }
            g.Freeze();
            _view.SetLayer("live", g);
        }
    }
}
