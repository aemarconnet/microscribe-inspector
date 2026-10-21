using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using MicroScribe.Core;
using MicroScribe.Export;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    /// <summary>Demo project generated on the simulator's virtual part.</summary>
    public static class Demo
    {
        public static Project Build()
        {
            var rnd = new Random(42);
            var p = new Project { Title = "Demo block" };
            Func<int, string, FeatureKind, int, Func<int, double[]>, Feature> meas = (target, name, kind, n, uv) =>
            {
                var f = p.NewFeature(kind, name);
                for (int i = 0; i < n; i++)
                {
                    var t = uv(i);
                    Vec3 pt, nn;
                    SimulatedArm.SamplePoint(target, t[0], t[1], out pt, out nn);
                    Vec3 noise = new Vec3(rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5) * 0.06;
                    f.Points.Add(new MeasuredPoint(pt + noise, -nn, 0));
                }
                return f;
            };
            meas(1, "Top", FeatureKind.Plane, 9, i => new[] { (i % 3) / 2.0, (i / 3) / 2.0 });
            meas(2, "Front", FeatureKind.Plane, 6, i => new[] { (i % 3) / 2.0, (i / 3) * 0.8 + 0.1 });
            meas(3, "Left", FeatureKind.Plane, 6, i => new[] { (i % 3) / 2.0, (i / 3) * 0.8 + 0.1 });
            meas(4, "Bore", FeatureKind.Cylinder, 16, i => new[] { (i % 8) / 8.0, i < 8 ? 0.15 : 0.85 });
            meas(5, "Boss", FeatureKind.Cylinder, 12, i => new[] { (i % 6) / 6.0 + 0.05, i < 6 ? 0.2 : 0.9 });
            meas(6, "Ball", FeatureKind.Sphere, 12, i => new[] { (i % 6) / 6.0, i < 6 ? 0.35 : 0.85 });
            meas(7, "Countersink", FeatureKind.Cone, 12, i => new[] { (i % 6) / 6.0, i < 6 ? 0.1 : 0.9 });
            meas(8, "Top front edge", FeatureKind.Curve, 25, i => new[] { i / 24.0, 0.5 });
            // a wavy freeform panel next to the block, traced as 4 curves (alternating directions)
            var ff = p.NewFeature(FeatureKind.Surface, "Freeform");
            for (int s = 0; s < 4; s++)
                for (int i = 0; i < 45; i++)
                {
                    double u = i / 44.0, v = s / 3.0;
                    if (s % 2 == 1) u = 1 - u;
                    double x = 230 + u * 110, y = 60 + v * 70;
                    double z = -60 + 9 * Math.Sin(u * 3.0) + 6 * Math.Cos(v * 2.5) + 4 * u * v;
                    Vec3 noise = new Vec3(rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5) * 0.05;
                    ff.Points.Add(new MeasuredPoint(new Vec3(x, y, z) + noise, -Vec3.UnitZ, 0) { Section = s });
                }
            var top = p.Features[0]; var front = p.Features[1];
            var edge = p.NewFeature(FeatureKind.Line, "Top∩Front");
            edge.Origin = FeatureOrigin.Constructed; edge.Construction = "intersect";
            edge.SourceIds.AddRange(new[] { top.Id, front.Id });
            FeatureSolver.SolveAll(p);
            return p;
        }
    }

    public sealed partial class MainWindow
    {
        /// <summary>Binary STL of a box 0..L × 0..W × 0..H (outward triangles).</summary>
        static void WriteBoxStl(string path, double L, double W, double H)
        {
            Vec3[] v = { new Vec3(0, 0, 0), new Vec3(L, 0, 0), new Vec3(L, W, 0), new Vec3(0, W, 0), new Vec3(0, 0, H), new Vec3(L, 0, H), new Vec3(L, W, H), new Vec3(0, W, H) };
            int[][] quads = { new[] { 0, 3, 2, 1 }, new[] { 4, 5, 6, 7 }, new[] { 0, 1, 5, 4 }, new[] { 3, 7, 6, 2 }, new[] { 0, 4, 7, 3 }, new[] { 1, 2, 6, 5 } };
            using (var w = new BinaryWriter(File.Create(path)))
            {
                w.Write(new byte[80]);
                w.Write((uint)12);
                foreach (var q in quads)
                    foreach (var tri in new[] { new[] { q[0], q[1], q[2] }, new[] { q[0], q[2], q[3] } })
                    {
                        for (int k = 0; k < 3; k++) w.Write(0f);
                        foreach (int i in tri) { w.Write((float)v[i].X); w.Write((float)v[i].Y); w.Write((float)v[i].Z); }
                        w.Write((ushort)0);
                    }
            }
        }

        /// <summary>Hidden "/selftest dir" mode: drives the UI, writes screenshots + exports, then exits.</summary>
        public void RunSelfTest(string dir)
        {
            Directory.CreateDirectory(dir);
            var log = new List<string>();
            var steps = new List<Action>();
            Action<string> shot = name =>
            {
                UpdateLayout();
                var w = (int)Math.Max(1, ActualWidth); var h = (int)Math.Max(1, ActualHeight);
                var rtb = new RenderTargetBitmap(w, h, 96, 96, PixelFormats.Pbgra32);
                rtb.Render(this);
                var enc = new PngBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create(rtb));
                using (var fs = File.Create(Path.Combine(dir, name + ".png"))) enc.Save(fs);
                log.Add("screenshot " + name);
            };
            Func<string, Feature> F = n => _proj.Features.First(x => x.Name.StartsWith(n));
            Action<string> check = msg => log.Add("CHECK " + msg);

            // start clean: no saved work area, millimetres
            _s.WorkArea = new WorkArea();
            SetUnits(false);

            steps.Add(() => { _proj = Demo.Build(); _selected = F("Bore"); RebuildAll(); _view.SetView("iso"); FitView(); });
            steps.Add(() => shot("01-features"));
            steps.Add(() =>
            {
                var r = _proj.AlignmentRecipe;
                r.Orientation = OrientationMode.Features;
                r.PrimaryId = F("Top 1").Id;
                r.SecondaryId = F("Top∩Front").Id;
                r.Origin = OriginMode.BoundingBox;
                r.AnchorX = Anchor.Min; r.AnchorY = Anchor.Min; r.AnchorZ = Anchor.Min;
                r.BoxFeatureIds.AddRange(_proj.Features.Where(x => x.Origin == FeatureOrigin.Measured && !x.Name.StartsWith("Ball") && !x.Name.StartsWith("Freeform")).Select(x => x.Id));
                _tabs.SelectedIndex = 2;
                Changed(); FitView();
            });
            steps.Add(() => shot("02-alignment-corner"));

            // inches + plane resize handles
            steps.Add(() => { SetUnits(true); _tabs.SelectedIndex = 0; SelectFeature(F("Top 1")); });
            steps.Add(() => shot("03-inch-plane-selected"));
            steps.Add(() =>
            {
                // drag the +U edge handle (index 5) of the Top plane outward by 40 mm through the real drag path
                var f = F("Top 1"); var pl = (PlanePrim)f.Geometry;
                Vec3 c, u; double w, h;
                PlaneBounds.Get(f, pl, out c, out u, out w, out h);
                Vec3 target = c + u * (w / 2 + 40);
                Point sp;
                _view.Project(DisplayMatrix.TransformPoint(target), out sp);
                Vec3 o, d;
                _view.Unproject(sp, out o, out d);
                var tag = new PickTag { FeatureId = f.Id, Handle = "plane", HandleIndex = 5 };
                OnDragStarted(tag); OnDragging(tag, o, d); OnDragEnded(tag);
                check(string.Format(Ui.CI, "plane width {0:0.0} -> {1:0.0} mm (expected ≈ {2:0.0})", w, f.BW, w + 40));
            });
            steps.Add(() => shot("04-plane-resized-inch"));

            // click measurements (through the pick routing)
            steps.Add(() =>
            {
                SetUnits(false);
                SetTool(Tool.Distance); PickEntity(new EntityRef { FeatureId = F("Bore").Id }); PickEntity(new EntityRef { FeatureId = F("Boss").Id });
                SetTool(Tool.Diameter); PickEntity(new EntityRef { FeatureId = F("Bore").Id });
                SetTool(Tool.Angle); PickEntity(new EntityRef { FeatureId = F("Top 1").Id }); PickEntity(new EntityRef { FeatureId = F("Front").Id });
                SetTool(Tool.Distance); PickEntity(new EntityRef { FeatureId = F("Top 1").Id }); PickEntity(new EntityRef { FeatureId = F("Countersink").Id, PointIndex = 0 });
                SetTool(Tool.Coordinate); PickEntity(new EntityRef { FeatureId = F("Ball").Id });
                SetTool(Tool.Select);
                _tabs.SelectedIndex = 1; _selected = null;
                RebuildAll();
                foreach (var d in _proj.Dimensions) check(DimText(d, DimensionSolver.Evaluate(_proj, d)));
                _view.SetView("iso"); FitFeatures();
            });
            steps.Add(() => shot("05-measurements"));

            // nominal values and tolerances: one in, one out
            steps.Add(() =>
            {
                var d1 = _proj.Dimensions[0]; d1.Toleranced = true; d1.Nominal = 35; d1.TolPlus = 0.05; d1.TolMinus = -0.05;
                var d2 = _proj.Dimensions[1]; d2.Toleranced = true; d2.Nominal = 20; d2.TolPlus = 0.005; d2.TolMinus = -0.005;
                _selectedDim = d2.Id;
                RebuildMeasurePanel(); RebuildDims();
                foreach (var d in new[] { d1, d2 }) check(d.Name + ": " + CadExport.ToleranceLine(_proj, d, DimensionSolver.Evaluate(_proj, d), Fmt));
            });
            steps.Add(() => shot("05b-tolerances"));

            // geometric tolerances by clicking feature (and datum)
            steps.Add(() =>
            {
                SetTool(Tool.Form); PickEntity(new EntityRef { FeatureId = F("Top 1").Id });
                SetTool(Tool.Perpendicular); PickEntity(new EntityRef { FeatureId = F("Front").Id }); PickEntity(new EntityRef { FeatureId = F("Top 1").Id });
                SetTool(Tool.Perpendicular); PickEntity(new EntityRef { FeatureId = F("Bore").Id }); PickEntity(new EntityRef { FeatureId = F("Top 1").Id });
                SetTool(Tool.Position); PickEntity(new EntityRef { FeatureId = F("Bore").Id });
                SetTool(Tool.Coaxial); PickEntity(new EntityRef { FeatureId = F("Boss").Id }); PickEntity(new EntityRef { FeatureId = F("Bore").Id });
                SetTool(Tool.Select);
                var flat = _proj.Dimensions.First(x => x.Kind == DimKind.Form);
                flat.Toleranced = true; flat.TolPlus = 0.05; flat.MinZone = true;
                _selectedDim = flat.Id;
                RebuildMeasurePanel(); RebuildDims();
                foreach (var d in _proj.Dimensions.Where(x => Gdt.IsGdt(x.Kind))) check(DimText(d, DimensionSolver.Evaluate(_proj, d)));
            });
            steps.Add(() => shot("05c-gdt"));

            // freeform surface + curve modes
            steps.Add(() => { _tabs.SelectedIndex = 0; SelectFeature(F("Freeform")); FitFeatures(); });
            steps.Add(() => shot("06-freeform"));
            steps.Add(() =>
            {
                var c = F("Top front edge"); c.CurveMode = CurveMode.Linear; Changed(); SelectFeature(c);
                check("curve linear: " + string.Join(" | ", CadExport.Describe(c.Geometry, c, Fmt)));
                c.CurveMode = CurveMode.Spline; Changed();
                check("curve spline: " + string.Join(" | ", CadExport.Describe(c.Geometry, c, Fmt)));
            });
            steps.Add(() => shot("07-curve-spline"));

            // work area: sketch a polygon on the table by clicking, set the height, Fit
            steps.Add(() => { _tabs.SelectedIndex = 4; SetTool(Tool.SketchArea); });
            steps.Add(() =>
            {
                var corners = new[] { new Vec3(120, -200, 0), new Vec3(420, -200, 0), new Vec3(470, 60, 0), new Vec3(330, 220, 0), new Vec3(100, 120, 0) };
                foreach (var q in corners)
                {
                    Point sp;
                    _view.Project(DisplayMatrix.TransformPoint(q), out sp);
                    SketchClick(sp);
                }
                check("sketched " + _sketch.Count + " vertices");
                Point first; _view.Project(DisplayMatrix.TransformPoint(corners[0]), out first);
                SketchClick(first); // clicking the first vertex closes the outline
                _s.WorkArea.Height = 180;
                RebuildScene(); RebuildWorkspacePanel(); FitView();
                check(string.Format("work area defined={0} corners={1}", _s.WorkArea.IsDefined, _s.WorkArea.Outline.Count));
            });
            steps.Add(() => shot("08-workarea-fit"));

            // arm check: single-point readings through the capture sink (not saved to the settings)
            steps.Add(() =>
            {
                _tabs.SelectedIndex = 3;
                ToggleCheck(CheckMode.SinglePoint);
                var rng = new Random(7);
                for (int i = 0; i < 12; i++)
                    CheckCapture(new Vec3(300, 40, 20) + new Vec3(rng.NextDouble() - 0.5, rng.NextDouble() - 0.5, rng.NextDouble() - 0.5) * 0.12, -Vec3.UnitZ);
                check("single-point test: " + _checkText.Text);
                UpdateLayout();
                _checkText.BringIntoView();
            });
            steps.Add(() => shot("09-arm-check"));
            steps.Add(() => { ToggleCheck(CheckMode.SinglePoint); _tabs.SelectedIndex = 0; });

            // nominal model: the demo block as STL in part coordinates, best fit over three faces, profile
            steps.Add(() =>
            {
                string stl = Path.Combine(dir, "block.stl");
                WriteBoxStl(stl, 120, 80, 30);
                LoadModel(stl, 1, true);
                Mat4 before = _proj.AlignmentMatrix;
                BestFitToModel(new List<Feature> { F("Top 1"), F("Front"), F("Left") });
                check("model fit: " + _proj.AlignmentRecipe.ModelNote);
                Vec3 shift = _proj.AlignmentMatrix.TransformPoint(Vec3.Zero) - before.TransformPoint(Vec3.Zero);
                check(string.Format(Ui.CI, "part frame moved by {0:0.000} {1:0.000} {2:0.000} mm", shift.X, shift.Y, shift.Z));
                SetTool(Tool.Profile); PickEntity(new EntityRef { FeatureId = F("Top 1").Id }); SetTool(Tool.Select);
                var pd = _proj.Dimensions.Last();
                check(DimText(pd, DimensionSolver.Evaluate(_proj, pd)));
                _s.ColorByDeviation = true; _s.DeviationRange = 0.05;
                _tabs.SelectedIndex = 2;
                RebuildAll(); _view.SetView("iso"); FitFeatures();
            });
            steps.Add(() => shot("10-model"));
            steps.Add(() => { _s.ColorByDeviation = false; _proj.NominalModel = null; _proj.ModelPath = ""; _tabs.SelectedIndex = 0; RebuildAll(); });

            steps.Add(() =>
            {
                string step = Path.Combine(dir, "demo.step");
                log.Add("STEP: " + CadExport.WriteStep(_proj, step, new ExportOptions()));
                File.WriteAllText(Path.Combine(dir, "report.txt"), CadExport.Report(_proj, true, Fmt, CapabilityLine()));
                File.WriteAllText(Path.Combine(dir, "report.html"), HtmlReport.Build(_proj, Fmt, ReportHeader(),
                    new[] { new KeyValuePair<string, byte[]>("3D view", SnapshotView()) }), new System.Text.UTF8Encoding(false));
                ProjectIO.Save(_proj, Path.Combine(dir, "demo.msproj"));
                try { Sounds.Dump(Path.Combine(dir, "sounds")); } catch (Exception ex) { log.Add("sounds: " + ex.Message); }
                foreach (var f in _proj.Features) log.Add(f.Name + ": " + Summary(f, false));
                _dirty = false;
                _s.WorkArea = new WorkArea();
            });
            // inspection plan: replay the demo project on a "next part" (the plan's points, slightly moved), log three parts
            string keepLog = null;
            steps.Add(() =>
            {
                keepLog = _s.ResultsLogPath;
                var learned = ProjectIO.Clone(_proj);
                StartPlan(learned, "Demo plan", "S001");
                var rng = new Random(3);
                int guard = 0;
                while (PlanRunning && guard++ < 100)
                {
                    var f = StepFeature; var src = PlanSteps[_planStep];
                    if (f.Kind == FeatureKind.Curve || f.Kind == FeatureKind.Surface)
                    {
                        foreach (var q in src.Points) f.Points.Add(new MeasuredPoint(q.Position, q.Direction, q.ProbeRadius) { Section = q.Section });
                        FeatureSolver.Solve(_proj, f);
                        FinishMeasure();
                        continue;
                    }
                    foreach (var q in src.ActivePoints.ToList())
                    {
                        if (StepFeature != f) break;
                        AddPoint(q.Position + new Vec3(rng.NextDouble() - 0.5, rng.NextDouble() - 0.5, rng.NextDouble() - 0.5) * 0.02, q.Direction);
                    }
                    if (PlanRunning && StepFeature == f) FinishMeasure();
                }
                check(string.Format("plan run: done={0}, {1} features measured", _planDone, _proj.Features.Count(x => x.Geometry != null)));
                _s.ResultsLogPath = Path.Combine(dir, "results-log.csv");
                foreach (var serial in new[] { "S001", "S002", "S003" })
                {
                    foreach (var f in _proj.Features.Where(x => x.Name.StartsWith("Bore")))
                        foreach (var q in f.Points) q.Position += new Vec3(0.004, 0, 0);
                    FeatureSolver.SolveAll(_proj);
                    _proj.PartSerial = serial;
                    Results.AppendCsv(_s.ResultsLogPath, _proj, ReportHeader(), Fmt);
                }
                _spcKey = "Distance 1 · " + DimensionSolver.Evaluate(_proj, _proj.Dimensions[0]).Title;
                foreach (var r in Results.Rows(_proj).Where(x => x.Status != TolStatus.None)) check(string.Format(Ui.CI, "plan result {0}: {1:0.000} {2}", r.Name, r.Actual, r.Status));
                RebuildPlanPanel();
            });
            steps.Add(() => shot("11-plan"));
            steps.Add(() => { _s.ResultsLogPath = keepLog; StopPlan(); });
            steps.Add(() => { File.WriteAllLines(Path.Combine(dir, "selftest.log"), log); _dirty = false; DisconnectArm(); Close(); });

            int k = 0;
            var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(400) };
            timer.Tick += (s, e) =>
            {
                if (k >= steps.Count) { timer.Stop(); return; }
                try { steps[k++](); }
                catch (Exception ex) { log.Add("step " + (k - 1) + " failed: " + ex); }
            };
            timer.Start();
        }
    }
}
