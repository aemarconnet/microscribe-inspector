using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using MicroScribe.Core;
using MicroScribe.Export;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.Tests
{
    static class FreeformTests
    {
        public static void Register()
        {
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("curve: linear vs smoothed spline", TestCurves));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("freeform loft through traced curves", TestLoft));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("plane resize + extend to plane", TestPlaneBounds));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("click dimensions", TestDimensions));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("STEP with freeform surface + curves", TestStepFreeform));
        }

        static readonly Random Rng = new Random(99);
        static double G(double s) { double u1 = 1 - Rng.NextDouble(), u2 = Rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

        /// <summary>Height field used as the "true" freeform surface.</summary>
        static double Z(double x, double y) { return 10 * Math.Sin(x / 40) + 6 * Math.Cos(y / 30) + 0.002 * x * y; }

        static void TestCurves()
        {
            // noisy trace of a sine wave, 1 mm spacing
            var pts = new List<Vec3>();
            for (int i = 0; i <= 150; i++) pts.Add(new Vec3(i, 15 * Math.Sin(i / 20.0), 0) + new Vec3(G(0.03), G(0.03), G(0.03)));
            var lin = CurvePrim.Build(pts, CurveMode.Linear, 0, false);
            Check(lin.Degree == 1 && lin.Ctrl.Length == pts.Count, "linear keeps every point");
            var sm = CurvePrim.Build(pts, CurveMode.Spline, 0.1, false);
            Check(sm.Degree == 3, "spline degree");
            Check(sm.Ctrl.Length < pts.Count / 3, "smoothing should reduce control points: {0}", sm.Ctrl.Length);
            Check(sm.Stats.MaxAbs <= 0.1 + 1e-9, "max deviation {0} > tol", sm.Stats.MaxAbs);
            // the smoothed curve should be closer to the true sine than the noise
            double maxTrue = 0;
            foreach (var s in sm.Samples) maxTrue = Math.Max(maxTrue, Math.Abs(s.Y - 15 * Math.Sin(s.X / 20.0)));
            Check(maxTrue < 0.12, "smoothed curve deviates from truth by {0}", maxTrue);
            var closed = CurvePrim.Build(pts.Take(50).ToList(), CurveMode.Linear, 0, true);
            Check((closed.Ctrl[0] - closed.Ctrl[closed.Ctrl.Length - 1]).Length < 1e-9, "closed curve returns to start");
        }

        static void TestLoft()
        {
            // 5 traced curves across a curved surface, alternating trace directions, uneven point counts
            var secs = new List<List<Vec3>>();
            for (int s = 0; s < 5; s++)
            {
                double y = s * 25;
                var c = new List<Vec3>();
                int n = 60 + s * 7;
                for (int i = 0; i < n; i++)
                {
                    double x = 100.0 * i / (n - 1);
                    c.Add(new Vec3(x, y + 2 * Math.Sin(x / 30), Z(x, y + 2 * Math.Sin(x / 30))) + new Vec3(G(0.02), G(0.02), G(0.02)));
                }
                if (s % 2 == 1) c.Reverse();
                secs.Add(c);
            }
            string err;
            var surf = Loft.Build(secs, 0, out err);
            Check(surf != null, "loft failed: {0}", err);
            Check(surf.Stats.Rms < 0.1, "loft RMS {0}", surf.Stats.Rms);
            // between the sections the surface should still follow the true field reasonably
            double maxErr = 0;
            for (int i = 1; i < 10; i++)
                for (int j = 1; j < 10; j++)
                {
                    Vec3 p = surf.Evaluate(i / 10.0, j / 10.0);
                    maxErr = Math.Max(maxErr, Math.Abs(p.Z - Z(p.X, p.Y)));
                }
            Check(maxErr < 1.0, "interior error {0}", maxErr);
            // project integration: measured surface feature with sections
            var proj = new Project();
            var f = proj.NewFeature(FeatureKind.Surface);
            for (int s = 0; s < secs.Count; s++) foreach (var p in secs[s]) f.Points.Add(new MeasuredPoint(p, -Vec3.UnitZ, 0) { Section = s });
            FeatureSolver.SolveAll(proj);
            Check(f.Geometry is SurfacePrim, "surface feature solved: {0}", f.FitError);
            // loft constructed from curve features
            var ids = new List<int>();
            foreach (var s in secs)
            {
                var cf = proj.NewFeature(FeatureKind.Curve);
                foreach (var p in s) cf.Points.Add(new MeasuredPoint(p, -Vec3.UnitZ, 0));
                ids.Add(cf.Id);
            }
            var lf = proj.NewFeature(FeatureKind.Surface, "Loft");
            lf.Origin = FeatureOrigin.Constructed; lf.Construction = "loft"; lf.SourceIds.AddRange(ids);
            FeatureSolver.SolveAll(proj);
            Check(lf.Geometry is SurfacePrim, "loft construction: {0}", lf.FitError);
            Check(lf.Geometry.Stats.Rms < 0.15, "loft construction RMS {0}", lf.Geometry.Stats.Rms);
        }

        static void TestPlaneBounds()
        {
            var proj = new Project();
            var top = proj.NewFeature(FeatureKind.Plane, "Top");
            foreach (var p in new[] { new Vec3(10, 10, 20), new Vec3(60, 10, 20), new Vec3(60, 40, 20), new Vec3(10, 40, 20), new Vec3(30, 25, 20) })
                top.Points.Add(new MeasuredPoint(p, -Vec3.UnitZ, 0));
            var front = proj.NewFeature(FeatureKind.Plane, "Front");
            foreach (var p in new[] { new Vec3(10, 0, 2), new Vec3(60, 0, 2), new Vec3(60, 0, 15), new Vec3(10, 0, 15) })
                front.Points.Add(new MeasuredPoint(p, Vec3.UnitY, 0));
            top.ExportMargin = 0;
            FeatureSolver.SolveAll(proj);
            var pl = (PlanePrim)top.Geometry;
            Vec3 c, u; double w, h;
            PlaneBounds.Get(top, pl, out c, out u, out w, out h);
            Check(Math.Abs(w * h - 50 * 30) < 1e-6, "fitted rectangle area {0}", w * h);
            // resize: 80 × 60 centred at (35, 25)
            PlaneBounds.Set(top, new Vec3(35, 25, 0), Vec3.UnitX, 80, 60);
            var k = PlaneBounds.Corners(top, pl);
            Check(Math.Abs(k[0].Z - 20) < 1e-9, "corners projected onto the plane");
            Check(Math.Abs((k[1] - k[0]).Length - 80) < 1e-9, "custom width");
            // extend the top plane to meet the front plane (y = 0)
            Check(PlaneBounds.ExtendTo(top, pl, (PlanePrim)front.Geometry), "extend failed");
            k = PlaneBounds.Corners(top, pl);
            Check(Math.Abs(k.Min(q => q.Y)) < 1e-6, "edge must lie on y = 0, got {0}", k.Min(q => q.Y));
            Check(Math.Abs(k.Max(q => q.Y) - 55) < 1e-6, "far edge unchanged, got {0}", k.Max(q => q.Y));
        }

        static void TestDimensions()
        {
            Mat4 truth;
            var p = ExportTests.DemoBlock(out truth);
            var bore = p.Features.First(f => f.Name.StartsWith("Bore"));
            var top = p.Features.First(f => f.Name.StartsWith("Top"));
            var d = new Dimension { Kind = DimKind.Diameter, Refs = { new EntityRef { FeatureId = bore.Id } } };
            var r = DimensionSolver.Evaluate(p, d);
            Check(r.Ok && Math.Abs(r.Value - 16) < 0.01, "bore diameter {0}", r.Value);
            var dist = new Dimension { Kind = DimKind.Distance, Refs = { new EntityRef { FeatureId = top.Id }, new EntityRef { FeatureId = bore.Id, PointIndex = 0 } } };
            r = DimensionSolver.Evaluate(p, dist);
            Check(r.Ok && Math.Abs(r.Value - 16) < 0.01, "point (z=4) to top (z=20) {0}", r.Value);
            var ang = new Dimension { Kind = DimKind.Angle, Refs = { new EntityRef { FeatureId = top.Id }, new EntityRef { FeatureId = bore.Id } } };
            r = DimensionSolver.Evaluate(p, ang);
            Check(r.Ok && Math.Abs(r.Value - 90) < 0.1, "bore axis is perpendicular to the top plane: {0}", r.Value);
            // three-point angle
            var tri = new Dimension { Kind = DimKind.Angle, Refs = { new EntityRef { FeatureId = top.Id, PointIndex = 0 }, new EntityRef { FeatureId = top.Id, PointIndex = 1 }, new EntityRef { FeatureId = top.Id, PointIndex = 2 } } };
            r = DimensionSolver.Evaluate(p, tri);
            Check(r.Ok && r.Value > 0 && r.Value < 180, "3-point angle {0}", r.Value);
            Check(DimensionSolver.PicksNeeded(DimKind.Angle, p, tri.Refs.Take(1).ToList()) == 3, "point start → 3 picks");
        }

        static void TestStepFreeform()
        {
            var proj = new Project { Title = "freeform" };
            var f = proj.NewFeature(FeatureKind.Surface);
            for (int s = 0; s < 4; s++)
                for (int i = 0; i < 40; i++)
                {
                    double x = i * 2.5, y = s * 20;
                    f.Points.Add(new MeasuredPoint(new Vec3(x, y, Z(x, y)), -Vec3.UnitZ, 0) { Section = s });
                }
            var c = proj.NewFeature(FeatureKind.Curve);
            for (int i = 0; i < 30; i++) c.Points.Add(new MeasuredPoint(new Vec3(i * 3, -10, Math.Sin(i / 4.0)), -Vec3.UnitZ, 0));
            var cl = proj.NewFeature(FeatureKind.Curve);
            cl.CurveMode = CurveMode.Linear;
            for (int i = 0; i < 5; i++) cl.Points.Add(new MeasuredPoint(new Vec3(i * 10, -20, 0), -Vec3.UnitZ, 0));
            FeatureSolver.SolveAll(proj);
            string path = Path.Combine(Path.GetTempPath(), "msinspector_freeform.step");
            CadExport.WriteStep(proj, path, new ExportOptions());
            string txt = File.ReadAllText(path);
            Check(txt.Contains("B_SPLINE_SURFACE_WITH_KNOTS"), "surface entity");
            Check(Regex.Matches(txt, "B_SPLINE_CURVE_WITH_KNOTS").Count == 5, "4 boundary curves + 1 spline curve");
            Check(txt.Contains("POLYLINE('Curve 2'"), "linear curve as polyline");
            var defined = new HashSet<string>(Regex.Matches(txt, @"^#(\d+)=", RegexOptions.Multiline).Cast<Match>().Select(m => m.Groups[1].Value));
            foreach (Match m in Regex.Matches(txt, @"#(\d+)"))
                Check(defined.Contains(m.Groups[1].Value), "undefined reference #{0}", m.Groups[1].Value);
            Console.WriteLine("      wrote " + path);
        }

        static void Check(bool c, string m, params object[] a) { Program.Check(c, m, a); }
    }
}
