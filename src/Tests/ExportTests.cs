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
    static class ExportTests
    {
        static ExportTests() { }

        public static void Register()
        {
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("bounding-box corner/centre alignment", TestAnchors));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("probe compensation", TestCompensation));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("spline interpolation", TestSpline));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("STEP export structure", TestStep));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("project XML round-trip", TestXml));
        }

        /// <summary>A synthetic 100 x 60 x 20 block with a bore, measured in a rotated/translated pose.</summary>
        public static Project DemoBlock(out Mat4 partToMachine)
        {
            double a = 25 * Math.PI / 180;
            Vec3 x = new Vec3(Math.Cos(a), Math.Sin(a), 0), y = new Vec3(-Math.Sin(a), Math.Cos(a), 0), z = Vec3.UnitZ;
            Vec3 o = new Vec3(250, 40, -30);
            partToMachine = Mat4.FromFrame(o, x, y, z);
            var M = partToMachine;
            var p = new Project { Title = "Demo block" };
            var rnd = new Random(7);
            Func<Vec3, Vec3, MeasuredPoint> mp = (q, d) => new MeasuredPoint(M.TransformPoint(q), M.TransformVector(d).Normalized(), 0);

            var top = p.NewFeature(FeatureKind.Plane, "Top");
            for (int i = 0; i < 12; i++) top.Points.Add(mp(new Vec3(5 + rnd.NextDouble() * 90, 5 + rnd.NextDouble() * 50, 20), -Vec3.UnitZ));
            var front = p.NewFeature(FeatureKind.Plane, "Front");
            for (int i = 0; i < 8; i++) front.Points.Add(mp(new Vec3(5 + rnd.NextDouble() * 90, 0, 3 + rnd.NextDouble() * 14), Vec3.UnitY));
            var left = p.NewFeature(FeatureKind.Plane, "Left");
            for (int i = 0; i < 8; i++) left.Points.Add(mp(new Vec3(0, 5 + rnd.NextDouble() * 50, 3 + rnd.NextDouble() * 14), Vec3.UnitX));
            var bore = p.NewFeature(FeatureKind.Cylinder, "Bore");
            for (int i = 0; i < 16; i++)
            {
                double t = i * 2 * Math.PI / 16;
                Vec3 radial = new Vec3(Math.Cos(t), Math.Sin(t), 0);
                bore.Points.Add(mp(new Vec3(70, 30, 0) + radial * 8 + new Vec3(0, 0, 4 + (i % 4) * 4), radial));
            }
            FeatureSolver.SolveAll(p);
            return p;
        }

        static void TestAnchors()
        {
            Mat4 truth;
            var p = DemoBlock(out truth);
            var top = p.Features[0]; var front = p.Features[1]; var left = p.Features[2];
            var r = p.AlignmentRecipe;
            r.Orientation = OrientationMode.Features;
            r.PrimaryId = top.Id; r.SecondaryId = front.Id;
            r.Origin = OriginMode.BoundingBox;
            r.AnchorX = Anchor.Min; r.AnchorY = Anchor.Min; r.AnchorZ = Anchor.Min;
            // front plane normal points toward the probe (-Y in part); secondary = plane normal => X along it.
            // Use a line-like secondary instead: the intersection of top and front.
            var edge = p.NewFeature(FeatureKind.Line, "Top/Front edge");
            edge.Origin = FeatureOrigin.Constructed; edge.Construction = "intersect";
            edge.SourceIds.AddRange(new[] { top.Id, front.Id });
            r.SecondaryId = edge.Id;
            FeatureSolver.SolveAll(p);
            Check(p.AlignmentError == null, "alignment error: {0}", p.AlignmentError);
            Mat4 m = p.AlignmentMatrix;
            // The probed corner region: the box min corner should be near the measured extremes.
            Vec3 a, b; string e;
            AlignmentSolver.Solve(p, r, out a, out b, out e);
            Check(Math.Abs(a.X) < 1e-6 && Math.Abs(a.Y) < 1e-6 && Math.Abs(a.Z) < 1e-6, "min corner should be origin, got {0}", a);
            // Bore axis must be parallel to part Z
            var bore = GeoUtil.Transform(p.Features[3].Geometry, m) as CylinderPrim;
            Check(Math.Abs(Math.Abs(bore.Axis.Z) - 1) < 1e-3, "bore axis {0}", bore.Axis);
            // centre anchor
            r.AnchorX = r.AnchorY = r.AnchorZ = Anchor.Center;
            AlignmentSolver.Solve(p, r, out a, out b, out e);
            Check((a + b).Length < 1e-6, "centre anchor should be symmetric: {0} {1}", a, b);
            // quarter turn about Z swaps X/Y extents
            Vec3 a2, b2;
            r.TurnsZ = 1;
            AlignmentSolver.Solve(p, r, out a2, out b2, out e);
            Check(Math.Abs((b2.X - a2.X) - (b.Y - a.Y)) < 1e-6, "turn should swap extents");
        }

        static void TestCompensation()
        {
            // 3 mm ball on a 20 mm diameter boss: centres lie on R = 13
            var p = new Project();
            var f = p.NewFeature(FeatureKind.Cylinder);
            for (int i = 0; i < 12; i++)
            {
                double t = i * Math.PI / 6;
                Vec3 radial = new Vec3(Math.Cos(t), Math.Sin(t), 0);
                f.Points.Add(new MeasuredPoint(radial * 13 + new Vec3(0, 0, i), -radial, 3));
            }
            var hole = p.NewFeature(FeatureKind.Cylinder);
            for (int i = 0; i < 12; i++)
            {
                double t = i * Math.PI / 6;
                Vec3 radial = new Vec3(Math.Cos(t), Math.Sin(t), 0);
                hole.Points.Add(new MeasuredPoint(radial * 7 + new Vec3(0, 0, i), radial, 3));
            }
            var pl = p.NewFeature(FeatureKind.Plane);
            for (int i = 0; i < 6; i++) pl.Points.Add(new MeasuredPoint(new Vec3(i * 3, i * i, 3), -Vec3.UnitZ, 3));
            FeatureSolver.SolveAll(p);
            Check(Math.Abs(((CylinderPrim)f.Geometry).Radius - 10) < 1e-6, "boss radius {0}", ((CylinderPrim)f.Geometry).Radius);
            Check(Math.Abs(((CylinderPrim)hole.Geometry).Radius - 10) < 1e-6, "bore radius {0}", ((CylinderPrim)hole.Geometry).Radius);
            var g = (PlanePrim)pl.Geometry;
            Check(Math.Abs(g.SignedDistance(Vec3.Zero)) < 1e-6, "plane should move to z=0");
        }

        static void TestSpline()
        {
            var q = new List<Vec3>();
            for (int i = 0; i <= 20; i++) { double t = i / 20.0 * Math.PI; q.Add(new Vec3(Math.Cos(t) * 50, Math.Sin(t) * 50, t * 5)); }
            Vec3[] c; double[] k;
            Spline.Interpolate(q, 3, out c, out k);
            Check(c.Length == q.Count, "ctrl count");
            Check((Spline.Evaluate(3, c, k, 0) - q[0]).Length < 1e-9, "start");
            Check((Spline.Evaluate(3, c, k, 1) - q[20]).Length < 1e-9, "end");
            // interior interpolation: reconstruct parameters
            double total = 0; for (int i = 1; i < q.Count; i++) total += (q[i] - q[i - 1]).Length;
            double u = 0;
            for (int i = 1; i < q.Count - 1; i++)
            {
                u += (q[i] - q[i - 1]).Length / total;
                Check((Spline.Evaluate(3, c, k, u) - q[i]).Length < 1e-6, "interp point {0}", i);
            }
        }

        static void TestStep()
        {
            Mat4 truth;
            var p = DemoBlock(out truth);
            var sph = p.NewFeature(FeatureKind.Sphere);
            var rnd = new Random(3);
            for (int i = 0; i < 20; i++)
            {
                Vec3 d = new Vec3(rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5, rnd.NextDouble()).Normalized();
                sph.Points.Add(new MeasuredPoint(new Vec3(0, 0, 100) + d * 12.7, -d, 0));
            }
            var cone = p.NewFeature(FeatureKind.Cone);
            for (int i = 0; i < 24; i++)
            {
                double t = i * 2 * Math.PI / 24, h = 10 + (i % 3) * 5, r = h * Math.Tan(30 * Math.PI / 180);
                cone.Points.Add(new MeasuredPoint(new Vec3(Math.Cos(t) * r, Math.Sin(t) * r, h) + new Vec3(-100, 0, 0), new Vec3(-Math.Cos(t), -Math.Sin(t), 0), 0));
            }
            var curve = p.NewFeature(FeatureKind.Curve);
            for (int i = 0; i < 10; i++) curve.Points.Add(new MeasuredPoint(new Vec3(i * 10, Math.Sin(i) * 5, 0), -Vec3.UnitZ, 0));
            FeatureSolver.SolveAll(p);
            string path = Path.Combine(Path.GetTempPath(), "msinspector_test.step");
            string summary = CadExport.WriteStep(p, path, new ExportOptions());
            string txt = File.ReadAllText(path);
            Check(txt.StartsWith("ISO-10303-21;"), "header");
            Check(Regex.Matches(txt, "SHELL_BASED_SURFACE_MODEL").Count == 6, "6 surface bodies expected: {0}", summary);
            Check(txt.Contains("CYLINDRICAL_SURFACE") && txt.Contains("CONICAL_SURFACE") && txt.Contains("SPHERICAL_SURFACE") && txt.Contains("PLANE("), "surface types");
            Check(txt.Contains("B_SPLINE_CURVE_WITH_KNOTS"), "curve");
            // every #ref must be defined
            var defined = new HashSet<string>(Regex.Matches(txt, @"^#(\d+)=", RegexOptions.Multiline).Cast<Match>().Select(m => m.Groups[1].Value));
            foreach (Match m in Regex.Matches(txt, @"#(\d+)"))
                Check(defined.Contains(m.Groups[1].Value), "undefined reference #{0}", m.Groups[1].Value);
            Console.WriteLine("      wrote " + path + " (" + summary + ")");
        }

        static void TestXml()
        {
            Mat4 truth;
            var p = DemoBlock(out truth);
            p.AlignmentRecipe.Origin = OriginMode.BoundingBox;
            string path = Path.Combine(Path.GetTempPath(), "msinspector_test.msproj");
            ProjectIO.Save(p, path);
            var q = ProjectIO.Load(path);
            Check(q.Features.Count == p.Features.Count, "feature count");
            Check(q.Features[3].Points.Count == p.Features[3].Points.Count, "points");
            Check(q.AlignmentRecipe.Origin == OriginMode.BoundingBox, "recipe");
            var c1 = (CylinderPrim)p.Features[3].Geometry; var c2 = (CylinderPrim)q.Features[3].Geometry;
            Check(Math.Abs(c1.Radius - c2.Radius) < 1e-9, "re-solved geometry");
        }

        static void Check(bool c, string m, params object[] a) { Program.Check(c, m, a); }
    }
}
