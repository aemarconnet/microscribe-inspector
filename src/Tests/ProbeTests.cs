using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.Tests
{
    /// <summary>Probe-ball compensation, probe qualification and arm-check statistics.</summary>
    static class ProbeTests
    {
        public static void Register()
        {
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("ball compensation: freeform surface", TestSurfaceCompensation));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("ball compensation: curves (stylus, outside, inside)", TestCurveCompensation));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("ball compensation: point, line, contact points", TestPointLineContact));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("probe qualification on a reference sphere", TestQualification));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("arm check statistics + stillness", TestArmCheck));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("scan modes, section-plane surface, point clouds", TestScans));
        }

        static void TestScans()
        {
            // distance and time sampling
            var sd = new ScanSampler { Mode = ScanMode.Distance, Spacing = 1 };
            int n = 0;
            for (int i = 0; i <= 100; i++) n += sd.Feed(new Vec3(i * 0.1, 0, 0), -Vec3.UnitZ, i * 16).Count;
            Check(n == 11, "distance sampling kept {0}", n);
            var st = new ScanSampler { Mode = ScanMode.Time, IntervalMs = 100 };
            n = 0;
            for (int i = 0; i <= 62; i++) n += st.Feed(new Vec3(0, 0, 0), -Vec3.UnitZ, i * 16).Count;
            Check(n == 10 || n == 11, "time sampling kept {0}", n);

            // section planes ⟂ x: sweep back and forth along x (across the planes) while creeping along y
            var ss = new ScanSampler { Mode = ScanMode.Sections, Axis = 0, SectionSpacing = 10 };
            var proj = new Project();
            var f = proj.NewFeature(FeatureKind.Surface);
            f.SectionScan = true; f.SectionNormal = ss.PlaneNormal;
            for (int i = 0; i <= 4000; i++)
            {
                double x = 50 + 45 * Math.Sin(i * 0.05), y = 5 + i * 0.022;
                Vec3 nrm = N(x, y), c = new Vec3(x, y, Z(x, y)) + nrm * R;
                foreach (var k in ss.Feed(c, -nrm, i * 16))
                {
                    Check(Math.Abs(k.Tip.X - 10 * k.Section) < 1e-9, "crossing at x = {0} for plane {1}", k.Tip.X, k.Section);
                    f.Points.Add(new MeasuredPoint(k.Tip, k.Direction, R) { Section = k.Section });
                }
            }
            int planes = f.Points.Select(q => q.Section).Distinct().Count();
            Check(planes == 9, "planes crossed {0}", planes);
            FeatureSolver.Solve(proj, f);
            Check(f.Geometry is SurfacePrim, "section scan lofts ({0})", f.FitError);
            double worst = 0;
            foreach (double x in new[] { 20.0, 40, 60, 80 }) foreach (double y in new[] { 20.0, 50, 80 }) worst = Math.Max(worst, Math.Abs(f.Geometry.SignedDistance(new Vec3(x, y, Z(x, y)))));
            Check(worst < 0.1, "section-scan surface off the true surface by {0}", worst);

            // point cloud on a flat face: compensated along the local normal, thinned, strays found
            var cl = proj.NewFeature(FeatureKind.Cloud);
            for (int i = 0; i < 40; i++) for (int j = 0; j < 40; j++)
                cl.Points.Add(new MeasuredPoint(new Vec3(i * 0.5, j * 0.5, R), new Vec3(0.3, 0.1, -1).Normalized(), R));
            cl.Points.Add(new MeasuredPoint(new Vec3(100, 100, 50), -Vec3.UnitZ, R));
            FeatureSolver.Solve(proj, cl);
            var cp = (CloudPrim)cl.Geometry;
            double dz = cp.Points.Take(1600).Max(q => Math.Abs(q.Z));
            Check(dz < 1e-6, "cloud contact points on z = 0 ({0})", dz);
            var keep = CloudTools.Thin(cl.Points.Take(1600).Select(q => q.Position).ToList(), 1.0);
            Check(keep.Count == 400, "thinned to {0}", keep.Count);
            var stray = CloudTools.Isolated(cl.Points.Select(q => q.Position).ToList(), 1.5, 2);
            Check(stray.Count == 1 && stray[0] == 1600, "stray points {0}", stray.Count);
            var xyz = FeatureSolver.ContactPoints(cl, cl.Points);
            Check(xyz.Count == cl.Points.Count && Math.Abs(xyz[5].Z) < 1e-6, "cloud contact points for export");
        }

        static readonly Random Rng = new Random(4242);
        static double G(double s) { double u1 = 1 - Rng.NextDouble(), u2 = Rng.NextDouble(); return s * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }
        static void Check(bool c, string m, params object[] a) { Program.Check(c, m, a); }

        const double R = 1.5; // ball radius

        static double Z(double x, double y) { return 10 * Math.Sin(x / 40) + 6 * Math.Cos(y / 30) + 0.002 * x * y; }
        static Vec3 N(double x, double y)
        {
            double zx = 10.0 / 40 * Math.Cos(x / 40) + 0.002 * y, zy = -6.0 / 30 * Math.Sin(y / 30) + 0.002 * x;
            return new Vec3(-zx, -zy, 1).Normalized();
        }

        static void TestSurfaceCompensation()
        {
            var proj = new Project();
            var f = proj.NewFeature(FeatureKind.Surface);
            for (int s = 0; s < 5; s++)
            {
                double y = s * 25;
                for (int i = 0; i <= 100; i++)
                {
                    double x = i;
                    Vec3 n = N(x, y), c = new Vec3(x, y, Z(x, y)) + n * R + new Vec3(G(0.005), G(0.005), G(0.005));
                    Vec3 dir = (-n + new Vec3(0.2, 0.1, 0)).Normalized(); // stylus into the material, tilted
                    f.Points.Add(new MeasuredPoint(c, dir, R) { Section = s });
                }
            }
            FeatureSolver.Solve(proj, f);
            Check(f.Geometry is SurfacePrim && f.RawFit is SurfacePrim && f.Geometry != f.RawFit, "compensated surface built ({0})", f.FitError);
            double worst = 0, rawMin = double.MaxValue;
            foreach (double y in new[] { 25.0, 50, 75 })
                for (double x = 10; x <= 90; x += 5)
                {
                    var t = new Vec3(x, y, Z(x, y));
                    worst = Math.Max(worst, Math.Abs(f.Geometry.SignedDistance(t)));
                    rawMin = Math.Min(rawMin, Math.Abs(f.RawFit.SignedDistance(t)));
                }
            Check(worst < 0.05, "compensated surface off the true surface by {0}", worst);
            Check(rawMin > 1.3, "ball-centre surface should sit ~1.5 off ({0})", rawMin);
            Check(f.Geometry.Stats.Rms < 0.05, "fit rms {0}", f.Geometry.Stats.Rms);
            // contact points lie on the true surface
            var cps = FeatureSolver.ContactPoints(f, f.Points);
            double cw = 0;
            for (int i = 0; i < cps.Count; i += 7) cw = Math.Max(cw, Math.Abs(cps[i].Z - Z(cps[i].X, cps[i].Y)));
            Check(cw < 0.05, "contact points off the surface by {0}", cw);
            // compensation off: the surface stays at the ball centres
            f.Compensation = CompensationSide.None;
            FeatureSolver.Solve(proj, f);
            Check(f.Geometry == f.RawFit, "None keeps the ball-centre surface");
        }

        static Feature Curve(Project proj, IEnumerable<Vec3> centres, Vec3 dir, CompensationSide side, bool closed)
        {
            var f = proj.NewFeature(FeatureKind.Curve);
            f.CurveMode = CurveMode.Linear;
            f.Closed = closed;
            f.Compensation = side;
            foreach (var c in centres) f.Points.Add(new MeasuredPoint(c, dir, R));
            FeatureSolver.Solve(proj, f);
            Check(f.Geometry is CurvePrim, "curve built ({0})", f.FitError);
            return f;
        }

        static void TestCurveCompensation()
        {
            var proj = new Project();
            // traced over a flat surface z = 0, stylus pointing down and tilted along the travel direction
            var line = Enumerable.Range(0, 60).Select(i => new Vec3(i, 0, R));
            var a = Curve(proj, line, new Vec3(0.3, 0, -1).Normalized(), CompensationSide.Auto, false);
            double dz = ((CurvePrim)a.Geometry).Samples.Max(p => Math.Abs(p.Z) + Math.Abs(p.Y));
            Check(dz < 1e-9, "auto: traced line should drop onto z = 0 ({0})", dz);

            // around a Ø40 boss, ball against the wall, stylus vertical; both trace directions
            Vec3 c0 = new Vec3(50, 50, 5);
            foreach (int sense in new[] { 1, -1 })
            {
                var outside = Enumerable.Range(0, 72).Select(i => c0 + new Vec3(Math.Cos(sense * i * Math.PI / 36), Math.Sin(sense * i * Math.PI / 36), 0) * (20 + R));
                var o = Curve(proj, outside, -Vec3.UnitZ, CompensationSide.Outside, true);
                double e = ((CurvePrim)o.Geometry).Samples.Max(p => Math.Abs((p - c0).Length - 20));
                Check(e < 1e-6, "outside (sense {0}): radius error {1}", sense, e);
                var inside = Enumerable.Range(0, 72).Select(i => c0 + new Vec3(Math.Cos(sense * i * Math.PI / 36), Math.Sin(sense * i * Math.PI / 36), 0) * (20 - R));
                var n = Curve(proj, inside, -Vec3.UnitZ, CompensationSide.Inside, true);
                e = ((CurvePrim)n.Geometry).Samples.Max(p => Math.Abs((p - c0).Length - 20));
                Check(e < 1e-6, "inside (sense {0}): radius error {1}", sense, e);
            }
        }

        static void TestPointLineContact()
        {
            var proj = new Project();
            var pt = proj.NewFeature(FeatureKind.Point);
            for (int i = 0; i < 3; i++) pt.Points.Add(new MeasuredPoint(new Vec3(10, 20, R), -Vec3.UnitZ, R));
            FeatureSolver.Solve(proj, pt);
            Check(Math.Abs(((PointPrim)pt.Geometry).Position.Z) < 1e-9, "point compensated along the stylus");

            var ln = proj.NewFeature(FeatureKind.Line);
            for (int i = 0; i < 10; i++) ln.Points.Add(new MeasuredPoint(new Vec3(i * 5, R, 10), -Vec3.UnitY, R));
            FeatureSolver.Solve(proj, ln);
            var l = (LinePrim)ln.Geometry;
            Check(Math.Abs(l.Origin.Y) < 1e-9 && Math.Abs(l.Origin.Z - 10) < 1e-9, "line moved onto the wall y = 0 ({0})", l.Origin.Y);

            var pl = proj.NewFeature(FeatureKind.Plane);
            for (int i = 0; i < 20; i++) pl.Points.Add(new MeasuredPoint(new Vec3(Rng.NextDouble() * 50, Rng.NextDouble() * 50, R + G(0.01)), -Vec3.UnitZ, R));
            FeatureSolver.Solve(proj, pl);
            var cps = FeatureSolver.ContactPoints(pl, pl.Points);
            for (int i = 0; i < cps.Count; i++)
                Check(Math.Abs(cps[i].Z - (pl.Points[i].Z - R)) < 1e-6, "plane contact point keeps its residual");

            var sp = proj.NewFeature(FeatureKind.Sphere);
            Vec3 c = new Vec3(5, 5, 5);
            for (int i = 0; i < 15; i++)
            {
                double t = i * 0.9, ph = 0.2 + 0.08 * i;
                Vec3 d = new Vec3(Math.Cos(t) * Math.Sin(ph), Math.Sin(t) * Math.Sin(ph), Math.Cos(ph));
                sp.Points.Add(new MeasuredPoint(c + d * (12.7 + R), -d, R));
            }
            FeatureSolver.Solve(proj, sp);
            foreach (var q in sp.Points)
                Check(Math.Abs((FeatureSolver.ContactPoint(sp, q) - c).Length - 12.7) < 1e-4, "sphere contact point on the ball");
            var dim = DimensionSolver.Resolve(proj, new EntityRef { FeatureId = sp.Id, PointIndex = 0 }, out _name) as PointPrim;
            Check(dim != null && Math.Abs((dim.Position - c).Length - 12.7) < 1e-4, "point dimensions use the contact point");
        }

        static string _name;

        static void TestQualification()
        {
            var proj = new Project();
            var sp = proj.NewFeature(FeatureKind.Sphere);
            Vec3 c = new Vec3(200, -40, 30);
            for (int i = 0; i < 20; i++)
            {
                double t = i * 2.4, ph = 0.1 + 1.3 * i / 19.0;   // upper half of the ball
                Vec3 d = new Vec3(Math.Cos(t) * Math.Sin(ph), Math.Sin(t) * Math.Sin(ph), Math.Cos(ph));
                sp.Points.Add(new MeasuredPoint(c + d * (12.7 + 1.52) + new Vec3(G(0.004), G(0.004), G(0.004)), -d, 1.5));
            }
            FeatureSolver.Solve(proj, sp);
            double r = Calibration.QualifiedBallRadius(sp.RawFit, 25.4);
            Check(Math.Abs(r - 1.52) < 0.01, "effective ball radius {0}", r);
            Check(double.IsNaN(Calibration.QualifiedBallRadius(new PointPrim(), 25.4)), "non-sphere gives NaN");
        }

        static void TestArmCheck()
        {
            var sp = ArmCheck.SinglePoint(new[] { new Vec3(0, 0, 0), new Vec3(0.1, 0, 0), new Vec3(-0.1, 0, 0), new Vec3(0, 0.1, 0) });
            Check(sp.N == 4 && Math.Abs(sp.Range - 0.2) < 1e-12, "range {0}", sp.Range);
            Check(Math.Abs(sp.MaxDeviation - Math.Sqrt(0.01 + 0.025 * 0.025)) < 1e-12, "max deviation {0}", sp.MaxDeviation);
            var ln = ArmCheck.Length(new[] { 100.1, 99.9, 100.2 }, 100);
            Check(Math.Abs(ln.MeanError - 0.2 / 3) < 1e-9 && Math.Abs(ln.MaxDeviation - 0.2) < 1e-9 && Math.Abs(ln.Range - 0.3) < 1e-9, "length stats");
            Check(Math.Abs(ln.TwoSigma - 2 * Math.Sqrt(0.07 / 3)) < 1e-6, "length 2 sigma {0}", ln.TwoSigma);
            Check(Math.Abs(ArmCheck.Movement(new[] { Vec3.Zero, new Vec3(0.3, 0, 0), new Vec3(0, 0.4, 0) }) - 0.5) < 1e-12, "movement");
            Vec3 tip, dir;
            var tips = new[] { new Vec3(0, 0, 0), new Vec3(0.03, 0, 0), new Vec3(0, 0.03, 0.02) };
            var dz = -Vec3.UnitZ;
            var dirs = new[] { dz, new Vec3(0.003, 0, -1).Normalized(), new Vec3(0, 0.003, -1).Normalized() };
            Check(ArmCheck.IsStill(tips, dirs, 0.1, 1.0, out tip, out dir), "held still");
            dirs[2] = new Vec3(0, 0.05, -1).Normalized(); // ~2.9° swing
            Check(!ArmCheck.IsStill(tips, dirs, 0.1, 1.0, out tip, out dir), "a swing is not still");
            tips[1] = new Vec3(0.5, 0, 0);
            dirs[2] = dz;
            Check(!ArmCheck.IsStill(tips, dirs, 0.1, 1.0, out tip, out dir), "a moving tip is not still");
        }
    }
}
