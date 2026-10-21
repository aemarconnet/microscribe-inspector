using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Threading;
using MicroScribe.Core;
using MicroScribe.Geometry;

namespace MicroScribe.Tests
{
    /// <summary>Self-checking tests for geometry fitting and exporters (no hardware needed).</summary>
    static class Program
    {
        static int _fail, _pass;
        static Random _rng = new Random(12345);
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        static int Main(string[] args)
        {
            Thread.CurrentThread.CurrentCulture = CI;
            Run("plane", TestPlane);
            Run("line", TestLine);
            Run("circle", TestCircle);
            Run("sphere", TestSphere);
            Run("cylinder", TestCylinder);
            Run("cylinder partial arc", TestCylinderPartial);
            Run("cone", TestCone);
            Run("cone shallow", TestConeShallow);
            Run("constructions", TestConstructions);
            Run("alignment", TestAlignment);
            ExportTests.Register();
            FreeformTests.Register();
            ProbeTests.Register();
            InspectionTests.Register();
            foreach (var t in ExtraTests) Run(t.Key, t.Value);
            Console.WriteLine();
            Console.WriteLine("{0} passed, {1} failed", _pass, _fail);
            return _fail == 0 ? 0 : 1;
        }

        /// <summary>Other test files register here (exporters etc.).</summary>
        public static readonly List<KeyValuePair<string, Action>> ExtraTests = new List<KeyValuePair<string, Action>>();

        static void Run(string name, Action t)
        {
            try { t(); Console.WriteLine("PASS  " + name); _pass++; }
            catch (Exception ex) { Console.WriteLine("FAIL  " + name + ": " + ex.Message); _fail++; }
        }

        public static void Check(bool cond, string msg, params object[] a)
        {
            if (!cond) throw new Exception(string.Format(CI, msg, a));
        }

        static double Gauss(double sigma)
        {
            double u1 = 1 - _rng.NextDouble(), u2 = _rng.NextDouble();
            return sigma * Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2);
        }

        static Vec3 Noise(double s) { return new Vec3(Gauss(s), Gauss(s), Gauss(s)); }

        static Vec3 RandomDir()
        {
            Vec3 v;
            do v = new Vec3(_rng.NextDouble() * 2 - 1, _rng.NextDouble() * 2 - 1, _rng.NextDouble() * 2 - 1); while (v.Length > 1 || v.Length < 0.1);
            return v.Normalized();
        }

        static double AxisErrDeg(Vec3 a, Vec3 b) { return Constructions.AngleBetween(a, b, true) * 180 / Math.PI; }

        const double Sigma = 0.02; // mm, typical digitiser noise is larger, but keeps tolerances tight

        static void TestPlane()
        {
            for (int trial = 0; trial < 20; trial++)
            {
                Vec3 n = RandomDir(), o = new Vec3(100, -50, 30) + Noise(50);
                Vec3 u = n.AnyPerpendicular(), v = n.Cross(u);
                var pts = new List<Vec3>();
                for (int i = 0; i < 30; i++) pts.Add(o + u * (_rng.NextDouble() * 80 - 40) + v * (_rng.NextDouble() * 50 - 25) + n * Gauss(Sigma));
                var f = Fitter.FitPlane(pts.ToArray());
                Check(AxisErrDeg(f.Normal, n) < 0.1, "normal err {0}", AxisErrDeg(f.Normal, n));
                Check(Math.Abs(f.SignedDistance(o)) < 0.05, "offset err");
                Check(f.Stats.Rms < 3 * Sigma, "rms {0}", f.Stats.Rms);
            }
        }

        static void TestLine()
        {
            Vec3 d = RandomDir(), o = new Vec3(10, 20, 30);
            var pts = new List<Vec3>();
            for (int i = 0; i < 15; i++) pts.Add(o + d * (i * 5.0) + Noise(Sigma));
            var f = Fitter.FitLine(pts.ToArray());
            Check(AxisErrDeg(f.Direction, d) < 0.1, "dir err");
            Check(f.Direction.Dot(d) > 0, "orientation should follow capture order");
            Check(Math.Abs(f.TMax - f.TMin - 70) < 0.5, "extent {0}", f.TMax - f.TMin);
        }

        static void TestCircle()
        {
            for (int trial = 0; trial < 20; trial++)
            {
                Vec3 n = RandomDir(), c = Noise(100);
                double r = 5 + _rng.NextDouble() * 50;
                Vec3 u = n.AnyPerpendicular(), v = n.Cross(u);
                var pts = new List<Vec3>();
                double a0 = _rng.NextDouble() * 6, span = 1.5 + _rng.NextDouble() * 4.7; // partial arcs >= 86°
                for (int i = 0; i < 12; i++)
                {
                    double a = a0 + span * i / 11.0;
                    pts.Add(c + (u * Math.Cos(a) + v * Math.Sin(a)) * r + Noise(Sigma));
                }
                var f = Fitter.FitCircle(pts.ToArray());
                // Least-squares optimality: the fit must be at least as good as the true circle
                // (measured in the fitted plane), and close to it.
                var truth = new CirclePrim { Center = c, Normal = f.Normal, Radius = r };
                double truthRms = FitStats.From(truth.Residuals(pts)).Rms;
                Check(f.Stats.Rms <= truthRms + 1e-6, "not optimal: {0} > {1}", f.Stats.Rms, truthRms);
                Check((f.Center - c).Length < 0.5, "center err {0}", (f.Center - c).Length);
                Check(Math.Abs(f.Radius - r) < 0.5, "radius err {0}", f.Radius - r);
            }
        }

        static void TestSphere()
        {
            for (int trial = 0; trial < 20; trial++)
            {
                Vec3 c = Noise(100);
                double r = 3 + _rng.NextDouble() * 40;
                var pts = new List<Vec3>();
                // only a cap (upper ~100°) is reachable in practice
                Vec3 pole = RandomDir();
                for (int i = 0; i < 25; i++)
                {
                    Vec3 d;
                    do d = RandomDir(); while (d.Dot(pole) < 0.2);
                    pts.Add(c + d * r + Noise(Sigma));
                }
                var f = Fitter.FitSphere(pts.ToArray());
                Check((f.Center - c).Length < 0.1, "center err {0}", (f.Center - c).Length);
                Check(Math.Abs(f.Radius - r) < 0.1, "radius err {0}", f.Radius - r);
            }
        }

        static List<Vec3> CylPoints(Vec3 axis, Vec3 o, double r, double len, double a0, double span, int n, out Vec3[] dirs)
        {
            Vec3 u = axis.AnyPerpendicular(), v = axis.Cross(u);
            var pts = new List<Vec3>();
            var ds = new List<Vec3>();
            for (int i = 0; i < n; i++)
            {
                double a = a0 + span * _rng.NextDouble();
                double h = (_rng.NextDouble() - 0.5) * len;
                Vec3 radial = u * Math.Cos(a) + v * Math.Sin(a);
                pts.Add(o + axis * h + radial * r + Noise(Sigma));
                ds.Add((-radial + axis * 0.3 * Gauss(1)).Normalized());
            }
            dirs = ds.ToArray();
            return pts;
        }

        static void TestCylinder()
        {
            for (int trial = 0; trial < 20; trial++)
            {
                Vec3 axis = RandomDir(), o = Noise(100);
                double r = 3 + _rng.NextDouble() * 40;
                double len = r * (0.3 + _rng.NextDouble() * 3); // includes short, disc-like cylinders
                Vec3[] dirs;
                var pts = CylPoints(axis, o, r, len, 0, 2 * Math.PI, 24, out dirs);
                var f = Fitter.FitCylinder(pts.ToArray());
                Check(f != null, "no fit");
                Check(AxisErrDeg(f.Axis, axis) < 0.5, "axis err {0} deg (r={1:0.0}, len={2:0.0})", AxisErrDeg(f.Axis, axis), r, len);
                Check(Math.Abs(f.Radius - r) < 0.05, "radius err {0}", f.Radius - r);
                Check(f.RadialDistance(o) < 0.05, "axis position err {0}", f.RadialDistance(o));
            }
        }

        static void TestCylinderPartial()
        {
            for (int trial = 0; trial < 20; trial++)
            {
                Vec3 axis = RandomDir(), o = Noise(100);
                double r = 10 + _rng.NextDouble() * 30;
                Vec3[] dirs;
                var pts = CylPoints(axis, o, r, r * 2, _rng.NextDouble() * 6, Math.PI * 0.6, 20, out dirs); // 108° arc
                var f = Fitter.FitCylinder(pts.ToArray(), dirs);
                Check(f != null, "no fit");
                Check(AxisErrDeg(f.Axis, axis) < 1.0, "axis err {0} deg", AxisErrDeg(f.Axis, axis));
                Check(Math.Abs(f.Radius - r) / r < 0.01, "radius err {0}", f.Radius - r);
            }
        }

        static void ConeTest(double minHalfDeg, double maxHalfDeg)
        {
            for (int trial = 0; trial < 20; trial++)
            {
                Vec3 axis = RandomDir(), apex = Noise(100);
                double half = (minHalfDeg + _rng.NextDouble() * (maxHalfDeg - minHalfDeg)) * Math.PI / 180;
                Vec3 u = axis.AnyPerpendicular(), v = axis.Cross(u);
                var pts = new List<Vec3>();
                double h0 = 10 + _rng.NextDouble() * 20, h1 = h0 + 10 + _rng.NextDouble() * 30;
                for (int i = 0; i < 30; i++)
                {
                    double a = _rng.NextDouble() * 2 * Math.PI;
                    double h = h0 + (h1 - h0) * _rng.NextDouble();
                    double rr = h * Math.Tan(half);
                    pts.Add(apex + axis * h + (u * Math.Cos(a) + v * Math.Sin(a)) * rr + Noise(Sigma));
                }
                var f = Fitter.FitCone(pts.ToArray());
                Check(f != null, "no fit");
                Check(AxisErrDeg(f.Axis, axis) < 0.5, "axis err {0} deg", AxisErrDeg(f.Axis, axis));
                Check(f.Axis.Dot(axis) > 0, "cone must open along +axis");
                Check(Math.Abs(f.HalfAngle - half) * 180 / Math.PI < 0.3, "half-angle err {0} deg", (f.HalfAngle - half) * 180 / Math.PI);
                Check((f.Apex - apex).Length < 0.5 || half < 5 * Math.PI / 180, "apex err {0}", (f.Apex - apex).Length);
            }
        }

        static void TestCone() { ConeTest(15, 60); }
        static void TestConeShallow() { ConeTest(3, 15); }

        static void TestConstructions()
        {
            var p1 = new PlanePrim { Origin = new Vec3(0, 0, 0), Normal = Vec3.UnitZ };
            var p2 = new PlanePrim { Origin = new Vec3(0, 0, 25), Normal = Vec3.UnitZ };
            var d = Constructions.Distance(p1, p2);
            Check(Math.Abs(d.Value - 25) < 1e-9, "plane-plane");
            var p3 = new PlanePrim { Origin = new Vec3(10, 0, 0), Normal = Vec3.UnitX };
            var line = Constructions.Intersect(p1, p3);
            Check(Math.Abs(Math.Abs(line.Direction.Y) - 1) < 1e-9, "intersection direction");
            Check(Math.Abs(line.Origin.X - 10) < 1e-9 && Math.Abs(line.Origin.Z) < 1e-9, "intersection origin");
            var ang = Constructions.Angle(p1, p3);
            Check(Math.Abs(ang.Value - 90) < 1e-9, "angle");
            // nearly parallel axes: distance must be measured at the features (35 mm), not at the skew closest points
            var c1 = new CylinderPrim { AxisPoint = new Vec3(0, 0, 10), Axis = Vec3.UnitZ, Radius = 5 };
            var c2 = new CylinderPrim { AxisPoint = new Vec3(0, 35, 10), Axis = new Vec3(0.0012, 0.0005, 1).Normalized(), Radius = 6 };
            var dd = Constructions.Distance(c1, c2);
            Check(Math.Abs(dd.Value - 35) < 0.01, "parallel axes distance {0}", dd.Value);
            var cyl = new CylinderPrim { AxisPoint = new Vec3(5, 5, 0), Axis = Vec3.UnitZ, Radius = 3 };
            var pt = Constructions.Intersect(cyl, p2) as PointPrim;
            Check(pt != null && (pt.Position - new Vec3(5, 5, 25)).Length < 1e-9, "axis-plane");
        }

        static void TestAlignment()
        {
            // Part frame: rotated 30° about Z and translated.
            double a = 30 * Math.PI / 180;
            Vec3 x = new Vec3(Math.Cos(a), Math.Sin(a), 0), y = new Vec3(-Math.Sin(a), Math.Cos(a), 0), z = Vec3.UnitZ;
            Vec3 o = new Vec3(100, 50, 10);
            var top = new PlanePrim { Origin = o + x * 20 + y * 20, Normal = z };
            var edge = new LinePrim { Origin = o + x * 40, Direction = x };
            var corner = new PointPrim { Position = o };
            Mat4 m = Constructions.Alignment(top, edge, corner);
            Vec3 q = m.TransformPoint(o + x * 7 + y * 3 + z * 2);
            Check((q - new Vec3(7, 3, 2)).Length < 1e-9, "aligned point {0}", q);
        }
    }
}
