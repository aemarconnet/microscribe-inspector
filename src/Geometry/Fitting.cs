using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    public enum FeatureKind { Point, Line, Plane, Circle, Sphere, Cylinder, Cone, Curve, Surface }

    /// <summary>Residual statistics of a fit (signed residuals, mm).</summary>
    public sealed class FitStats
    {
        public int N;
        public double Rms, Min, Max, MeanAbs;
        public double Form { get { return Max - Min; } }
        public double MaxAbs { get { return Math.Max(Math.Abs(Min), Math.Abs(Max)); } }
        public double[] Residuals;

        public static FitStats From(double[] res)
        {
            var s = new FitStats { N = res.Length, Residuals = res };
            if (res.Length == 0) return s;
            double ss = 0, sa = 0;
            s.Min = double.MaxValue; s.Max = double.MinValue;
            foreach (double r in res)
            {
                ss += r * r; sa += Math.Abs(r);
                if (r < s.Min) s.Min = r;
                if (r > s.Max) s.Max = r;
            }
            s.Rms = Math.Sqrt(ss / res.Length);
            s.MeanAbs = sa / res.Length;
            return s;
        }
    }

    /// <summary>Base of all fitted primitives.</summary>
    public abstract class Primitive
    {
        public FitStats Stats;
        public abstract FeatureKind Kind { get; }
        /// <summary>Signed distance from p to the primitive (outside / along normal = positive).</summary>
        public abstract double SignedDistance(Vec3 p);
        public double[] Residuals(IList<Vec3> pts)
        {
            var r = new double[pts.Count];
            for (int i = 0; i < r.Length; i++) r[i] = SignedDistance(pts[i]);
            return r;
        }
        public abstract Primitive Clone();
    }

    public sealed class PointPrim : Primitive
    {
        public Vec3 Position;
        public override FeatureKind Kind { get { return FeatureKind.Point; } }
        public override double SignedDistance(Vec3 p) { return (p - Position).Length; }
        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    public sealed class LinePrim : Primitive
    {
        public Vec3 Origin, Direction; // Origin = centroid
        public double TMin, TMax;       // extent of measured points along Direction
        public override FeatureKind Kind { get { return FeatureKind.Line; } }
        public Vec3 Start { get { return Origin + Direction * TMin; } }
        public Vec3 End { get { return Origin + Direction * TMax; } }
        public override double SignedDistance(Vec3 p)
        {
            Vec3 w = p - Origin;
            return (w - Direction * w.Dot(Direction)).Length;
        }
        public Vec3 Project(Vec3 p) { return Origin + Direction * (p - Origin).Dot(Direction); }
        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    public sealed class PlanePrim : Primitive
    {
        public Vec3 Origin, Normal;
        public override FeatureKind Kind { get { return FeatureKind.Plane; } }
        public override double SignedDistance(Vec3 p) { return (p - Origin).Dot(Normal); }
        public Vec3 Project(Vec3 p) { return p - Normal * SignedDistance(p); }
        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    public sealed class CirclePrim : Primitive
    {
        public Vec3 Center, Normal;
        public double Radius;
        public override FeatureKind Kind { get { return FeatureKind.Circle; } }
        /// <summary>Radial residual in the circle plane (roundness).</summary>
        public override double SignedDistance(Vec3 p)
        {
            Vec3 w = p - Center;
            Vec3 inPlane = w - Normal * w.Dot(Normal);
            return inPlane.Length - Radius;
        }
        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    public sealed class SpherePrim : Primitive
    {
        public Vec3 Center;
        public double Radius;
        public override FeatureKind Kind { get { return FeatureKind.Sphere; } }
        public override double SignedDistance(Vec3 p) { return (p - Center).Length - Radius; }
        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    public sealed class CylinderPrim : Primitive
    {
        public Vec3 AxisPoint, Axis;
        public double Radius;
        public double HMin, HMax; // measured extent along the axis, relative to AxisPoint
        public override FeatureKind Kind { get { return FeatureKind.Cylinder; } }
        public double RadialDistance(Vec3 p)
        {
            Vec3 w = p - AxisPoint;
            return (w - Axis * w.Dot(Axis)).Length;
        }
        public override double SignedDistance(Vec3 p) { return RadialDistance(p) - Radius; }
        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    /// <summary>Cone: radius r(h) = RadiusAtOrigin + h * tan(HalfAngle), h measured along Axis from AxisPoint.</summary>
    public sealed class ConePrim : Primitive
    {
        public Vec3 AxisPoint, Axis;
        public double RadiusAtOrigin, HalfAngle;
        public double HMin, HMax;
        public override FeatureKind Kind { get { return FeatureKind.Cone; } }
        public double RadiusAt(double h) { return RadiusAtOrigin + h * Math.Tan(HalfAngle); }
        /// <summary>Apex position (undefined when HalfAngle ≈ 0).</summary>
        public Vec3 Apex { get { return AxisPoint - Axis * (RadiusAtOrigin / Math.Tan(HalfAngle)); } }
        public override double SignedDistance(Vec3 p)
        {
            Vec3 w = p - AxisPoint;
            double h = w.Dot(Axis);
            double rho = (w - Axis * h).Length;
            return (rho - RadiusAt(h)) * Math.Cos(HalfAngle);
        }
        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    /// <summary>Least-squares fitting of geometric primitives to probed points.</summary>
    public static class Fitter
    {
        public static int MinPoints(FeatureKind k)
        {
            switch (k)
            {
                case FeatureKind.Point: return 1;
                case FeatureKind.Line: return 2;
                case FeatureKind.Plane: return 3;
                case FeatureKind.Circle: return 3;
                case FeatureKind.Sphere: return 4;
                case FeatureKind.Cylinder: return 5;
                case FeatureKind.Cone: return 6;
                case FeatureKind.Curve: return 2;
                case FeatureKind.Surface: return 4;
            }
            return 1;
        }

        public static Primitive Fit(FeatureKind kind, Vec3[] pts, Vec3[] stylusDirs = null, PlanePrim projectionPlane = null)
        {
            if (pts.Length < MinPoints(kind)) return null;
            switch (kind)
            {
                case FeatureKind.Point: return FitPoint(pts);
                case FeatureKind.Line: return FitLine(pts);
                case FeatureKind.Plane: return FitPlane(pts);
                case FeatureKind.Circle: return FitCircle(pts, projectionPlane);
                case FeatureKind.Sphere: return FitSphere(pts);
                case FeatureKind.Cylinder: return FitCylinder(pts, stylusDirs);
                case FeatureKind.Cone: return FitCone(pts, stylusDirs);
            }
            return null;
        }

        // ------------------------------------------------------------------ point / line / plane

        public static PointPrim FitPoint(Vec3[] pts)
        {
            Vec3 c = Vec3.Zero;
            foreach (var p in pts) c += p;
            var r = new PointPrim { Position = c / pts.Length };
            r.Stats = FitStats.From(r.Residuals(pts));
            return r;
        }

        public static LinePrim FitLine(Vec3[] pts)
        {
            double[,] s;
            Vec3 c = LinAlg.Scatter(pts, out s);
            double[] ev; Vec3[] vec;
            LinAlg.SymmetricEigen3(s, out ev, out vec);
            Vec3 d = vec[2];
            // orient along capture order
            if ((pts[pts.Length - 1] - pts[0]).Dot(d) < 0) d = -d;
            var r = new LinePrim { Origin = c, Direction = d };
            r.TMin = double.MaxValue; r.TMax = double.MinValue;
            foreach (var p in pts)
            {
                double t = (p - c).Dot(d);
                r.TMin = Math.Min(r.TMin, t); r.TMax = Math.Max(r.TMax, t);
            }
            r.Stats = FitStats.From(r.Residuals(pts));
            return r;
        }

        public static PlanePrim FitPlane(Vec3[] pts, Vec3[] stylusDirs = null)
        {
            double[,] s;
            Vec3 c = LinAlg.Scatter(pts, out s);
            double[] ev; Vec3[] vec;
            LinAlg.SymmetricEigen3(s, out ev, out vec);
            Vec3 n = vec[0];
            // Orient the normal toward the probe (against the stylus direction) when known.
            if (stylusDirs != null && stylusDirs.Length > 0)
            {
                Vec3 avg = Vec3.Zero;
                foreach (var d in stylusDirs) avg += d;
                if (avg.Dot(n) > 0) n = -n;
            }
            else if (n.Z < 0 && Math.Abs(n.Z) > 0.5) n = -n;
            var r = new PlanePrim { Origin = c, Normal = n };
            r.Stats = FitStats.From(r.Residuals(pts));
            return r;
        }

        // ------------------------------------------------------------------ circle

        /// <summary>2D algebraic (Kasa) circle fit followed by geometric refinement.</summary>
        public static bool FitCircle2D(double[] x, double[] y, out double cx, out double cy, out double r)
        {
            int n = x.Length;
            double mx = x.Average(), my = y.Average();
            var rows = new double[n][];
            var rhs = new double[n];
            for (int i = 0; i < n; i++)
            {
                double u = x[i] - mx, v = y[i] - my;
                rows[i] = new[] { u, v, 1.0 };
                rhs[i] = u * u + v * v;
            }
            double[] sol;
            if (!LinAlg.LeastSquares(rows, rhs, out sol)) { cx = cy = r = double.NaN; return false; }
            double a = sol[0] / 2, b = sol[1] / 2;
            double rr = sol[2] + a * a + b * b;
            cx = a + mx; cy = b + my; r = Math.Sqrt(Math.Max(rr, 0));
            // geometric refinement
            double[] p0 = { cx, cy, r };
            var lm = new LevenbergMarquardt { MaxIterations = 100 };
            double[] p = lm.Minimize((q, res) =>
            {
                for (int i = 0; i < n; i++)
                {
                    double dx = x[i] - q[0], dy = y[i] - q[1];
                    res[i] = Math.Sqrt(dx * dx + dy * dy) - q[2];
                }
            }, p0, n);
            cx = p[0]; cy = p[1]; r = Math.Abs(p[2]);
            return !double.IsNaN(r);
        }

        public static CirclePrim FitCircle(Vec3[] pts, PlanePrim projectionPlane = null)
        {
            Vec3 normal, origin;
            if (projectionPlane != null) { normal = projectionPlane.Normal; origin = projectionPlane.Origin; }
            else
            {
                var pl = FitPlane(pts);
                normal = pl.Normal; origin = pl.Origin;
            }
            Vec3 u = normal.AnyPerpendicular(), v = normal.Cross(u);
            int n = pts.Length;
            var xs = new double[n]; var ys = new double[n];
            for (int i = 0; i < n; i++) { Vec3 w = pts[i] - origin; xs[i] = w.Dot(u); ys[i] = w.Dot(v); }
            double cx, cy, r;
            if (!FitCircle2D(xs, ys, out cx, out cy, out r)) return null;
            var c = new CirclePrim { Center = origin + u * cx + v * cy, Normal = normal, Radius = r };
            c.Stats = FitStats.From(c.Residuals(pts));
            return c;
        }

        // ------------------------------------------------------------------ sphere

        public static SpherePrim FitSphere(Vec3[] pts)
        {
            int n = pts.Length;
            Vec3 m = Vec3.Zero;
            foreach (var pt in pts) m += pt;
            m /= n;
            var rows = new double[n][];
            var rhs = new double[n];
            for (int i = 0; i < n; i++)
            {
                Vec3 q = pts[i] - m;
                rows[i] = new[] { 2 * q.X, 2 * q.Y, 2 * q.Z, 1.0 };
                rhs[i] = q.LengthSquared;
            }
            double[] sol;
            Vec3 c0; double r0;
            if (LinAlg.LeastSquares(rows, rhs, out sol))
            {
                c0 = m + new Vec3(sol[0], sol[1], sol[2]);
                r0 = Math.Sqrt(Math.Max(1e-12, sol[3] + sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2]));
            }
            else { c0 = m; r0 = 1; }
            var lm = new LevenbergMarquardt();
            double[] p = lm.Minimize((q, res) =>
            {
                var c = new Vec3(q[0], q[1], q[2]);
                for (int i = 0; i < n; i++) res[i] = (pts[i] - c).Length - q[3];
            }, new[] { c0.X, c0.Y, c0.Z, r0 }, n);
            var s = new SpherePrim { Center = new Vec3(p[0], p[1], p[2]), Radius = Math.Abs(p[3]) };
            s.Stats = FitStats.From(s.Residuals(pts));
            return s;
        }

        // ------------------------------------------------------------------ cylinder

        /// <summary>Unit directions spread over a hemisphere (Fibonacci lattice).</summary>
        public static Vec3[] HemisphereDirections(int count)
        {
            var dirs = new Vec3[count];
            double golden = Math.PI * (3 - Math.Sqrt(5));
            for (int i = 0; i < count; i++)
            {
                double z = 1 - (i + 0.5) / count; // (0,1]
                double rad = Math.Sqrt(1 - z * z);
                double th = golden * i;
                dirs[i] = new Vec3(Math.Cos(th) * rad, Math.Sin(th) * rad, z);
            }
            return dirs;
        }

        sealed class AxisCandidate { public Vec3 Dir; public double Score; }

        /// <summary>Scores axis directions by how well points projected along them form a circle
        /// (optionally with radius varying linearly with height = cone).</summary>
        static List<Vec3> AxisCandidates(Vec3[] pts, Vec3[] stylusDirs, bool cone, int keep)
        {
            var cands = new List<Vec3>();
            double[,] s;
            Vec3 c = LinAlg.Scatter(pts, out s);
            double[] ev; Vec3[] vec;
            LinAlg.SymmetricEigen3(s, out ev, out vec);
            cands.AddRange(vec);
            if (stylusDirs != null && stylusDirs.Length >= 3)
            {
                // The stylus is roughly perpendicular to the axis when probing a cylinder wall.
                double[,] ds = new double[3, 3];
                foreach (var d in stylusDirs)
                    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ds[i, j] += d[i] * d[j];
                double[] dev; Vec3[] dvec;
                LinAlg.SymmetricEigen3(ds, out dev, out dvec);
                cands.Add(dvec[0]);
            }
            cands.AddRange(HemisphereDirections(600));

            var scored = new List<AxisCandidate>();
            int n = pts.Length;
            var xs = new double[n]; var ys = new double[n]; var hs = new double[n];
            foreach (Vec3 d0 in cands)
            {
                Vec3 d = d0.Normalized();
                Vec3 u = d.AnyPerpendicular(), v = d.Cross(u);
                for (int i = 0; i < n; i++) { Vec3 w = pts[i] - c; xs[i] = w.Dot(u); ys[i] = w.Dot(v); hs[i] = w.Dot(d); }
                double score = CircleScore(xs, ys, hs, cone);
                if (!double.IsNaN(score)) scored.Add(new AxisCandidate { Dir = d, Score = score });
            }
            scored.Sort((a, b) => a.Score.CompareTo(b.Score));
            var result = new List<Vec3>();
            foreach (var sc in scored)
            {
                bool dup = false;
                foreach (var r in result) if (Math.Abs(r.Dot(sc.Dir)) > 0.995) { dup = true; break; }
                if (!dup) result.Add(sc.Dir);
                if (result.Count >= keep) break;
            }
            return result;
        }

        /// <summary>Fast algebraic circle fit RMS (no refinement), normalised by radius.</summary>
        static double CircleScore(double[] x, double[] y, double[] h, bool cone)
        {
            int n = x.Length;
            double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sz = 0, sxz = 0, syz = 0;
            for (int i = 0; i < n; i++)
            {
                double z = x[i] * x[i] + y[i] * y[i];
                sx += x[i]; sy += y[i]; sz += z;
                sxx += x[i] * x[i]; syy += y[i] * y[i]; sxy += x[i] * y[i];
                sxz += x[i] * z; syz += y[i] * z;
            }
            var A = new double[3, 3] { { sxx, sxy, sx }, { sxy, syy, sy }, { sx, sy, n } };
            double[] sol;
            if (!LinAlg.Solve(A, new[] { sxz, syz, sz }, out sol)) return double.NaN;
            double a = sol[0] / 2, b = sol[1] / 2;
            double r2 = sol[2] + a * a + b * b;
            if (r2 <= 0) return double.NaN;
            double r = Math.Sqrt(r2);
            var rho = new double[n];
            for (int i = 0; i < n; i++) rho[i] = Math.Sqrt((x[i] - a) * (x[i] - a) + (y[i] - b) * (y[i] - b));
            double ss = 0;
            if (!cone)
            {
                for (int i = 0; i < n; i++) ss += (rho[i] - r) * (rho[i] - r);
            }
            else
            {
                // rho = r0 + k h  (linear regression)
                double mh = h.Average(), mr = rho.Average(), shh = 0, shr = 0;
                for (int i = 0; i < n; i++) { shh += (h[i] - mh) * (h[i] - mh); shr += (h[i] - mh) * (rho[i] - mr); }
                double k = shh > 1e-12 ? shr / shh : 0;
                for (int i = 0; i < n; i++) { double e = rho[i] - (mr + k * (h[i] - mh)); ss += e * e; }
            }
            return Math.Sqrt(ss / n) / Math.Max(r, 1e-9);
        }

        public static CylinderPrim FitCylinder(Vec3[] pts, Vec3[] stylusDirs = null)
        {
            CylinderPrim best = null;
            foreach (Vec3 axis0 in AxisCandidates(pts, stylusDirs, false, 4))
            {
                var c = RefineCylinder(pts, axis0);
                if (c == null) continue;
                // second pass re-centred on the refined axis for conditioning
                c = RefineCylinder(pts, c.Axis, c.AxisPoint) ?? c;
                if (best == null || c.Stats.Rms < best.Stats.Rms) best = c;
            }
            return best;
        }

        static CylinderPrim RefineCylinder(Vec3[] pts, Vec3 axis0, Vec3? axisPoint0 = null)
        {
            int n = pts.Length;
            Vec3 cen = Vec3.Zero;
            foreach (var p in pts) cen += p;
            cen /= n;
            Vec3 e1 = axis0.AnyPerpendicular(), e2 = axis0.Cross(e1);
            Vec3 basePt;
            double r0;
            if (axisPoint0.HasValue) basePt = axisPoint0.Value;
            else
            {
                var xs = new double[n]; var ys = new double[n];
                for (int i = 0; i < n; i++) { Vec3 w = pts[i] - cen; xs[i] = w.Dot(e1); ys[i] = w.Dot(e2); }
                double cx, cy, rr;
                if (!FitCircle2D(xs, ys, out cx, out cy, out rr)) return null;
                basePt = cen + e1 * cx + e2 * cy;
            }
            // project basePt to the plane through the centroid perpendicular to axis0
            basePt = basePt - axis0 * (basePt - cen).Dot(axis0);
            {
                double sum = 0;
                foreach (var p in pts) { Vec3 w = p - basePt; sum += (w - axis0 * w.Dot(axis0)).Length; }
                r0 = sum / n;
            }
            double scale = Math.Max(r0, 1e-3);
            LevenbergMarquardt.ResidualFn f = (q, res) =>
            {
                Vec3 a = (axis0 + e1 * q[0] + e2 * q[1]).Normalized();
                Vec3 o = basePt + e1 * q[2] + e2 * q[3];
                for (int i = 0; i < n; i++)
                {
                    Vec3 w = pts[i] - o;
                    res[i] = (w - a * w.Dot(a)).Length - q[4];
                }
            };
            var lm = new LevenbergMarquardt();
            double[] p0 = { 0, 0, 0, 0, r0 };
            double[] step = { 1e-7, 1e-7, 1e-7 * scale, 1e-7 * scale, 1e-7 * scale };
            double[] sol = lm.Minimize(f, p0, n, step);
            Vec3 axis = (axis0 + e1 * sol[0] + e2 * sol[1]).Normalized();
            Vec3 org = basePt + e1 * sol[2] + e2 * sol[3];
            // move the axis point to the foot of the centroid
            org = org + axis * (cen - org).Dot(axis);
            var cyl = new CylinderPrim { Axis = axis, AxisPoint = org, Radius = Math.Abs(sol[4]) };
            cyl.HMin = double.MaxValue; cyl.HMax = double.MinValue;
            foreach (var p in pts)
            {
                double h = (p - org).Dot(axis);
                cyl.HMin = Math.Min(cyl.HMin, h); cyl.HMax = Math.Max(cyl.HMax, h);
            }
            cyl.Stats = FitStats.From(cyl.Residuals(pts));
            if (double.IsNaN(cyl.Stats.Rms)) return null;
            return cyl;
        }

        // ------------------------------------------------------------------ cone

        public static ConePrim FitCone(Vec3[] pts, Vec3[] stylusDirs = null)
        {
            ConePrim best = null;
            var axes = AxisCandidates(pts, stylusDirs, true, 4);
            foreach (Vec3 axis0 in axes)
            {
                var c = RefineCone(pts, axis0, null);
                if (c == null) continue;
                c = RefineCone(pts, c.Axis, c) ?? c;
                if (best == null || c.Stats.Rms < best.Stats.Rms) best = c;
            }
            if (best != null)
            {
                // Orient so that the cone opens along +Axis (radius grows with h).
                if (best.HalfAngle < 0)
                {
                    best.Axis = -best.Axis;
                    best.HalfAngle = -best.HalfAngle;
                    double hmin = -best.HMax, hmax = -best.HMin;
                    best.HMin = hmin; best.HMax = hmax;
                }
            }
            return best;
        }

        static ConePrim RefineCone(Vec3[] pts, Vec3 axis0, ConePrim start)
        {
            int n = pts.Length;
            Vec3 cen = Vec3.Zero;
            foreach (var p in pts) cen += p;
            cen /= n;
            Vec3 e1 = axis0.AnyPerpendicular(), e2 = axis0.Cross(e1);
            Vec3 basePt;
            double r0, k0;
            var hs = new double[n];
            if (start != null)
            {
                basePt = start.AxisPoint - axis0 * (start.AxisPoint - cen).Dot(axis0);
                double h0 = (basePt - start.AxisPoint).Dot(start.Axis);
                r0 = start.RadiusAt(h0);
                k0 = Math.Tan(start.HalfAngle) * Math.Sign(start.Axis.Dot(axis0));
            }
            else
            {
                var xs = new double[n]; var ys = new double[n];
                for (int i = 0; i < n; i++) { Vec3 w = pts[i] - cen; xs[i] = w.Dot(e1); ys[i] = w.Dot(e2); }
                double cx, cy, rr;
                if (!FitCircle2D(xs, ys, out cx, out cy, out rr)) return null;
                basePt = cen + e1 * cx + e2 * cy;
                // linear regression of radius vs height
                var rho = new double[n];
                for (int i = 0; i < n; i++)
                {
                    Vec3 w = pts[i] - basePt;
                    hs[i] = w.Dot(axis0);
                    rho[i] = (w - axis0 * hs[i]).Length;
                }
                double mh = hs.Average(), mr = rho.Average(), shh = 0, shr = 0;
                for (int i = 0; i < n; i++) { shh += (hs[i] - mh) * (hs[i] - mh); shr += (hs[i] - mh) * (rho[i] - mr); }
                k0 = shh > 1e-12 ? shr / shh : 0;
                r0 = mr - k0 * mh;
            }
            double scale = Math.Max(Math.Abs(r0), 1e-3);
            LevenbergMarquardt.ResidualFn f = (q, res) =>
            {
                Vec3 a = (axis0 + e1 * q[0] + e2 * q[1]).Normalized();
                Vec3 o = basePt + e1 * q[2] + e2 * q[3];
                double th = q[5];
                double ct = Math.Cos(th), tt = Math.Tan(th);
                for (int i = 0; i < n; i++)
                {
                    Vec3 w = pts[i] - o;
                    double h = w.Dot(a);
                    double rho = (w - a * h).Length;
                    res[i] = (rho - (q[4] + h * tt)) * ct;
                }
            };
            var lm = new LevenbergMarquardt();
            double[] p0 = { 0, 0, 0, 0, r0, Math.Atan(k0) };
            double[] step = { 1e-7, 1e-7, 1e-7 * scale, 1e-7 * scale, 1e-7 * scale, 1e-7 };
            double[] sol = lm.Minimize(f, p0, n, step);
            var cone = new ConePrim
            {
                Axis = (axis0 + e1 * sol[0] + e2 * sol[1]).Normalized(),
                AxisPoint = basePt + e1 * sol[2] + e2 * sol[3],
                RadiusAtOrigin = sol[4],
                HalfAngle = sol[5]
            };
            // Re-base the axis point at the centroid foot.
            double hc = (cen - cone.AxisPoint).Dot(cone.Axis);
            cone.RadiusAtOrigin = cone.RadiusAt(hc);
            cone.AxisPoint = cone.AxisPoint + cone.Axis * hc;
            cone.HMin = double.MaxValue; cone.HMax = double.MinValue;
            foreach (var p in pts)
            {
                double h = (p - cone.AxisPoint).Dot(cone.Axis);
                cone.HMin = Math.Min(cone.HMin, h); cone.HMax = Math.Max(cone.HMax, h);
            }
            cone.Stats = FitStats.From(cone.Residuals(pts));
            if (double.IsNaN(cone.Stats.Rms) || Math.Abs(cone.HalfAngle) > 1.55) return null;
            return cone;
        }
    }
}
