using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>
    /// A closed planar outline probed around its wall: a round-ended slot, a rectangle (square slot, pocket
    /// or tab) or an ellipse. Length is along <see cref="XDir"/>, width across it (overall sizes, mm).
    /// </summary>
    public sealed class ProfilePrim : Primitive
    {
        public FeatureKind Shape = FeatureKind.Slot;
        public Vec3 Center, Normal = Vec3.UnitZ, XDir = Vec3.UnitX;
        public double Length, Width;

        public override FeatureKind Kind { get { return Shape; } }
        public Vec3 YDir { get { return Normal.Cross(XDir).Normalized(); } }

        /// <summary>In-plane distance from the outline (positive outside), like a circle's roundness residual.</summary>
        public override double SignedDistance(Vec3 p)
        {
            Vec3 w = p - Center;
            return Sd2(Shape, w.Dot(XDir), w.Dot(YDir), Length, Width);
        }

        public static double Sd2(FeatureKind shape, double x, double y, double L, double W)
        {
            x = Math.Abs(x); y = Math.Abs(y);
            switch (shape)
            {
                case FeatureKind.Slot:
                    {
                        double r = W / 2, a = Math.Max(0, (L - W) / 2), dx = Math.Max(0, x - a);
                        return Math.Sqrt(dx * dx + y * y) - r;
                    }
                case FeatureKind.Rectangle:
                    {
                        double qx = x - L / 2, qy = y - W / 2;
                        double outside = Math.Sqrt(Math.Max(qx, 0) * Math.Max(qx, 0) + Math.Max(qy, 0) * Math.Max(qy, 0));
                        return outside + Math.Min(Math.Max(qx, qy), 0);
                    }
                default:
                    {
                        // ellipse: closest point by Newton on the parameter angle
                        double a = Math.Max(1e-9, L / 2), b = Math.Max(1e-9, W / 2);
                        double t = Math.Atan2(a * y, b * x);
                        for (int i = 0; i < 12; i++)
                        {
                            double c = Math.Cos(t), s = Math.Sin(t);
                            double f = (a * a - b * b) * c * s - x * a * s + y * b * c;
                            double df = (a * a - b * b) * (c * c - s * s) - x * a * c - y * b * s;
                            if (Math.Abs(df) < 1e-15) break;
                            double nt = t - f / df;
                            nt = Math.Max(0, Math.Min(Math.PI / 2, nt));
                            if (Math.Abs(nt - t) < 1e-13) { t = nt; break; }
                            t = nt;
                        }
                        double ex = a * Math.Cos(t) - x, ey = b * Math.Sin(t) - y;
                        double d = Math.Sqrt(ex * ex + ey * ey);
                        return (x / a) * (x / a) + (y / b) * (y / b) < 1 ? -d : d;
                    }
            }
        }

        /// <summary>The outline as a closed polyline (machine frame).</summary>
        public List<Vec3> Outline(int perQuarter = 16)
        {
            var r = new List<Vec3>();
            Vec3 X = XDir, Y = YDir;
            Func<double, double, Vec3> P = (x, y) => Center + X * x + Y * y;
            if (Shape == FeatureKind.Rectangle)
            {
                double hx = Length / 2, hy = Width / 2;
                r.Add(P(-hx, -hy)); r.Add(P(hx, -hy)); r.Add(P(hx, hy)); r.Add(P(-hx, hy)); r.Add(P(-hx, -hy));
                return r;
            }
            if (Shape == FeatureKind.Slot)
            {
                double rr = Width / 2, a = Math.Max(0, (Length - Width) / 2);
                for (int i = 0; i <= 2 * perQuarter; i++) { double t = -Math.PI / 2 + Math.PI * i / (2 * perQuarter); r.Add(P(a + rr * Math.Cos(t), rr * Math.Sin(t))); }
                for (int i = 0; i <= 2 * perQuarter; i++) { double t = Math.PI / 2 + Math.PI * i / (2 * perQuarter); r.Add(P(-a + rr * Math.Cos(t), rr * Math.Sin(t))); }
                r.Add(r[0]);
                return r;
            }
            for (int i = 0; i <= 4 * perQuarter; i++) { double t = 2 * Math.PI * i / (4 * perQuarter); r.Add(P(Length / 2 * Math.Cos(t), Width / 2 * Math.Sin(t))); }
            return r;
        }

        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }
    }

    public static class ProfileFit
    {
        public static bool IsProfile(FeatureKind k) { return k == FeatureKind.Slot || k == FeatureKind.Rectangle || k == FeatureKind.Ellipse; }

        /// <summary>
        /// Fits a slot, rectangle or ellipse to points around its wall, in the points' best-fit plane (or a
        /// given projection plane). Start: centroid and principal axes; then least squares on the outline distance.
        /// </summary>
        public static ProfilePrim Fit(FeatureKind shape, Vec3[] pts, PlanePrim projectionPlane = null)
        {
            if (pts.Length < 6) return null;
            PlanePrim pl = projectionPlane ?? Fitter.FitPlane(pts);
            Vec3 n = pl.Normal.Normalized();
            Vec3 u = n.AnyPerpendicular(), v = n.Cross(u);
            var xs = pts.Select(p => (p - pl.Origin).Dot(u)).ToArray();
            var ys = pts.Select(p => (p - pl.Origin).Dot(v)).ToArray();
            int m = pts.Length;
            double mx = xs.Average(), my = ys.Average(), sxx = 0, sxy = 0, syy = 0;
            for (int i = 0; i < m; i++) { double x = xs[i] - mx, y = ys[i] - my; sxx += x * x; sxy += x * y; syy += y * y; }
            double th = 0.5 * Math.Atan2(2 * sxy, sxx - syy);
            double c0 = Math.Cos(th), s0 = Math.Sin(th);
            double lmin = double.MaxValue, lmax = double.MinValue, wmin = double.MaxValue, wmax = double.MinValue;
            for (int i = 0; i < m; i++)
            {
                double x = xs[i] - mx, y = ys[i] - my, a = x * c0 + y * s0, b = -x * s0 + y * c0;
                lmin = Math.Min(lmin, a); lmax = Math.Max(lmax, a); wmin = Math.Min(wmin, b); wmax = Math.Max(wmax, b);
            }
            double cx = mx + c0 * (lmin + lmax) / 2 - s0 * (wmin + wmax) / 2, cy = my + s0 * (lmin + lmax) / 2 + c0 * (wmin + wmax) / 2;
            var p0 = new[] { cx, cy, th, Math.Max(1e-3, lmax - lmin), Math.Max(1e-3, wmax - wmin) };
            LevenbergMarquardt.ResidualFn res = (p, r) =>
            {
                double c = Math.Cos(p[2]), s = Math.Sin(p[2]);
                for (int i = 0; i < m; i++)
                {
                    double x = xs[i] - p[0], y = ys[i] - p[1];
                    r[i] = ProfilePrim.Sd2(shape, x * c + y * s, -x * s + y * c, Math.Abs(p[3]), Math.Abs(p[4]));
                }
            };
            var best = new LevenbergMarquardt().Minimize(res, p0, m);
            double L = Math.Abs(best[3]), W = Math.Abs(best[4]), ang = best[2];
            if (W > L) { double t = L; L = W; W = t; ang += Math.PI / 2; }   // length is the long side
            Vec3 xd = (u * Math.Cos(ang) + v * Math.Sin(ang)).Normalized();
            var prim = new ProfilePrim { Shape = shape, Center = pl.Origin + u * best[0] + v * best[1], Normal = n, XDir = xd, Length = L, Width = W };
            prim.Stats = FitStats.From(prim.Residuals(pts));
            return prim;
        }
    }
}
