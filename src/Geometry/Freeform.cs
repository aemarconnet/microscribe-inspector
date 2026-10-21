using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    public enum CurveMode { Spline, Linear }

    /// <summary>A traced curve: polyline (degree 1) or B-spline.</summary>
    public sealed class CurvePrim : Primitive
    {
        public int Degree = 1;
        public Vec3[] Ctrl = new Vec3[0];
        public double[] Knots = new double[0];
        public bool Closed;
        /// <summary>Dense polyline used for display and distances.</summary>
        public Vec3[] Samples = new Vec3[0];

        public override FeatureKind Kind { get { return FeatureKind.Curve; } }

        public override double SignedDistance(Vec3 p)
        {
            double best = double.MaxValue;
            for (int i = 0; i + 1 < Samples.Length; i++)
                best = Math.Min(best, SegmentDistance(p, Samples[i], Samples[i + 1]));
            return Samples.Length == 1 ? (p - Samples[0]).Length : best;
        }

        public static double SegmentDistance(Vec3 p, Vec3 a, Vec3 b)
        {
            Vec3 d = b - a;
            double l2 = d.LengthSquared;
            double t = l2 > 1e-24 ? Math.Max(0, Math.Min(1, (p - a).Dot(d) / l2)) : 0;
            return (p - (a + d * t)).Length;
        }

        public double Length
        {
            get { double s = 0; for (int i = 1; i < Samples.Length; i++) s += (Samples[i] - Samples[i - 1]).Length; return s; }
        }

        public override Primitive Clone() { return (Primitive)MemberwiseClone(); }

        public CurvePrim Transformed(Mat4 m)
        {
            var c = (CurvePrim)Clone();
            c.Ctrl = Ctrl.Select(m.TransformPoint).ToArray();
            c.Samples = Samples.Select(m.TransformPoint).ToArray();
            return c;
        }

        /// <summary>Builds a curve from probed points.</summary>
        public static CurvePrim Build(IList<Vec3> raw, CurveMode mode, double tolerance, bool closed)
        {
            var pts = new List<Vec3>();
            foreach (var p in raw) if (pts.Count == 0 || (p - pts[pts.Count - 1]).Length > 1e-4) pts.Add(p);
            if (closed && pts.Count > 2 && (pts[0] - pts[pts.Count - 1]).Length > 1e-4) pts.Add(pts[0]);
            var c = new CurvePrim { Closed = closed };
            if (pts.Count < 2 || mode == CurveMode.Linear || pts.Count < 4)
            {
                c.Degree = 1;
                c.Ctrl = pts.ToArray();
                c.Knots = Spline.ClampedUniform(pts.Count, 1, ChordParams(pts));
                c.Samples = pts.ToArray();
            }
            else
            {
                Vec3[] ctrl; double[] knots;
                if (tolerance <= 0) Spline.Interpolate(pts, 3, out ctrl, out knots);
                else Spline.Approximate(pts, 3, tolerance, out ctrl, out knots);
                c.Degree = Math.Min(3, ctrl.Length - 1);
                c.Ctrl = ctrl; c.Knots = knots;
                int ns = Math.Max(32, Math.Min(2000, pts.Count * 4));
                c.Samples = new Vec3[ns + 1];
                for (int i = 0; i <= ns; i++) c.Samples[i] = Spline.Evaluate(c.Degree, ctrl, knots, (double)i / ns);
            }
            c.Stats = FitStats.From(raw.Select(p => c.SignedDistance(p)).ToArray());
            return c;
        }

        public static double[] ChordParams(IList<Vec3> q)
        {
            int n = q.Count;
            var u = new double[n];
            double total = 0;
            for (int k = 1; k < n; k++) total += (q[k] - q[k - 1]).Length;
            if (total < 1e-12) total = 1;
            for (int k = 1; k < n; k++) u[k] = u[k - 1] + (q[k] - q[k - 1]).Length / total;
            if (n > 0) u[n - 1] = 1;
            return u;
        }
    }

    /// <summary>Tensor-product B-spline surface (non-rational).</summary>
    public sealed class SurfacePrim : Primitive
    {
        public int DegU, DegV;
        public double[] KnotsU, KnotsV;
        /// <summary>Control net [i (u), j (v)].</summary>
        public Vec3[,] Ctrl;
        public int SectionCount;

        Vec3[,] _grid;  // cached evaluation for display and distance queries
        Vec3[,] _gridN;
        public const int GridN = 48;

        public override FeatureKind Kind { get { return FeatureKind.Surface; } }

        public Vec3 Evaluate(double u, double v)
        {
            int nu = Ctrl.GetLength(0) - 1, nv = Ctrl.GetLength(1) - 1;
            int su = Spline.FindSpan(nu, DegU, u, KnotsU), sv = Spline.FindSpan(nv, DegV, v, KnotsV);
            var Nu = Spline.BasisFuns(su, u, DegU, KnotsU);
            var Nv = Spline.BasisFuns(sv, v, DegV, KnotsV);
            Vec3 s = Vec3.Zero;
            for (int a = 0; a <= DegU; a++)
                for (int b = 0; b <= DegV; b++)
                    s += Ctrl[su - DegU + a, sv - DegV + b] * (Nu[a] * Nv[b]);
            return s;
        }

        public Vec3 Normal(double u, double v)
        {
            const double h = 1e-4;
            double u0 = Math.Max(0, u - h), u1 = Math.Min(1, u + h), v0 = Math.Max(0, v - h), v1 = Math.Min(1, v + h);
            Vec3 du = Evaluate(u1, v) - Evaluate(u0, v), dv = Evaluate(u, v1) - Evaluate(u, v0);
            return du.Cross(dv).Normalized();
        }

        public Vec3[,] Grid
        {
            get
            {
                if (_grid == null)
                {
                    _grid = new Vec3[GridN + 1, GridN + 1];
                    _gridN = new Vec3[GridN + 1, GridN + 1];
                    for (int i = 0; i <= GridN; i++)
                        for (int j = 0; j <= GridN; j++)
                        {
                            double u = (double)i / GridN, v = (double)j / GridN;
                            _grid[i, j] = Evaluate(u, v);
                            _gridN[i, j] = Normal(u, v);
                        }
                }
                return _grid;
            }
        }

        public Vec3[,] GridNormals { get { var g = Grid; return _gridN; } }

        /// <summary>Closest point parameters (grid search + local refinement).</summary>
        public void Closest(Vec3 p, out double u, out double v, out Vec3 q)
        {
            var g = Grid;
            int bi = 0, bj = 0; double best = double.MaxValue;
            for (int i = 0; i <= GridN; i++)
                for (int j = 0; j <= GridN; j++)
                {
                    double d = (g[i, j] - p).LengthSquared;
                    if (d < best) { best = d; bi = i; bj = j; }
                }
            u = (double)bi / GridN; v = (double)bj / GridN;
            double step = 1.0 / GridN;
            q = g[bi, bj];
            for (int it = 0; it < 24; it++)
            {
                bool moved = false;
                for (int k = 0; k < 8; k++)
                {
                    double du = new[] { 1, -1, 0, 0, 1, 1, -1, -1 }[k] * step, dv = new[] { 0, 0, 1, -1, 1, -1, 1, -1 }[k] * step;
                    double uu = Math.Max(0, Math.Min(1, u + du)), vv = Math.Max(0, Math.Min(1, v + dv));
                    Vec3 c = Evaluate(uu, vv);
                    double d = (c - p).LengthSquared;
                    if (d < best) { best = d; u = uu; v = vv; q = c; moved = true; }
                }
                if (!moved) step *= 0.5;
                if (step < 1e-6) break;
            }
        }

        public override double SignedDistance(Vec3 p)
        {
            double u, v; Vec3 q;
            Closest(p, out u, out v, out q);
            Vec3 n = Normal(u, v);
            double d = (p - q).Length;
            return (p - q).Dot(n) >= 0 ? d : -d;
        }

        public override Primitive Clone()
        {
            var s = (SurfacePrim)MemberwiseClone();
            s.Ctrl = (Vec3[,])Ctrl.Clone();
            s._grid = null; s._gridN = null;
            return s;
        }

        public SurfacePrim Transformed(Mat4 m)
        {
            var s = (SurfacePrim)Clone();
            for (int i = 0; i < Ctrl.GetLength(0); i++)
                for (int j = 0; j < Ctrl.GetLength(1); j++) s.Ctrl[i, j] = m.TransformPoint(Ctrl[i, j]);
            return s;
        }
    }

    /// <summary>Skinning (lofting) a B-spline surface through traced section curves.</summary>
    public static class Loft
    {
        /// <summary>
        /// sections: ordered curves (≥2, each ≥2 points). ctrlU: control points along the curves (0 = auto).
        /// Each section is resampled by arc length, approximated by a cubic with a common knot vector,
        /// then the control points are interpolated across sections (degree ≤ 3 in v).
        /// </summary>
        public static SurfacePrim Build(List<List<Vec3>> sections, int ctrlU, out string error)
        {
            error = null;
            var secs = sections.Where(s => s.Count >= 2).Select(s => s.ToList()).ToList();
            if (secs.Count < 2) { error = "Need at least 2 traced curves with 2+ points each"; return null; }
            // consistent direction: compare with the previous section
            for (int i = 1; i < secs.Count; i++)
            {
                var a = secs[i - 1]; var b = secs[i];
                double same = (b[0] - a[0]).Length + (b[b.Count - 1] - a[a.Count - 1]).Length;
                double flip = (b[0] - a[a.Count - 1]).Length + (b[b.Count - 1] - a[0]).Length;
                if (flip < same) b.Reverse();
            }
            int avg = (int)secs.Average(s => s.Count);
            int K = Math.Max(8, Math.Min(80, avg));
            int p = 3;
            int nc = ctrlU > 0 ? ctrlU : Math.Max(4, Math.Min(14, avg / 3));
            nc = Math.Max(p + 1, Math.Min(nc, K));
            double[] uk = new double[K];
            for (int k = 0; k < K; k++) uk[k] = (double)k / (K - 1);
            double[] U = Spline.ApproxKnots(uk, nc - 1, p);
            var rows = new List<Vec3[]>();
            foreach (var s in secs)
            {
                var res = Resample(s, K);
                Vec3[] ctrl;
                if (!Spline.FitLeastSquares(res, uk, p, nc, U, out ctrl)) { error = "Curve fit failed"; return null; }
                rows.Add(ctrl);
            }
            int m = rows.Count;
            int q = Math.Min(3, m - 1);
            // v parameters from average distance between consecutive control rows
            var vk = new double[m];
            double total = 0;
            var dsum = new double[m];
            for (int i = 1; i < m; i++)
            {
                double d = 0;
                for (int j = 0; j < nc; j++) d += (rows[i][j] - rows[i - 1][j]).Length;
                dsum[i] = d / nc; total += dsum[i];
            }
            if (total < 1e-9) { error = "Curves coincide"; return null; }
            for (int i = 1; i < m; i++) vk[i] = vk[i - 1] + dsum[i] / total;
            vk[m - 1] = 1;
            double[] V = Spline.InterpKnots(vk, q);
            var net = new Vec3[nc, m];
            for (int j = 0; j < nc; j++)
            {
                var col = rows.Select(r => r[j]).ToList();
                Vec3[] cp;
                if (!Spline.InterpolateWith(col, vk, q, V, out cp)) { error = "Interpolation failed"; return null; }
                for (int i = 0; i < m; i++) net[j, i] = cp[i];
            }
            var surf = new SurfacePrim { DegU = p, DegV = q, KnotsU = U, KnotsV = V, Ctrl = net, SectionCount = m };
            var all = sections.SelectMany(s => s).ToArray();
            surf.Stats = FitStats.From(all.Select(x => surf.SignedDistance(x)).ToArray());
            return surf;
        }

        /// <summary>Resamples a polyline to n points equally spaced in arc length.</summary>
        public static List<Vec3> Resample(List<Vec3> pts, int n)
        {
            var cum = new double[pts.Count];
            for (int i = 1; i < pts.Count; i++) cum[i] = cum[i - 1] + (pts[i] - pts[i - 1]).Length;
            double L = cum[pts.Count - 1];
            var r = new List<Vec3>();
            int seg = 0;
            for (int k = 0; k < n; k++)
            {
                double s = L * k / (n - 1);
                while (seg < pts.Count - 2 && cum[seg + 1] < s) seg++;
                double l = cum[seg + 1] - cum[seg];
                double t = l > 1e-12 ? (s - cum[seg]) / l : 0;
                r.Add(pts[seg] + (pts[seg + 1] - pts[seg]) * Math.Max(0, Math.Min(1, t)));
            }
            return r;
        }
    }
}
