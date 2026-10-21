using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>Rigid transforms of primitives and 2D helpers used for export extents.</summary>
    public static class GeoUtil
    {
        public static Primitive Transform(Primitive p, Mat4 m)
        {
            if (p == null) return null;
            var g = p.Clone();
            if (g is PointPrim) { var q = (PointPrim)g; q.Position = m.TransformPoint(q.Position); }
            else if (g is LinePrim) { var q = (LinePrim)g; q.Origin = m.TransformPoint(q.Origin); q.Direction = m.TransformVector(q.Direction); }
            else if (g is PlanePrim) { var q = (PlanePrim)g; q.Origin = m.TransformPoint(q.Origin); q.Normal = m.TransformVector(q.Normal); }
            else if (g is CirclePrim) { var q = (CirclePrim)g; q.Center = m.TransformPoint(q.Center); q.Normal = m.TransformVector(q.Normal); }
            else if (g is SpherePrim) { var q = (SpherePrim)g; q.Center = m.TransformPoint(q.Center); }
            else if (g is CylinderPrim) { var q = (CylinderPrim)g; q.AxisPoint = m.TransformPoint(q.AxisPoint); q.Axis = m.TransformVector(q.Axis); }
            else if (g is ConePrim) { var q = (ConePrim)g; q.AxisPoint = m.TransformPoint(q.AxisPoint); q.Axis = m.TransformVector(q.Axis); }
            else if (g is CurvePrim) { var t = ((CurvePrim)p).Transformed(m); t.Stats = p.Stats; return t; }
            else if (g is SurfacePrim) { var t = ((SurfacePrim)p).Transformed(m); t.Stats = p.Stats; return t; }
            return g;
        }

        /// <summary>2D convex hull (Andrew's monotone chain), CCW.</summary>
        public static List<double[]> ConvexHull(List<double[]> pts)
        {
            var p = new List<double[]>(pts);
            p.Sort((a, b) => a[0] != b[0] ? a[0].CompareTo(b[0]) : a[1].CompareTo(b[1]));
            if (p.Count < 3) return p;
            var h = new List<double[]>();
            Func<double[], double[], double[], double> cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
            foreach (var q in p)
            {
                while (h.Count >= 2 && cross(h[h.Count - 2], h[h.Count - 1], q) <= 0) h.RemoveAt(h.Count - 1);
                h.Add(q);
            }
            int lower = h.Count + 1;
            for (int i = p.Count - 2; i >= 0; i--)
            {
                var q = p[i];
                while (h.Count >= lower && cross(h[h.Count - 2], h[h.Count - 1], q) <= 0) h.RemoveAt(h.Count - 1);
                h.Add(q);
            }
            h.RemoveAt(h.Count - 1);
            return h;
        }

        /// <summary>
        /// Minimum-area enclosing rectangle of 2D points (rotating calipers over hull edges).
        /// Returns 4 corners CCW, expanded by margin on each side.
        /// </summary>
        public static double[][] MinAreaRectangle(List<double[]> pts, double margin)
        {
            var hull = ConvexHull(pts);
            if (hull.Count == 0) return null;
            if (hull.Count < 3)
            {
                // degenerate: build a thin rectangle around the segment/point
                double[] a = hull[0], b = hull[hull.Count - 1];
                double dx = b[0] - a[0], dy = b[1] - a[1], l = Math.Sqrt(dx * dx + dy * dy);
                if (l < 1e-9) { dx = 1; dy = 0; l = 1; }
                dx /= l; dy /= l;
                double m = Math.Max(margin, 1);
                return new[]
                {
                    new[] { a[0] - dx * m + dy * m, a[1] - dy * m - dx * m },
                    new[] { b[0] + dx * m + dy * m, b[1] + dy * m - dx * m },
                    new[] { b[0] + dx * m - dy * m, b[1] + dy * m + dx * m },
                    new[] { a[0] - dx * m - dy * m, a[1] - dy * m + dx * m },
                };
            }
            double best = double.MaxValue;
            double[][] rect = null;
            for (int i = 0; i < hull.Count; i++)
            {
                double[] p0 = hull[i], p1 = hull[(i + 1) % hull.Count];
                double ex = p1[0] - p0[0], ey = p1[1] - p0[1], el = Math.Sqrt(ex * ex + ey * ey);
                if (el < 1e-12) continue;
                ex /= el; ey /= el;
                double nx = -ey, ny = ex;
                double umin = double.MaxValue, umax = double.MinValue, vmin = double.MaxValue, vmax = double.MinValue;
                foreach (var q in hull)
                {
                    double u = q[0] * ex + q[1] * ey, v = q[0] * nx + q[1] * ny;
                    umin = Math.Min(umin, u); umax = Math.Max(umax, u); vmin = Math.Min(vmin, v); vmax = Math.Max(vmax, v);
                }
                double area = (umax - umin) * (vmax - vmin);
                if (area < best)
                {
                    best = area;
                    umin -= margin; umax += margin; vmin -= margin; vmax += margin;
                    Func<double, double, double[]> P = (u, v) => new[] { u * ex + v * nx, u * ey + v * ny };
                    rect = new[] { P(umin, vmin), P(umax, vmin), P(umax, vmax), P(umin, vmax) };
                }
            }
            return rect;
        }

        /// <summary>
        /// Angular coverage of angles (rad): returns [start, end] with end - start ≤ 2π covering all
        /// samples (complement of the largest gap). full=true when the largest gap is below fullGap.
        /// </summary>
        public static void AngularRange(List<double> angles, double fullGap, out double start, out double end, out bool full)
        {
            var a = new List<double>();
            foreach (double t in angles) { double v = t % (2 * Math.PI); if (v < 0) v += 2 * Math.PI; a.Add(v); }
            a.Sort();
            if (a.Count < 2) { start = a.Count > 0 ? a[0] : 0; end = start; full = false; return; }
            double maxGap = -1; int gi = 0;
            for (int i = 0; i < a.Count; i++)
            {
                double next = i + 1 < a.Count ? a[i + 1] : a[0] + 2 * Math.PI;
                double gap = next - a[i];
                if (gap > maxGap) { maxGap = gap; gi = i; }
            }
            full = maxGap < fullGap;
            start = gi + 1 < a.Count ? a[gi + 1] : a[0];
            end = a[gi] + (gi + 1 < a.Count ? 2 * Math.PI : 0);
            if (end < start) end += 2 * Math.PI;
            if (full) { start = 0; end = 2 * Math.PI; }
        }
    }

    /// <summary>Cubic B-spline curve interpolation (The NURBS Book, A9.1, chord-length parameters).</summary>
    public static class Spline
    {
        public static int FindSpan(int n, int p, double u, double[] U)
        {
            if (u >= U[n + 1]) return n;
            if (u <= U[p]) return p;
            int low = p, high = n + 1, mid = (low + high) / 2;
            while (u < U[mid] || u >= U[mid + 1])
            {
                if (u < U[mid]) high = mid; else low = mid;
                mid = (low + high) / 2;
            }
            return mid;
        }

        public static double[] BasisFuns(int i, double u, int p, double[] U)
        {
            var N = new double[p + 1];
            var left = new double[p + 1];
            var right = new double[p + 1];
            N[0] = 1;
            for (int j = 1; j <= p; j++)
            {
                left[j] = u - U[i + 1 - j];
                right[j] = U[i + j] - u;
                double saved = 0;
                for (int r = 0; r < j; r++)
                {
                    double den = right[r + 1] + left[j - r];
                    double temp = den != 0 ? N[r] / den : 0;
                    N[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }
                N[j] = saved;
            }
            return N;
        }

        /// <summary>Interpolates the points with a degree-p (≤3) B-spline. Returns control points and knot vector.</summary>
        public static void Interpolate(IList<Vec3> Q, int degree, out Vec3[] ctrl, out double[] knots)
        {
            int n = Q.Count - 1;
            int p = Math.Min(degree, n);
            var u = new double[n + 1];
            double total = 0;
            for (int k = 1; k <= n; k++) total += (Q[k] - Q[k - 1]).Length;
            if (total < 1e-12) total = 1;
            for (int k = 1; k <= n; k++) u[k] = u[k - 1] + (Q[k] - Q[k - 1]).Length / total;
            u[n] = 1;
            int m = n + p + 1;
            var U = new double[m + 1];
            for (int j = 0; j <= p; j++) { U[j] = 0; U[m - j] = 1; }
            for (int j = 1; j <= n - p; j++)
            {
                double s = 0;
                for (int i = j; i <= j + p - 1; i++) s += u[i];
                U[j + p] = s / p;
            }
            var A = new double[n + 1, n + 1];
            for (int k = 0; k <= n; k++)
            {
                int span = FindSpan(n, p, u[k], U);
                var N = BasisFuns(span, u[k], p, U);
                for (int j = 0; j <= p; j++) A[k, span - p + j] = N[j];
            }
            ctrl = new Vec3[n + 1];
            var bx = new double[n + 1]; var by = new double[n + 1]; var bz = new double[n + 1];
            for (int k = 0; k <= n; k++) { bx[k] = Q[k].X; by[k] = Q[k].Y; bz[k] = Q[k].Z; }
            double[] x, y, z;
            if (!LinAlg.Solve(A, bx, out x) || !LinAlg.Solve(A, by, out y) || !LinAlg.Solve(A, bz, out z))
            {
                ctrl = new Vec3[Q.Count]; for (int k = 0; k <= n; k++) ctrl[k] = Q[k];
                knots = U; return;
            }
            for (int k = 0; k <= n; k++) ctrl[k] = new Vec3(x[k], y[k], z[k]);
            knots = U;
        }

        /// <summary>Knots for interpolation with given parameters (averaging, NURBS Book eq. 9.8).</summary>
        public static double[] InterpKnots(double[] u, int p)
        {
            int n = u.Length - 1;
            int m = n + p + 1;
            var U = new double[m + 1];
            for (int j = 0; j <= p; j++) { U[j] = 0; U[m - j] = 1; }
            for (int j = 1; j <= n - p; j++)
            {
                double s = 0;
                for (int i = j; i <= j + p - 1; i++) s += u[i];
                U[j + p] = s / p;
            }
            return U;
        }

        /// <summary>Knot vector for a degree-1 polyline through the points (chord parameters).</summary>
        public static double[] ClampedUniform(int count, int p, double[] u)
        {
            if (count < 2) return new double[] { 0, 0, 1, 1 };
            return InterpKnots(u, 1);
        }

        /// <summary>Interpolation with given parameters and knots.</summary>
        public static bool InterpolateWith(IList<Vec3> Q, double[] u, int p, double[] U, out Vec3[] ctrl)
        {
            int n = Q.Count - 1;
            var A = new double[n + 1, n + 1];
            for (int k = 0; k <= n; k++)
            {
                int span = FindSpan(n, p, u[k], U);
                var N = BasisFuns(span, u[k], p, U);
                for (int j = 0; j <= p; j++) A[k, span - p + j] = N[j];
            }
            ctrl = new Vec3[n + 1];
            double[] x, y, z;
            if (!LinAlg.Solve(A, Q.Select(q => q.X).ToArray(), out x) || !LinAlg.Solve(A, Q.Select(q => q.Y).ToArray(), out y) ||
                !LinAlg.Solve(A, Q.Select(q => q.Z).ToArray(), out z)) return false;
            for (int k = 0; k <= n; k++) ctrl[k] = new Vec3(x[k], y[k], z[k]);
            return true;
        }

        /// <summary>Knots for least-squares approximation with n+1 control points (NURBS Book eq. 9.68/9.69).</summary>
        public static double[] ApproxKnots(double[] u, int n, int p)
        {
            int m = u.Length - 1;
            var U = new double[n + p + 2];
            for (int j = 0; j <= p; j++) { U[j] = 0; U[n + p + 1 - j] = 1; }
            double d = (double)(m + 1) / (n - p + 1);
            for (int j = 1; j <= n - p; j++)
            {
                int i = (int)(j * d);
                double a = j * d - i;
                U[p + j] = (1 - a) * u[Math.Max(0, i - 1)] + a * u[Math.Min(m, i)];
            }
            return U;
        }

        /// <summary>
        /// Least-squares B-spline fit with nc control points; first/last control points equal the end points.
        /// </summary>
        public static bool FitLeastSquares(IList<Vec3> Q, double[] u, int p, int nc, double[] U, out Vec3[] ctrl)
        {
            int m = Q.Count - 1, n = nc - 1;
            ctrl = new Vec3[nc];
            ctrl[0] = Q[0]; ctrl[n] = Q[m];
            if (n < 2) return true;
            var Nmat = new double[m + 1, n + 1];
            for (int k = 0; k <= m; k++)
            {
                int span = FindSpan(n, p, u[k], U);
                var N = BasisFuns(span, u[k], p, U);
                for (int j = 0; j <= p; j++) Nmat[k, span - p + j] = N[j];
            }
            int sz = n - 1;
            var NtN = new double[sz, sz];
            var Rx = new double[sz]; var Ry = new double[sz]; var Rz = new double[sz];
            for (int k = 1; k < m; k++)
            {
                Vec3 R = Q[k] - Q[0] * Nmat[k, 0] - Q[m] * Nmat[k, n];
                for (int i = 1; i < n; i++)
                {
                    double ni = Nmat[k, i];
                    if (ni == 0) continue;
                    Rx[i - 1] += ni * R.X; Ry[i - 1] += ni * R.Y; Rz[i - 1] += ni * R.Z;
                    for (int j = 1; j < n; j++) NtN[i - 1, j - 1] += ni * Nmat[k, j];
                }
            }
            for (int i = 0; i < sz; i++) NtN[i, i] += 1e-12;
            double[] x, y, z;
            if (!LinAlg.Solve(NtN, Rx, out x) || !LinAlg.Solve(NtN, Ry, out y) || !LinAlg.Solve(NtN, Rz, out z)) return false;
            for (int i = 1; i < n; i++) ctrl[i] = new Vec3(x[i - 1], y[i - 1], z[i - 1]);
            return true;
        }

        /// <summary>Smoothing approximation: fewest control points whose curve stays within tol of every point.</summary>
        public static void Approximate(IList<Vec3> Q, int p, double tol, out Vec3[] ctrl, out double[] knots)
        {
            int m = Q.Count - 1;
            var u = CurvePrim.ChordParams(Q);
            int lo = p + 1, cap = Math.Min(m + 1, 150);
            Vec3[] best = null; double[] bestU = null;
            Vec3[] last = null; double[] lastU = null;
            Func<int, double> err = nc =>
            {
                double[] U = ApproxKnots(u, nc - 1, p);
                Vec3[] c;
                if (!FitLeastSquares(Q, u, p, nc, U, out c)) return double.MaxValue;
                last = c; lastU = U;
                double e = 0;
                for (int k = 0; k <= m; k++) e = Math.Max(e, (Evaluate(p, c, U, u[k]) - Q[k]).Length);
                if (e <= tol && (best == null || nc < best.Length)) { best = c; bestU = U; }
                return e;
            };
            if (err(cap) > tol)
            {
                // tolerance not reachable with the allowed control points
                if (cap == m + 1) Interpolate(Q, p, out ctrl, out knots);
                else { ctrl = last; knots = lastU; }
                return;
            }
            int hi = cap;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (err(mid) <= tol) hi = mid; else lo = mid;
            }
            err(lo);
            ctrl = best; knots = bestU;
        }

        public static Vec3 Evaluate(int p, Vec3[] ctrl, double[] U, double u)
        {
            int n = ctrl.Length - 1;
            int span = FindSpan(n, p, u, U);
            var N = BasisFuns(span, u, p, U);
            Vec3 c = Vec3.Zero;
            for (int j = 0; j <= p; j++) c += ctrl[span - p + j] * N[j];
            return c;
        }
    }
}
