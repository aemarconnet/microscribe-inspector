using System;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>Small dense linear-algebra helpers (no external dependencies).</summary>
    public static class LinAlg
    {
        /// <summary>
        /// Eigen-decomposition of a symmetric 3x3 matrix (cyclic Jacobi).
        /// Returns eigenvalues ascending; vectors[k] is the unit eigenvector of values[k].
        /// </summary>
        public static void SymmetricEigen3(double[,] m, out double[] values, out Vec3[] vectors)
        {
            var a = (double[,])m.Clone();
            var v = new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            for (int sweep = 0; sweep < 64; sweep++)
            {
                double off = Math.Abs(a[0, 1]) + Math.Abs(a[0, 2]) + Math.Abs(a[1, 2]);
                if (off < 1e-30) break;
                for (int p = 0; p < 2; p++)
                    for (int q = p + 1; q < 3; q++)
                    {
                        if (Math.Abs(a[p, q]) < 1e-300) continue;
                        double theta = (a[q, q] - a[p, p]) / (2 * a[p, q]);
                        double t = Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                        if (theta == 0) t = 1;
                        double c = 1 / Math.Sqrt(t * t + 1), s = t * c;
                        for (int k = 0; k < 3; k++)
                        {
                            double akp = a[k, p], akq = a[k, q];
                            a[k, p] = c * akp - s * akq;
                            a[k, q] = s * akp + c * akq;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            double apk = a[p, k], aqk = a[q, k];
                            a[p, k] = c * apk - s * aqk;
                            a[q, k] = s * apk + c * aqk;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            double vkp = v[k, p], vkq = v[k, q];
                            v[k, p] = c * vkp - s * vkq;
                            v[k, q] = s * vkp + c * vkq;
                        }
                    }
            }
            var idx = new[] { 0, 1, 2 };
            Array.Sort(new[] { a[0, 0], a[1, 1], a[2, 2] }, idx);
            values = new double[3];
            vectors = new Vec3[3];
            for (int k = 0; k < 3; k++)
            {
                int j = idx[k];
                values[k] = a[j, j];
                vectors[k] = new Vec3(v[0, j], v[1, j], v[2, j]).Normalized();
            }
        }

        /// <summary>Centroid and scatter (covariance * n) matrix of points.</summary>
        public static Vec3 Scatter(Vec3[] pts, out double[,] s)
        {
            Vec3 c = Vec3.Zero;
            foreach (var p in pts) c += p;
            c /= pts.Length;
            s = new double[3, 3];
            foreach (var p in pts)
            {
                Vec3 d = p - c;
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        s[i, j] += d[i] * d[j];
            }
            return c;
        }

        /// <summary>Solves A x = b in place by Gaussian elimination with partial pivoting. Returns false if singular.</summary>
        public static bool Solve(double[,] A, double[] b, out double[] x)
        {
            int n = b.Length;
            var a = (double[,])A.Clone();
            var r = (double[])b.Clone();
            x = new double[n];
            for (int col = 0; col < n; col++)
            {
                int piv = col;
                double best = Math.Abs(a[col, col]);
                for (int i = col + 1; i < n; i++)
                    if (Math.Abs(a[i, col]) > best) { best = Math.Abs(a[i, col]); piv = i; }
                if (best < 1e-300) return false;
                if (piv != col)
                {
                    for (int j = 0; j < n; j++) { double t = a[col, j]; a[col, j] = a[piv, j]; a[piv, j] = t; }
                    double tb = r[col]; r[col] = r[piv]; r[piv] = tb;
                }
                for (int i = col + 1; i < n; i++)
                {
                    double f = a[i, col] / a[col, col];
                    if (f == 0) continue;
                    for (int j = col; j < n; j++) a[i, j] -= f * a[col, j];
                    r[i] -= f * r[col];
                }
            }
            for (int i = n - 1; i >= 0; i--)
            {
                double s = r[i];
                for (int j = i + 1; j < n; j++) s -= a[i, j] * x[j];
                x[i] = s / a[i, i];
            }
            return true;
        }

        /// <summary>Linear least squares via normal equations: minimise |J x - y|.</summary>
        public static bool LeastSquares(double[][] rows, double[] y, out double[] x)
        {
            int n = rows[0].Length;
            var ata = new double[n, n];
            var aty = new double[n];
            for (int k = 0; k < rows.Length; k++)
            {
                var r = rows[k];
                for (int i = 0; i < n; i++)
                {
                    aty[i] += r[i] * y[k];
                    for (int j = 0; j < n; j++) ata[i, j] += r[i] * r[j];
                }
            }
            return Solve(ata, aty, out x);
        }
    }

    /// <summary>
    /// Levenberg-Marquardt for small problems: minimise sum r_i(p)^2.
    /// Jacobian by central differences.
    /// </summary>
    public sealed class LevenbergMarquardt
    {
        public delegate void ResidualFn(double[] p, double[] r);

        public int MaxIterations = 200;
        public double Tolerance = 1e-12;
        public int Iterations { get; private set; }
        public double FinalCost { get; private set; }

        public double[] Minimize(ResidualFn f, double[] p0, int m, double[] step = null)
        {
            int n = p0.Length;
            var p = (double[])p0.Clone();
            var r = new double[m];
            var rp = new double[m];
            var rm = new double[m];
            var J = new double[m, n];
            f(p, r);
            double cost = SumSq(r);
            double lambda = 1e-3;
            Iterations = 0;
            for (int it = 0; it < MaxIterations; it++)
            {
                Iterations = it + 1;
                // numeric Jacobian
                for (int j = 0; j < n; j++)
                {
                    double h = step != null ? step[j] : 1e-6 * Math.Max(1.0, Math.Abs(p[j]));
                    double old = p[j];
                    p[j] = old + h; f(p, rp);
                    p[j] = old - h; f(p, rm);
                    p[j] = old;
                    for (int i = 0; i < m; i++) J[i, j] = (rp[i] - rm[i]) / (2 * h);
                }
                var jtj = new double[n, n];
                var jtr = new double[n];
                for (int i = 0; i < m; i++)
                    for (int a = 0; a < n; a++)
                    {
                        double ja = J[i, a];
                        if (ja == 0) continue;
                        jtr[a] += ja * r[i];
                        for (int b = 0; b < n; b++) jtj[a, b] += ja * J[i, b];
                    }
                bool improved = false;
                for (int tries = 0; tries < 12; tries++)
                {
                    var A = (double[,])jtj.Clone();
                    for (int a = 0; a < n; a++) A[a, a] += lambda * Math.Max(jtj[a, a], 1e-12);
                    var g = new double[n];
                    for (int a = 0; a < n; a++) g[a] = -jtr[a];
                    double[] dp;
                    if (!LinAlg.Solve(A, g, out dp)) { lambda *= 10; continue; }
                    var pn = new double[n];
                    for (int a = 0; a < n; a++) pn[a] = p[a] + dp[a];
                    f(pn, rp);
                    double cn = SumSq(rp);
                    if (cn < cost)
                    {
                        double rel = (cost - cn) / Math.Max(cost, 1e-300);
                        p = pn; Array.Copy(rp, r, m);
                        cost = cn;
                        lambda = Math.Max(lambda / 10, 1e-12);
                        improved = true;
                        if (rel < Tolerance) { FinalCost = cost; return p; }
                        break;
                    }
                    lambda *= 10;
                }
                if (!improved) break;
            }
            FinalCost = cost;
            return p;
        }

        static double SumSq(double[] r)
        {
            double s = 0;
            for (int i = 0; i < r.Length; i++) s += r[i] * r[i];
            return s;
        }
    }
}
