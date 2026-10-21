using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>
    /// Triangle mesh of a nominal CAD model (STL), in the model's own coordinates (mm), with an AABB tree
    /// for closest-point and signed-distance queries. Distances are positive outside the material.
    /// </summary>
    public sealed class TriMesh
    {
        public Vec3[] V;
        /// <summary>Three vertex indices per triangle.</summary>
        public int[] T;
        /// <summary>Outward unit normal per triangle.</summary>
        public Vec3[] N;
        public Vec3 Min, Max;

        sealed class Node { public Vec3 Min, Max; public Node L, R; public int Start, Count; }
        Node _root;
        int[] _order;

        public int TriangleCount { get { return T.Length / 3; } }

        /// <summary>Reads a binary or ASCII STL; <paramref name="scale"/> converts file units to mm (25.4 for inch).</summary>
        public static TriMesh LoadStl(string path, double scale = 1)
        {
            byte[] data = File.ReadAllBytes(path);
            var corners = new List<Vec3>();
            long n = data.Length >= 84 ? (long)BitConverter.ToUInt32(data, 80) : -1;
            bool text = data.Length > 5 && System.Text.Encoding.ASCII.GetString(data, 0, 5).Equals("solid", StringComparison.OrdinalIgnoreCase);
            bool binary = n >= 0 && 84 + 50 * n == data.Length || (!text && n >= 0 && 84 + 50 * n <= data.Length);
            if (binary)
            {
                for (long i = 0; i < n; i++)
                {
                    int o = (int)(84 + 50 * i) + 12;
                    for (int k = 0; k < 3; k++, o += 12)
                        corners.Add(new Vec3(BitConverter.ToSingle(data, o), BitConverter.ToSingle(data, o + 4), BitConverter.ToSingle(data, o + 8)) * scale);
                }
            }
            else
            {
                var tok = System.Text.Encoding.ASCII.GetString(data).Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                for (int i = 0; i + 3 < tok.Length; i++)
                    if (tok[i].Equals("vertex", StringComparison.OrdinalIgnoreCase))
                    {
                        corners.Add(new Vec3(double.Parse(tok[i + 1], CultureInfo.InvariantCulture), double.Parse(tok[i + 2], CultureInfo.InvariantCulture),
                            double.Parse(tok[i + 3], CultureInfo.InvariantCulture)) * scale);
                        i += 3;
                    }
            }
            if (corners.Count < 3) throw new InvalidDataException("No triangles found in " + Path.GetFileName(path));
            return FromCorners(corners);
        }

        /// <summary>Builds a mesh from triangle corners (3 per triangle), merging shared vertices.</summary>
        public static TriMesh FromCorners(IList<Vec3> corners)
        {
            var verts = new List<Vec3>();
            var tris = new List<int>();
            Vec3 lo, hi;
            AlignmentBounds(corners, out lo, out hi);
            double q = Math.Max(1e-6, (hi - lo).Length * 1e-7);   // merge tolerance
            var index = new Dictionary<Tuple<long, long, long>, int>();
            foreach (var c in corners)
            {
                var key = Tuple.Create((long)Math.Round(c.X / q), (long)Math.Round(c.Y / q), (long)Math.Round(c.Z / q));
                int vi;
                if (!index.TryGetValue(key, out vi)) { vi = verts.Count; verts.Add(c); index[key] = vi; }
                tris.Add(vi);
            }
            var m = new TriMesh { V = verts.ToArray() };
            // drop degenerate triangles
            var keep = new List<int>();
            for (int t = 0; t + 2 < tris.Count; t += 3)
            {
                int a = tris[t], b = tris[t + 1], c = tris[t + 2];
                if (a == b || b == c || a == c) continue;
                if ((m.V[b] - m.V[a]).Cross(m.V[c] - m.V[a]).Length < 1e-14) continue;
                keep.Add(a); keep.Add(b); keep.Add(c);
            }
            m.T = keep.ToArray();
            m.N = new Vec3[m.TriangleCount];
            double vol = 0;
            for (int t = 0; t < m.TriangleCount; t++)
            {
                Vec3 a = m.V[m.T[3 * t]], b = m.V[m.T[3 * t + 1]], c = m.V[m.T[3 * t + 2]];
                m.N[t] = (b - a).Cross(c - a).Normalized();
                vol += a.Dot(b.Cross(c)) / 6;
            }
            // closed solids exported inside-out: make the normals point out of the material
            if (vol < 0) for (int t = 0; t < m.N.Length; t++) m.N[t] = -m.N[t];
            AlignmentBounds(m.V, out m.Min, out m.Max);
            m.Build();
            return m;
        }

        static void AlignmentBounds(IEnumerable<Vec3> pts, out Vec3 min, out Vec3 max)
        {
            min = new Vec3(double.MaxValue, double.MaxValue, double.MaxValue);
            max = new Vec3(double.MinValue, double.MinValue, double.MinValue);
            foreach (var p in pts)
            {
                min = new Vec3(Math.Min(min.X, p.X), Math.Min(min.Y, p.Y), Math.Min(min.Z, p.Z));
                max = new Vec3(Math.Max(max.X, p.X), Math.Max(max.Y, p.Y), Math.Max(max.Z, p.Z));
            }
        }

        // ------------------------------------------------------------------ AABB tree

        void Build()
        {
            int n = TriangleCount;
            _order = Enumerable.Range(0, n).ToArray();
            var cen = new Vec3[n];
            for (int t = 0; t < n; t++) cen[t] = (V[T[3 * t]] + V[T[3 * t + 1]] + V[T[3 * t + 2]]) / 3;
            _root = n > 0 ? Build(cen, 0, n) : null;
        }

        Node Build(Vec3[] cen, int start, int count)
        {
            var node = new Node { Start = start, Count = count };
            Vec3 lo = new Vec3(double.MaxValue, double.MaxValue, double.MaxValue), hi = new Vec3(double.MinValue, double.MinValue, double.MinValue);
            for (int i = start; i < start + count; i++)
            {
                int t = _order[i];
                for (int k = 0; k < 3; k++)
                {
                    Vec3 p = V[T[3 * t + k]];
                    lo = new Vec3(Math.Min(lo.X, p.X), Math.Min(lo.Y, p.Y), Math.Min(lo.Z, p.Z));
                    hi = new Vec3(Math.Max(hi.X, p.X), Math.Max(hi.Y, p.Y), Math.Max(hi.Z, p.Z));
                }
            }
            node.Min = lo; node.Max = hi;
            if (count <= 8) return node;
            Vec3 ext = hi - lo;
            int axis = ext.X > ext.Y ? (ext.X > ext.Z ? 0 : 2) : (ext.Y > ext.Z ? 1 : 2);
            var keys = new double[count];
            for (int i = 0; i < count; i++) { Vec3 c = cen[_order[start + i]]; keys[i] = axis == 0 ? c.X : axis == 1 ? c.Y : c.Z; }
            Array.Sort(keys, _order, start, count);
            int half = count / 2;
            node.L = Build(cen, start, half);
            node.R = Build(cen, start + half, count - half);
            return node;
        }

        static double BoxDist2(Vec3 p, Vec3 lo, Vec3 hi)
        {
            double dx = Math.Max(0, Math.Max(lo.X - p.X, p.X - hi.X));
            double dy = Math.Max(0, Math.Max(lo.Y - p.Y, p.Y - hi.Y));
            double dz = Math.Max(0, Math.Max(lo.Z - p.Z, p.Z - hi.Z));
            return dx * dx + dy * dy + dz * dz;
        }

        /// <summary>Closest point on the mesh to p, with the triangle's outward normal.</summary>
        public bool Closest(Vec3 p, out Vec3 q, out Vec3 n, out int tri)
        {
            q = n = Vec3.Zero; tri = -1;
            if (_root == null) return false;
            double best = double.MaxValue;
            var stack = new Stack<Node>();
            stack.Push(_root);
            while (stack.Count > 0)
            {
                var nd = stack.Pop();
                if (BoxDist2(p, nd.Min, nd.Max) >= best) continue;
                if (nd.L == null)
                {
                    for (int i = nd.Start; i < nd.Start + nd.Count; i++)
                    {
                        int t = _order[i];
                        Vec3 c = ClosestOnTriangle(p, V[T[3 * t]], V[T[3 * t + 1]], V[T[3 * t + 2]]);
                        double d = (c - p).LengthSquared;
                        if (d < best) { best = d; q = c; tri = t; }
                    }
                    continue;
                }
                double dl = BoxDist2(p, nd.L.Min, nd.L.Max), dr = BoxDist2(p, nd.R.Min, nd.R.Max);
                if (dl < dr) { stack.Push(nd.R); stack.Push(nd.L); } else { stack.Push(nd.L); stack.Push(nd.R); }
            }
            if (tri < 0) return false;
            n = N[tri];
            return true;
        }

        /// <summary>Signed distance (positive outside the material); NaN for an empty mesh.</summary>
        public double SignedDistance(Vec3 p)
        {
            Vec3 q, n; int t;
            if (!Closest(p, out q, out n, out t)) return double.NaN;
            double d = (p - q).Length;
            return (p - q).Dot(n) >= 0 ? d : -d;
        }

        /// <summary>Closest point on triangle abc (Ericson, Real-Time Collision Detection 5.1.5).</summary>
        public static Vec3 ClosestOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c)
        {
            Vec3 ab = b - a, ac = c - a, ap = p - a;
            double d1 = ab.Dot(ap), d2 = ac.Dot(ap);
            if (d1 <= 0 && d2 <= 0) return a;
            Vec3 bp = p - b;
            double d3 = ab.Dot(bp), d4 = ac.Dot(bp);
            if (d3 >= 0 && d4 <= d3) return b;
            double vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
            Vec3 cp = p - c;
            double d5 = ab.Dot(cp), d6 = ac.Dot(cp);
            if (d6 >= 0 && d5 <= d6) return c;
            double vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
            double va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
            double den = 1 / (va + vb + vc);
            return a + ab * (vb * den) + ac * (vc * den);
        }
    }

    /// <summary>Aligning measured points to a nominal model.</summary>
    public static class ModelFit
    {
        /// <summary>
        /// Deviation of a probed point from the model: signed distance of the ball centre minus the ball
        /// radius (the ball surface touches the part), positive = material above nominal.
        /// </summary>
        public static double Deviation(TriMesh mesh, Mat4 toModel, Vec3 ballCentre, double radius)
        {
            return mesh.SignedDistance(toModel.TransformPoint(ballCentre)) - radius;
        }

        /// <summary>Rigid transform taking <paramref name="from"/> onto <paramref name="to"/> (Horn's quaternion method, ≥3 pairs).</summary>
        public static Mat4 FromPairs(IList<Vec3> from, IList<Vec3> to, out double rms)
        {
            int n = Math.Min(from.Count, to.Count);
            Vec3 ca = Vec3.Zero, cb = Vec3.Zero;
            for (int i = 0; i < n; i++) { ca += from[i]; cb += to[i]; }
            ca = ca / n; cb = cb / n;
            var S = new double[3, 3];
            for (int i = 0; i < n; i++)
            {
                Vec3 a = from[i] - ca, b = to[i] - cb;
                double[] av = { a.X, a.Y, a.Z }, bv = { b.X, b.Y, b.Z };
                for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) S[r, c] += av[r] * bv[c];
            }
            double sxx = S[0, 0], sxy = S[0, 1], sxz = S[0, 2], syx = S[1, 0], syy = S[1, 1], syz = S[1, 2], szx = S[2, 0], szy = S[2, 1], szz = S[2, 2];
            var N = new double[,]
            {
                { sxx + syy + szz, syz - szy, szx - sxz, sxy - syx },
                { syz - szy, sxx - syy - szz, sxy + syx, szx + sxz },
                { szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy },
                { sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz }
            };
            double[] q = MaxEigenvector(N);
            double w = q[0], x = q[1], y = q[2], z = q[3];
            var m = Mat4.Identity();
            m[0, 0] = w * w + x * x - y * y - z * z; m[0, 1] = 2 * (x * y - w * z); m[0, 2] = 2 * (x * z + w * y);
            m[1, 0] = 2 * (x * y + w * z); m[1, 1] = w * w - x * x + y * y - z * z; m[1, 2] = 2 * (y * z - w * x);
            m[2, 0] = 2 * (x * z - w * y); m[2, 1] = 2 * (y * z + w * x); m[2, 2] = w * w - x * x - y * y + z * z;
            Vec3 t = cb - m.TransformVector(ca);
            m[0, 3] = t.X; m[1, 3] = t.Y; m[2, 3] = t.Z;
            double ss = 0;
            for (int i = 0; i < n; i++) ss += (m.TransformPoint(from[i]) - to[i]).LengthSquared;
            rms = Math.Sqrt(ss / Math.Max(1, n));
            return m;
        }

        /// <summary>Eigenvector of the largest eigenvalue of a small symmetric matrix (cyclic Jacobi).</summary>
        static double[] MaxEigenvector(double[,] a0)
        {
            int n = a0.GetLength(0);
            var a = (double[,])a0.Clone();
            var v = new double[n, n];
            for (int i = 0; i < n; i++) v[i, i] = 1;
            for (int sweep = 0; sweep < 60; sweep++)
            {
                double off = 0;
                for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) off += a[i, j] * a[i, j];
                if (off < 1e-30) break;
                for (int p = 0; p < n; p++)
                    for (int r = p + 1; r < n; r++)
                    {
                        if (Math.Abs(a[p, r]) < 1e-300) continue;
                        double th = (a[r, r] - a[p, p]) / (2 * a[p, r]);
                        double t = Math.Sign(th == 0 ? 1 : th) / (Math.Abs(th) + Math.Sqrt(th * th + 1));
                        double c = 1 / Math.Sqrt(t * t + 1), s = t * c;
                        for (int k = 0; k < n; k++)
                        {
                            double akp = a[k, p], akr = a[k, r];
                            a[k, p] = c * akp - s * akr; a[k, r] = s * akp + c * akr;
                        }
                        for (int k = 0; k < n; k++)
                        {
                            double apk = a[p, k], ark = a[r, k];
                            a[p, k] = c * apk - s * ark; a[r, k] = s * apk + c * ark;
                        }
                        for (int k = 0; k < n; k++)
                        {
                            double vkp = v[k, p], vkr = v[k, r];
                            v[k, p] = c * vkp - s * vkr; v[k, r] = s * vkp + c * vkr;
                        }
                    }
            }
            int best = 0;
            for (int i = 1; i < n; i++) if (a[i, i] > a[best, best]) best = i;
            var e = new double[n];
            for (int k = 0; k < n; k++) e[k] = v[k, best];
            return e;
        }

        /// <summary>Result of a best fit.</summary>
        public sealed class FitResult
        {
            public Mat4 Transform;
            public double Rms, MaxAbs;
            public int Used, Total, Iterations;
        }

        /// <summary>
        /// Best fit (point-to-plane ICP): refines the machine → model transform so every probe ball sits
        /// one radius outside the model surface. Needs a rough starting alignment (a few mm).
        /// </summary>
        public static FitResult BestFit(TriMesh mesh, IList<Vec3> centres, IList<double> radii, Mat4 start, int maxIterations = 60)
        {
            Mat4 T = start.Clone();
            int n = centres.Count;
            double rmsPrev = double.MaxValue;
            var res = new FitResult { Total = n };
            for (int it = 0; it < maxIterations; it++)
            {
                var q = new Vec3[n]; var c = new Vec3[n]; var nn = new Vec3[n]; var r = new double[n];
                double ss = 0; int m = 0;
                for (int i = 0; i < n; i++)
                {
                    q[i] = T.TransformPoint(centres[i]);
                    int tri;
                    if (!mesh.Closest(q[i], out c[i], out nn[i], out tri)) return null;
                    r[i] = (q[i] - c[i]).Dot(nn[i]) - radii[i];
                }
                // after a few iterations, ignore points far from the surface (other parts, fixtures)
                double limit = it < 3 ? double.MaxValue : Math.Max(3 * rmsPrev, 0.2);
                Vec3 qc = Vec3.Zero;
                for (int i = 0; i < n; i++) if (Math.Abs(r[i]) <= limit) { qc += q[i]; m++; }
                if (m < 6) return null;
                qc = qc / m;
                var rows = new List<double[]>(); var rhs = new List<double>();
                for (int i = 0; i < n; i++)
                {
                    if (Math.Abs(r[i]) > limit) continue;
                    Vec3 a = (q[i] - qc).Cross(nn[i]);
                    rows.Add(new[] { a.X, a.Y, a.Z, nn[i].X, nn[i].Y, nn[i].Z });
                    rhs.Add(-r[i]);
                    ss += r[i] * r[i];
                }
                // a little damping keeps directions the points cannot fix (e.g. one plane only) in place
                for (int k = 0; k < 6; k++) { var row = new double[6]; row[k] = 1e-3 * Math.Sqrt(m); rows.Add(row); rhs.Add(0); }
                double[] x;
                if (!LinAlg.LeastSquares(rows.ToArray(), rhs.ToArray(), out x)) return null;
                Vec3 w = new Vec3(x[0], x[1], x[2]), t = new Vec3(x[3], x[4], x[5]);
                double ang = w.Length;
                Mat4 R = Mat4.Identity();
                if (ang > 1e-15)
                {
                    Vec3 k = w / ang; double co = Math.Cos(ang), si = Math.Sin(ang), vc = 1 - co;
                    R[0, 0] = co + k.X * k.X * vc; R[0, 1] = k.X * k.Y * vc - k.Z * si; R[0, 2] = k.X * k.Z * vc + k.Y * si;
                    R[1, 0] = k.Y * k.X * vc + k.Z * si; R[1, 1] = co + k.Y * k.Y * vc; R[1, 2] = k.Y * k.Z * vc - k.X * si;
                    R[2, 0] = k.Z * k.X * vc - k.Y * si; R[2, 1] = k.Z * k.Y * vc + k.X * si; R[2, 2] = co + k.Z * k.Z * vc;
                }
                Mat4 D = Mat4.Translation(qc.X + t.X, qc.Y + t.Y, qc.Z + t.Z) * R * Mat4.Translation(-qc.X, -qc.Y, -qc.Z);
                T = D * T;
                rmsPrev = Math.Sqrt(ss / m);
                res.Iterations = it + 1;
                if (ang < 1e-9 && t.Length < 1e-7) break;
            }
            // final statistics with the robust limit
            var dev = centres.Select((p, i) => Deviation(mesh, T, p, radii[i])).ToList();
            double lim = Math.Max(3 * rmsPrev, 0.2);
            var used = dev.Where(d => Math.Abs(d) <= lim).ToList();
            res.Transform = T;
            res.Used = used.Count;
            res.Rms = used.Count > 0 ? Math.Sqrt(used.Sum(d => d * d) / used.Count) : double.NaN;
            res.MaxAbs = used.Count > 0 ? used.Max(d => Math.Abs(d)) : double.NaN;
            return res;
        }
    }
}
