using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>A line or a counter-clockwise arc of a 2D sketch (sketch-plane coordinates, mm).</summary>
    public sealed class Seg2
    {
        public bool IsArc;
        public double X0, Y0, X1, Y1;          // start / end (arcs: in the direction they were traced)
        public double CX, CY, R;               // arcs
        public bool Ccw;                       // arcs: traced counter-clockwise
        public int From, To;                   // range of the traced points it replaces
        public double Deviation;

        /// <summary>DXF-style angles (degrees, counter-clockwise from A0 to A1).</summary>
        public void Angles(out double a0, out double a1)
        {
            double s = Math.Atan2(Y0 - CY, X0 - CX), e = Math.Atan2(Y1 - CY, X1 - CX);
            if (!Ccw) { double t = s; s = e; e = t; }
            while (e <= s) e += 2 * Math.PI;
            a0 = s * 180 / Math.PI; a1 = e * 180 / Math.PI;
        }
    }

    /// <summary>
    /// A traced profile turned into a sketch: the points are projected onto a plane and split into the
    /// fewest lines and arcs that stay within a tolerance; neighbouring pieces share their end points
    /// (lines meet at their intersection, so corners stay sharp). Exports to DXF.
    /// </summary>
    public sealed class Sketch2D
    {
        public Vec3 Origin, U, V, Normal;      // machine frame
        public List<Seg2> Segments = new List<Seg2>();
        public double MaxDeviation;
        public bool Closed;

        public int Lines { get { return Segments.Count(s => !s.IsArc); } }
        public int Arcs { get { return Segments.Count(s => s.IsArc); } }

        public Vec3 To3D(double x, double y) { return Origin + U * x + V * y; }

        /// <summary>
        /// Sketch axes on the plane, following the part frame: a plane facing part Z uses part X/Y, one facing
        /// Y uses X/Z, one facing X uses Y/Z; the origin is the part origin projected onto the plane.
        /// </summary>
        public static void Basis(PlanePrim plane, Mat4 partFrame, out Vec3 o, out Vec3 u, out Vec3 v)
        {
            Mat4 inv = partFrame.RigidInverse();
            Vec3 n = plane.Normal.Normalized();
            Vec3 px = inv.TransformVector(Vec3.UnitX), py = inv.TransformVector(Vec3.UnitY), pz = inv.TransformVector(Vec3.UnitZ);
            double ax = Math.Abs(n.Dot(px)), ay = Math.Abs(n.Dot(py)), az = Math.Abs(n.Dot(pz));
            Vec3 a, b;
            if (az >= ax && az >= ay) { a = px; b = py; } else if (ay >= ax) { a = px; b = pz; } else { a = py; b = pz; }
            u = (a - n * a.Dot(n)).Normalized();
            v = b - n * b.Dot(n); v = (v - u * v.Dot(u)).Normalized();
            o = plane.Project(inv.TransformPoint(Vec3.Zero));
        }

        struct P2
        {
            public double X, Y;
            public P2(double x, double y) { X = x; Y = y; }
            public static P2 operator +(P2 a, P2 b) { return new P2(a.X + b.X, a.Y + b.Y); }
            public static P2 operator -(P2 a, P2 b) { return new P2(a.X - b.X, a.Y - b.Y); }
            public static P2 operator *(P2 a, double s) { return new P2(a.X * s, a.Y * s); }
            public double Dot(P2 b) { return X * b.X + Y * b.Y; }
            public double Len { get { return Math.Sqrt(X * X + Y * Y); } }
        }

        public static Sketch2D Build(IList<Vec3> pts3, PlanePrim plane, Mat4 partFrame, double tol, bool closed)
        {
            var sk = new Sketch2D { Closed = closed };
            Basis(plane, partFrame, out sk.Origin, out sk.U, out sk.V);
            sk.Normal = sk.U.Cross(sk.V).Normalized();
            var q = new List<P2>();
            foreach (var p in pts3)
            {
                var w = p - sk.Origin;
                var c = new P2(w.Dot(sk.U), w.Dot(sk.V));
                if (q.Count == 0 || (c - q[q.Count - 1]).Len > 1e-6) q.Add(c);
            }
            if (closed && q.Count > 2 && (q[0] - q[q.Count - 1]).Len > 1e-6) q.Add(q[0]);
            int n = q.Count;
            if (n < 2) return sk;
            tol = Math.Max(1e-4, tol);

            // greedy: from each start, the piece (line or arc) that covers the most points within the tolerance
            var pieces = new List<Seg2>();
            int s0 = 0;
            while (s0 < n - 1)
            {
                int eL = s0 + 1;
                for (int e = s0 + 2; e < n; e++) { if (LineDev(q, s0, e) <= tol) eL = e; else if (e - eL > 2) break; }
                int eA = -1;
                double cx = 0, cy = 0, r = 0;
                for (int e = s0 + 4; e < n; e++)
                {
                    double x, y, rr;
                    if (ArcFit(q, s0, e, tol, out x, out y, out rr)) { eA = e; cx = x; cy = y; r = rr; }
                    else if (e - Math.Max(eA, Math.Max(eL, s0 + 3)) > 8) break;
                }
                Seg2 seg;
                if (eA >= eL + 2)
                {
                    seg = new Seg2 { IsArc = true, CX = cx, CY = cy, R = r, From = s0, To = eA };
                    P2 c = new P2(cx, cy), a = Radial(q[s0], c, r), b = Radial(q[eA], c, r);
                    seg.X0 = a.X; seg.Y0 = a.Y; seg.X1 = b.X; seg.Y1 = b.Y;
                    seg.Ccw = Sweep(q, s0, eA, c) > 0;
                    seg.Deviation = Enumerable.Range(s0, eA - s0 + 1).Max(i => Math.Abs((q[i] - c).Len - r));
                }
                else
                {
                    P2 o, d;
                    LineFit(q, s0, eL, out o, out d);
                    P2 a = o + d * (q[s0] - o).Dot(d), b = o + d * (q[eL] - o).Dot(d);
                    seg = new Seg2 { X0 = a.X, Y0 = a.Y, X1 = b.X, Y1 = b.Y, From = s0, To = eL, Deviation = LineDev(q, s0, eL) };
                }
                pieces.Add(seg);
                s0 = seg.To;
            }

            // neighbours share their end points: lines meet at their intersection, otherwise the average
            int m = pieces.Count;
            for (int i = 0; i < (closed ? m : m - 1); i++)
            {
                var a = pieces[i]; var b = pieces[(i + 1) % m];
                P2 ea = new P2(a.X1, a.Y1), sb = new P2(b.X0, b.Y0), j = (ea + sb) * 0.5;
                if (!a.IsArc && !b.IsArc)
                {
                    P2 x;
                    if (Intersect(new P2(a.X0, a.Y0), ea, sb, new P2(b.X1, b.Y1), out x) && (x - j).Len < 3 * tol + 1.0) j = x;
                }
                else if (a.IsArc != b.IsArc)
                {
                    // a line running into an arc tangentially: join at the tangent point (foot of the centre)
                    var ln = a.IsArc ? b : a; var ar = a.IsArc ? a : b;
                    P2 l0 = new P2(ln.X0, ln.Y0), d = new P2(ln.X1, ln.Y1) - l0;
                    double L = d.Len;
                    if (L > 1e-9)
                    {
                        d = d * (1 / L);
                        P2 c = new P2(ar.CX, ar.CY), t = l0 + d * (c - l0).Dot(d);
                        if (Math.Abs((c - t).Len - ar.R) < 3 * tol && (t - j).Len < 3 * tol + 2.0) j = t;
                    }
                }
                a.X1 = j.X; a.Y1 = j.Y; b.X0 = j.X; b.Y0 = j.Y;
            }
            // arcs keep their fitted centre; the joints move onto the circle (and the neighbours follow)
            for (int i = 0; i < m; i++)
            {
                var a = pieces[i];
                if (!a.IsArc) continue;
                P2 c = new P2(a.CX, a.CY), s = new P2(a.X0, a.Y0), e = new P2(a.X1, a.Y1);
                a.R = 0.5 * ((s - c).Len + (e - c).Len);
                s = Radial(s, c, a.R); e = Radial(e, c, a.R);
                a.X0 = s.X; a.Y0 = s.Y; a.X1 = e.X; a.Y1 = e.Y;
                var prev = i > 0 ? pieces[i - 1] : closed ? pieces[m - 1] : null;
                var next = i < m - 1 ? pieces[i + 1] : closed ? pieces[0] : null;
                if (prev != null && prev != a) { prev.X1 = s.X; prev.Y1 = s.Y; }
                if (next != null && next != a) { next.X0 = e.X; next.Y0 = e.Y; }
            }
            sk.Segments = pieces;
            sk.MaxDeviation = pieces.Count > 0 ? pieces.Max(p => p.Deviation) : 0;
            return sk;
        }

        static P2 Radial(P2 p, P2 c, double r) { P2 d = p - c; double l = d.Len; return l > 1e-12 ? c + d * (r / l) : p; }

        static void LineFit(List<P2> q, int s, int e, out P2 o, out P2 d)
        {
            int n = e - s + 1;
            double mx = 0, my = 0;
            for (int i = s; i <= e; i++) { mx += q[i].X; my += q[i].Y; }
            mx /= n; my /= n;
            double sxx = 0, sxy = 0, syy = 0;
            for (int i = s; i <= e; i++) { double x = q[i].X - mx, y = q[i].Y - my; sxx += x * x; sxy += x * y; syy += y * y; }
            double ang = 0.5 * Math.Atan2(2 * sxy, sxx - syy);
            o = new P2(mx, my); d = new P2(Math.Cos(ang), Math.Sin(ang));
        }

        static double LineDev(List<P2> q, int s, int e)
        {
            P2 o, d;
            LineFit(q, s, e, out o, out d);
            double worst = 0;
            for (int i = s; i <= e; i++) { P2 w = q[i] - o; worst = Math.Max(worst, Math.Abs(w.X * d.Y - w.Y * d.X)); }
            return worst;
        }

        /// <summary>Algebraic circle fit (Kåsa) of points s..e; true when it is a real arc within the tolerance.</summary>
        static bool ArcFit(List<P2> q, int s, int e, double tol, out double cx, out double cy, out double r)
        {
            cx = cy = r = 0;
            P2 o = q[s];
            var rows = new double[e - s + 1][]; var rhs = new double[e - s + 1];
            for (int i = s; i <= e; i++) { P2 w = q[i] - o; rows[i - s] = new[] { w.X, w.Y, 1.0 }; rhs[i - s] = -(w.X * w.X + w.Y * w.Y); }
            double[] x;
            if (!LinAlg.LeastSquares(rows, rhs, out x)) return false;
            double lx = -x[0] / 2, ly = -x[1] / 2, r2 = lx * lx + ly * ly - x[2];
            if (r2 <= 0) return false;
            r = Math.Sqrt(r2); cx = lx + o.X; cy = ly + o.Y;
            double chord = (q[e] - q[s]).Len;
            if (r > 1e4 || r < tol || chord < 1e-9) return false;
            P2 c = new P2(cx, cy);
            if (Math.Abs(Sweep(q, s, e, c)) < 8 * Math.PI / 180) return false;
            for (int i = s; i <= e; i++) if (Math.Abs((q[i] - c).Len - r) > tol) return false;
            return true;
        }

        /// <summary>Signed angle swept around c from point s to point e (positive = counter-clockwise).</summary>
        static double Sweep(List<P2> q, int s, int e, P2 c)
        {
            double tot = 0;
            for (int i = s + 1; i <= e; i++)
            {
                P2 a = q[i - 1] - c, b = q[i] - c;
                tot += Math.Atan2(a.X * b.Y - a.Y * b.X, a.Dot(b));
            }
            return tot;
        }

        static bool Intersect(P2 a0, P2 a1, P2 b0, P2 b1, out P2 x)
        {
            P2 da = a1 - a0, db = b1 - b0;
            double den = da.X * db.Y - da.Y * db.X;
            x = a1;
            if (Math.Abs(den) < 1e-12 * da.Len * db.Len) return false;
            double t = ((b0.X - a0.X) * db.Y - (b0.Y - a0.Y) * db.X) / den;
            x = a0 + da * t;
            return true;
        }

        /// <summary>The sketch as a polyline in 3D (arcs sampled every <paramref name="step"/> degrees), machine frame.</summary>
        public List<Vec3> Polyline3D(double step = 5)
        {
            var r = new List<Vec3>();
            foreach (var s in Segments)
            {
                if (!s.IsArc) { r.Add(To3D(s.X0, s.Y0)); r.Add(To3D(s.X1, s.Y1)); continue; }
                double a0 = Math.Atan2(s.Y0 - s.CY, s.X0 - s.CX), a1 = Math.Atan2(s.Y1 - s.CY, s.X1 - s.CX);
                if (s.Ccw) { while (a1 <= a0) a1 += 2 * Math.PI; } else { while (a1 >= a0) a1 -= 2 * Math.PI; }
                int k = Math.Max(2, (int)Math.Ceiling(Math.Abs(a1 - a0) * 180 / Math.PI / step));
                for (int i = 0; i <= k; i++)
                {
                    double t = a0 + (a1 - a0) * i / k;
                    r.Add(To3D(s.CX + s.R * Math.Cos(t), s.CY + s.R * Math.Sin(t)));
                }
            }
            return r;
        }

        /// <summary>DXF (R12 ASCII) with LINE and ARC entities on layer PROFILE, in mm or inch.</summary>
        public string ToDxf(bool inch)
        {
            var ci = CultureInfo.InvariantCulture;
            double k = inch ? 1 / 25.4 : 1;
            var sb = new StringBuilder();
            Action<int, string> g = (code, val) => sb.Append(code).Append("\r\n").Append(val).Append("\r\n");
            Action<int, double> gn = (code, val) => g(code, val.ToString("0.######", ci));
            g(0, "SECTION"); g(2, "HEADER"); g(9, "$ACADVER"); g(1, "AC1009"); g(9, "$INSUNITS"); g(70, inch ? "1" : "4"); g(0, "ENDSEC");
            g(0, "SECTION"); g(2, "ENTITIES");
            foreach (var s in Segments)
            {
                if (!s.IsArc)
                {
                    g(0, "LINE"); g(8, "PROFILE");
                    gn(10, s.X0 * k); gn(20, s.Y0 * k); gn(30, 0); gn(11, s.X1 * k); gn(21, s.Y1 * k); gn(31, 0);
                }
                else
                {
                    double a0, a1;
                    s.Angles(out a0, out a1);
                    g(0, "ARC"); g(8, "PROFILE");
                    gn(10, s.CX * k); gn(20, s.CY * k); gn(30, 0); gn(40, s.R * k); gn(50, a0); gn(51, a1);
                }
            }
            g(0, "ENDSEC"); g(0, "EOF");
            return sb.ToString();
        }
    }
}
