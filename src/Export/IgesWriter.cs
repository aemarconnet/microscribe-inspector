using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MicroScribe.Core;

namespace MicroScribe.Export
{
    /// <summary>NURBS helpers for exact circles and arcs.</summary>
    public static class Nurbs
    {
        /// <summary>
        /// Rational quadratic arc on the unit circle from angle a0 to a1 (radians, a1 &gt; a0), in pieces of at
        /// most 90°: control points (x, y), weights and the clamped knot vector on [0, 1].
        /// </summary>
        public static void Arc(double a0, double a1, out double[] x, out double[] y, out double[] w, out double[] knots)
        {
            double span = a1 - a0;
            int seg = Math.Max(1, (int)Math.Ceiling(span / (Math.PI / 2) - 1e-9));
            double d = span / seg, wm = Math.Cos(d / 2);
            int n = 2 * seg + 1;
            x = new double[n]; y = new double[n]; w = new double[n];
            for (int k = 0; k <= seg; k++)
            {
                double a = a0 + k * d;
                x[2 * k] = Math.Cos(a); y[2 * k] = Math.Sin(a); w[2 * k] = 1;
                if (k < seg)
                {
                    double m = a + d / 2;
                    x[2 * k + 1] = Math.Cos(m) / wm; y[2 * k + 1] = Math.Sin(m) / wm; w[2 * k + 1] = wm;
                }
            }
            var kv = new List<double> { 0, 0, 0 };
            for (int k = 1; k < seg; k++) { kv.Add((double)k / seg); kv.Add((double)k / seg); }
            kv.AddRange(new double[] { 1, 1, 1 });
            knots = kv.ToArray();
        }
    }

    /// <summary>
    /// Minimal IGES 5.3 writer (millimetres): points (116), lines (110), rational B-spline curves (126) and
    /// surfaces (128). Analytic surfaces are written as exact rational B-spline surfaces.
    /// </summary>
    public sealed class IgesWriter
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        sealed class Entity { public int Type, Form; public string Params, Label; }
        readonly List<Entity> _ents = new List<Entity>();
        double _maxCoord = 1;

        public int Count { get { return _ents.Count; } }

        static string N(double v)
        {
            if (Math.Abs(v) < 1e-12) v = 0;
            return v.ToString("0.##########", CI);
        }

        void Track(Vec3 p) { _maxCoord = Math.Max(_maxCoord, Math.Max(Math.Abs(p.X), Math.Max(Math.Abs(p.Y), Math.Abs(p.Z)))); }

        static string Label(string name)
        {
            var s = new string((name ?? "").Where(char.IsLetterOrDigit).ToArray()).ToUpperInvariant();
            return s.Length > 8 ? s.Substring(0, 8) : s;
        }

        void Add(int type, string name, IEnumerable<string> p, int form = 0)
        {
            _ents.Add(new Entity { Type = type, Form = form, Label = Label(name), Params = type.ToString(CI) + "," + string.Join(",", p) + ";" });
        }

        public void AddPoint(string name, Vec3 p)
        {
            Track(p);
            Add(116, name, new[] { N(p.X), N(p.Y), N(p.Z), "0" });
        }

        public void AddLine(string name, Vec3 a, Vec3 b)
        {
            Track(a); Track(b);
            Add(110, name, new[] { N(a.X), N(a.Y), N(a.Z), N(b.X), N(b.Y), N(b.Z) });
        }

        /// <summary>B-spline curve (weights null = polynomial); planeNormal set for planar curves.</summary>
        public void AddCurve(string name, int degree, IList<Vec3> ctrl, IList<double> weights, IList<double> knots, bool closed, Vec3? planeNormal = null)
        {
            int k = ctrl.Count - 1;
            var p = new List<string> { k.ToString(CI), degree.ToString(CI), planeNormal.HasValue ? "1" : "0", closed ? "1" : "0", weights == null ? "1" : "0", "0" };
            p.AddRange(knots.Select(N));
            for (int i = 0; i <= k; i++) p.Add(N(weights == null ? 1 : weights[i]));
            foreach (var c in ctrl) { Track(c); p.Add(N(c.X)); p.Add(N(c.Y)); p.Add(N(c.Z)); }
            p.Add(N(knots[degree])); p.Add(N(knots[k + 1]));
            Vec3 nrm = planeNormal ?? Vec3.Zero;
            p.Add(N(nrm.X)); p.Add(N(nrm.Y)); p.Add(N(nrm.Z));
            Add(126, name, p);
        }

        /// <summary>B-spline surface, control net [u, v] (weights null = polynomial).</summary>
        public void AddSurface(string name, int degU, int degV, Vec3[,] ctrl, double[,] weights, IList<double> ku, IList<double> kv, bool closedU = false, bool closedV = false)
        {
            int k1 = ctrl.GetLength(0) - 1, k2 = ctrl.GetLength(1) - 1;
            var p = new List<string> { k1.ToString(CI), k2.ToString(CI), degU.ToString(CI), degV.ToString(CI), closedU ? "1" : "0", closedV ? "1" : "0", weights == null ? "1" : "0", "0", "0" };
            p.AddRange(ku.Select(N));
            p.AddRange(kv.Select(N));
            for (int j = 0; j <= k2; j++) for (int i = 0; i <= k1; i++) p.Add(N(weights == null ? 1 : weights[i, j]));
            for (int j = 0; j <= k2; j++) for (int i = 0; i <= k1; i++) { var c = ctrl[i, j]; Track(c); p.Add(N(c.X)); p.Add(N(c.Y)); p.Add(N(c.Z)); }
            p.Add(N(ku[degU])); p.Add(N(ku[k1 + 1])); p.Add(N(kv[degV])); p.Add(N(kv[k2 + 1]));
            Add(128, name, p);
        }

        /// <summary>Surface of revolution patch: r(h) along the axis, angles t0..t1 from xRef toward axis × xRef.</summary>
        public void AddRevolved(string name, Vec3 o, Vec3 axis, Vec3 xRef, Func<double, double> radius, double t0, double t1, double h0, double h1)
        {
            Vec3 z = axis.Normalized(), x = (xRef - z * xRef.Dot(z)).Normalized(), y = z.Cross(x);
            double[] cx, cy, w, ku;
            Nurbs.Arc(t0, t1, out cx, out cy, out w, out ku);
            var net = new Vec3[cx.Length, 2]; var wt = new double[cx.Length, 2];
            double[] hs = { h0, h1 };
            for (int i = 0; i < cx.Length; i++)
                for (int j = 0; j < 2; j++) { net[i, j] = o + z * hs[j] + (x * cx[i] + y * cy[i]) * radius(hs[j]); wt[i, j] = w[i]; }
            AddSurface(name, 2, 1, net, wt, ku, new double[] { 0, 0, 1, 1 }, t1 - t0 >= 2 * Math.PI - 1e-9, false);
        }

        /// <summary>Spherical cap of angular radius cap (rad) around pole.</summary>
        public void AddSphere(string name, Vec3 center, double r, Vec3 pole, double cap)
        {
            Vec3 z = pole.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
            double[] ax, ay, aw, ku, mx, my, mw, kv;
            Nurbs.Arc(0, 2 * Math.PI, out ax, out ay, out aw, out ku);
            Nurbs.Arc(0, Math.Min(Math.PI, cap), out mx, out my, out mw, out kv);   // meridian: (cos φ, sin φ) = (height, radius)
            var net = new Vec3[ax.Length, mx.Length]; var wt = new double[ax.Length, mx.Length];
            for (int i = 0; i < ax.Length; i++)
                for (int j = 0; j < mx.Length; j++)
                {
                    net[i, j] = center + (z * mx[j] + (x * ax[i] + y * ay[i]) * my[j]) * r;
                    wt[i, j] = aw[i] * mw[j];
                }
            AddSurface(name, 2, 2, net, wt, ku, kv, true, false);
        }

        /// <summary>Full circle as a rational B-spline curve.</summary>
        public void AddCircle(string name, Vec3 center, Vec3 normal, double r)
        {
            Vec3 z = normal.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
            double[] cx, cy, w, k;
            Nurbs.Arc(0, 2 * Math.PI, out cx, out cy, out w, out k);
            AddCurve(name, 2, cx.Select((c, i) => center + (x * c + y * cy[i]) * r).ToList(), w, k, true, z);
        }

        static IEnumerable<string> Chunks(string s, int width)
        {
            // break after a delimiter so no number is split
            var line = new StringBuilder();
            int i = 0;
            while (i < s.Length)
            {
                int j = i;
                while (j < s.Length && s[j] != ',' && s[j] != ';') j++;
                string tok = s.Substring(i, Math.Min(s.Length, j + 1) - i);
                if (line.Length + tok.Length > width && line.Length > 0) { yield return line.ToString(); line.Clear(); }
                while (tok.Length > width) { yield return tok.Substring(0, width); tok = tok.Substring(width); }
                line.Append(tok);
                i = j + 1;
            }
            if (line.Length > 0) yield return line.ToString();
        }

        static string H(string s) { s = s ?? ""; return s.Length.ToString(CI) + "H" + s; }

        public void Write(string path, string author, string organization, string software)
        {
            var sb = new StringBuilder();
            int seq = 0;
            Action<string, char> line = (data, sec) => { seq++; sb.Append(data.PadRight(72).Substring(0, 72)).Append(sec).Append(seq.ToString("D7", CI)).Append("\r\n"); };
            // start
            line("MicroScribe Inspector: measured surfaces, curves and points (millimetres)", 'S');
            int nS = seq; seq = 0;
            // global
            string now = DateTime.Now.ToString("yyyyMMdd.HHmmss", CI);
            string g = string.Join(",", new[]
            {
                "1H,", "1H;", H(Path.GetFileNameWithoutExtension(path)), H(Path.GetFileName(path)), H(software), H(software), "32", "38", "6", "308", "15",
                H(Path.GetFileNameWithoutExtension(path)), "1.0", "2", "2HMM", "1", "1.0", H(now), "0.000001", N(_maxCoord), H(author), H(organization), "11", "0", H(now)
            }) + ";";
            foreach (var c in Chunks(g, 72)) line(c, 'G');
            int nG = seq; seq = 0;
            // parameter data first (to know each entity's first line and line count)
            var pLines = new List<KeyValuePair<int, string>>();   // (entity index, data)
            var firstP = new int[_ents.Count]; var countP = new int[_ents.Count];
            for (int e = 0; e < _ents.Count; e++)
            {
                firstP[e] = pLines.Count + 1;
                foreach (var c in Chunks(_ents[e].Params, 64)) pLines.Add(new KeyValuePair<int, string>(e, c));
                countP[e] = pLines.Count + 1 - firstP[e];
            }
            Func<object, string> F = v => v.ToString().PadLeft(8);
            for (int e = 0; e < _ents.Count; e++)
            {
                var en = _ents[e];
                line(F(en.Type) + F(firstP[e]) + F(0) + F(0) + F(0) + F(0) + F(0) + F(0) + "00000000", 'D');
                line(F(en.Type) + F(0) + F(0) + F(countP[e]) + F(en.Form) + F("") + F("") + F(en.Label) + F(0), 'D');
            }
            int nD = seq; seq = 0;
            foreach (var pl in pLines)
            {
                seq++;
                sb.Append(pl.Value.PadRight(64)).Append(' ').Append((2 * pl.Key + 1).ToString(CI).PadLeft(7)).Append('P').Append(seq.ToString("D7", CI)).Append("\r\n");
            }
            int nP = seq; seq = 0;
            line(string.Format(CI, "S{0:D7}G{1:D7}D{2:D7}P{3:D7}", nS, nG, nD, nP), 'T');
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }
    }
}
