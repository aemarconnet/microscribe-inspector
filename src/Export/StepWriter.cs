using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MicroScribe.Core;

namespace MicroScribe.Export
{
    /// <summary>
    /// Minimal ISO 10303-21 (STEP AP214) writer for analytic surface patches and wireframe.
    /// Surfaces are written as SHELL_BASED_SURFACE_MODELs (one per feature) of ADVANCED_FACEs
    /// on PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE / SPHERICAL_SURFACE, which SolidWorks,
    /// Fusion, Onshape, Inventor, FreeCAD and Rhino import as surface bodies.
    /// </summary>
    public sealed class StepWriter
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        readonly List<string> _ents = new List<string>();
        readonly List<int> _shellModels = new List<int>();
        readonly List<int> _wireItems = new List<int>();
        readonly Dictionary<string, int> _dirCache = new Dictionary<string, int>();
        int _ctx;

        public string Author = "";
        public string Organization = "";

        int Add(string body)
        {
            _ents.Add(body);
            return _ents.Count; // ids are 1-based
        }

        public static string R(double v)
        {
            if (double.IsNaN(v) || double.IsInfinity(v)) v = 0;
            if (Math.Abs(v) < 1e-14) return "0.";
            string s = v.ToString("0.0###############", CI);
            if (s.Length > 24) s = v.ToString("0.0##############E+0", CI);
            return s;
        }

        static string S(string s)
        {
            if (s == null) return "''";
            var sb = new StringBuilder("'");
            foreach (char c in s)
            {
                if (c == '\'') sb.Append("''");
                else if (c == '\\') sb.Append("\\\\");
                else if (c >= 32 && c < 127) sb.Append(c);
                else sb.Append('_');
            }
            return sb.Append('\'').ToString();
        }

        string Ref(int id) { return "#" + id; }

        public int Point(Vec3 p) { return Add(string.Format("CARTESIAN_POINT('',({0},{1},{2}))", R(p.X), R(p.Y), R(p.Z))); }

        public int Direction(Vec3 d)
        {
            d = d.Normalized();
            string key = R(d.X) + "," + R(d.Y) + "," + R(d.Z);
            int id;
            if (_dirCache.TryGetValue(key, out id)) return id;
            id = Add("DIRECTION('',(" + key + "))");
            _dirCache[key] = id;
            return id;
        }

        public int Placement(Vec3 origin, Vec3 axis, Vec3 refDir)
        {
            // make refDir exactly perpendicular to axis
            Vec3 a = axis.Normalized();
            Vec3 r = refDir - a * refDir.Dot(a);
            if (r.Length < 1e-9) r = a.AnyPerpendicular();
            return Add(string.Format("AXIS2_PLACEMENT_3D('',{0},{1},{2})", Ref(Point(origin)), Ref(Direction(a)), Ref(Direction(r))));
        }

        int Vertex(Vec3 p) { return Add("VERTEX_POINT(''," + Ref(Point(p)) + ")"); }

        int LineCurve(Vec3 p, Vec3 q)
        {
            Vec3 d = q - p;
            double len = Math.Max(d.Length, 1e-9);
            int vec = Add(string.Format("VECTOR('',{0},{1})", Ref(Direction(d)), R(len)));
            return Add(string.Format("LINE('',{0},{1})", Ref(Point(p)), Ref(vec)));
        }

        int CircleCurve(Vec3 center, Vec3 axis, Vec3 refDir, double r)
        {
            return Add(string.Format("CIRCLE('',{0},{1})", Ref(Placement(center, axis, refDir)), R(r)));
        }

        int Edge(int v1, int v2, int curve, bool sameSense = true)
        {
            return Add(string.Format("EDGE_CURVE('',{0},{1},{2},{3})", Ref(v1), Ref(v2), Ref(curve), sameSense ? ".T." : ".F."));
        }

        /// <summary>Oriented edge reference: positive = forward, negative = reversed (edge id is the absolute value).</summary>
        static int Fwd(int edge) { return edge; }
        static int Rev(int edge) { return -edge; }

        /// <summary>
        /// Face whose loops are given in the surface's natural (counter-clockwise about the surface normal)
        /// orientation. When sameSense is false the face normal is flipped and the loops are reversed.
        /// </summary>
        int Face(int surface, bool sameSense, params int[][] loops)
        {
            var bounds = new List<string>();
            for (int i = 0; i < loops.Length; i++)
            {
                var seq = new List<int>(loops[i]);
                if (!sameSense) { seq.Reverse(); for (int k = 0; k < seq.Count; k++) seq[k] = -seq[k]; }
                var oes = new List<string>();
                foreach (int e in seq)
                    oes.Add(Ref(Add(string.Format("ORIENTED_EDGE('',*,*,{0},{1})", Ref(Math.Abs(e)), e > 0 ? ".T." : ".F."))));
                int loop = Add("EDGE_LOOP('',(" + string.Join(",", oes) + "))");
                bounds.Add(Ref(Add(string.Format("{0}('',{1},.T.)", i == 0 ? "FACE_OUTER_BOUND" : "FACE_BOUND", Ref(loop)))));
            }
            return Add(string.Format("ADVANCED_FACE('',({0}),{1},{2})", string.Join(",", bounds), Ref(surface), sameSense ? ".T." : ".F."));
        }

        void Shell(string name, List<int> faces)
        {
            var fs = new List<string>();
            foreach (int f in faces) fs.Add(Ref(f));
            int shell = Add(string.Format("OPEN_SHELL({0},({1}))", S(name), string.Join(",", fs)));
            _shellModels.Add(Add(string.Format("SHELL_BASED_SURFACE_MODEL({0},({1}))", S(name), Ref(shell))));
        }

        // ------------------------------------------------------------------ surfaces

        /// <summary>Planar face bounded by a polygon (counter-clockwise about normal).</summary>
        public void AddPlane(string name, Vec3 origin, Vec3 normal, IList<Vec3> polygon)
        {
            int n = polygon.Count;
            if (n < 3) return;
            Vec3 u = (polygon[1] - polygon[0]).Normalized();
            int surf = Add("PLANE(''," + Ref(Placement(origin, normal, u)) + ")");
            // ensure CCW about normal
            var poly = new List<Vec3>(polygon);
            Vec3 area = Vec3.Zero;
            for (int i = 0; i < n; i++) area += poly[i].Cross(poly[(i + 1) % n]);
            if (area.Dot(normal) < 0) poly.Reverse();
            var verts = new int[n];
            for (int i = 0; i < n; i++) verts[i] = Vertex(poly[i]);
            var oes = new int[n];
            for (int i = 0; i < n; i++)
                oes[i] = Fwd(Edge(verts[i], verts[(i + 1) % n], LineCurve(poly[i], poly[(i + 1) % n])));
            Shell(name, new List<int> { Face(surf, true, oes) });
        }

        /// <summary>
        /// Generic surface of revolution patch: radius r(h) linear in h (cylinder, cone),
        /// angle range [t0, t1] (t1 - t0 ≤ 2π), height range [h0, h1].
        /// Full revolutions are split into two half faces sharing their seam edges.
        /// </summary>
        void AddRevolved(string name, int surface, Vec3 o, Vec3 axis, Vec3 xRef, Func<double, double> radius,
                         double t0, double t1, double h0, double h1, bool reversed)
        {
            Vec3 z = axis.Normalized();
            Vec3 x = (xRef - z * xRef.Dot(z)).Normalized();
            Vec3 y = z.Cross(x);
            bool full = t1 - t0 >= 2 * Math.PI - 1e-9;
            int segs = full ? 2 : (t1 - t0 > Math.PI ? 2 : 1);
            double r0 = radius(h0), r1 = radius(h1);
            Func<double, double, Vec3> P = (t, h) => o + z * h + (x * Math.Cos(t) + y * Math.Sin(t)) * radius(h);

            var angles = new double[segs + 1];
            for (int i = 0; i <= segs; i++) angles[i] = t0 + (t1 - t0) * i / segs;
            int nv = full ? segs : segs + 1;
            var vb = new int[nv]; var vt = new int[nv];
            for (int i = 0; i < nv; i++)
            {
                vb[i] = r0 > 1e-9 ? Vertex(P(angles[i], h0)) : (i == 0 ? Vertex(P(0, h0)) : vb[0]);
                vt[i] = r1 > 1e-9 ? Vertex(P(angles[i], h1)) : (i == 0 ? Vertex(P(0, h1)) : vt[0]);
            }
            Func<int, int> VI = i => full ? i % segs : i;

            // circles (one per ring, shared by all segments)
            int cb = r0 > 1e-9 ? CircleCurve(o + z * h0, z, x, r0) : 0;
            int ct = r1 > 1e-9 ? CircleCurve(o + z * h1, z, x, r1) : 0;
            // generator lines
            var gen = new int[nv];
            for (int i = 0; i < nv; i++)
                gen[i] = Edge(vb[i], vt[i], LineCurve(P(angles[i], h0), P(angles[i], h1)));

            var faces = new List<int>();
            for (int s = 0; s < segs; s++)
            {
                int a = VI(s), b = VI(s + 1);
                var loop = new List<int>();
                if (cb != 0) loop.Add(Fwd(Edge(vb[a], vb[b], cb)));   // bottom, +t
                loop.Add(Fwd(gen[b]));                                  // up at t(s+1)
                if (ct != 0) loop.Add(Rev(Edge(vt[a], vt[b], ct)));   // top, -t
                loop.Add(Rev(gen[a]));                                  // down at t(s)
                faces.Add(Face(surface, !reversed, loop.ToArray()));
            }
            Shell(name, faces);
        }

        /// <summary>reversed = face normal points toward the axis (bores / internal features).</summary>
        public void AddCylinder(string name, Vec3 axisPoint, Vec3 axis, Vec3 xRef, double radius,
                                double t0, double t1, double h0, double h1, bool reversed = false)
        {
            Vec3 z = axis.Normalized();
            int surf = Add(string.Format("CYLINDRICAL_SURFACE('',{0},{1})", Ref(Placement(axisPoint, z, xRef)), R(radius)));
            AddRevolved(name, surf, axisPoint, z, xRef, h => radius, t0, t1, h0, h1, reversed);
        }

        /// <summary>Cone with radius rAt0 at axisPoint, opening along +axis with halfAngle (rad, &gt; 0).</summary>
        public void AddCone(string name, Vec3 axisPoint, Vec3 axis, Vec3 xRef, double rAt0, double halfAngle,
                            double t0, double t1, double h0, double h1, bool reversed = false)
        {
            Vec3 z = axis.Normalized();
            double tan = Math.Tan(halfAngle);
            // keep the placement at h0 so the STEP radius is non-negative
            double hApex = -rAt0 / tan;
            if (h0 < hApex) h0 = hApex;
            Vec3 loc = axisPoint + z * h0;
            double rLoc = Math.Max(0, rAt0 + h0 * tan);
            int surf = Add(string.Format("CONICAL_SURFACE('',{0},{1},{2})", Ref(Placement(loc, z, xRef)), R(rLoc), R(halfAngle)));
            AddRevolved(name, surf, axisPoint, z, xRef, h => Math.Max(0, rAt0 + h * tan), t0, t1, h0, h1, reversed);
        }

        /// <summary>
        /// Spherical cap around 'pole' with angular radius capAngle (rad). capAngle ≥ ~179° gives a full sphere
        /// (two hemispheres sharing the equator).
        /// </summary>
        public void AddSphere(string name, Vec3 center, double radius, Vec3 pole, double capAngle, bool reversed = false)
        {
            Vec3 z = pole.Normalized(), x = z.AnyPerpendicular();
            int surf = Add(string.Format("SPHERICAL_SURFACE('',{0},{1})", Ref(Placement(center, z, x)), R(radius)));
            var faces = new List<int>();
            if (capAngle >= Math.PI - 1e-3)
            {
                Vec3 p = center + x * radius;
                int v = Vertex(p);
                int c = CircleCurve(center, z, x, radius);
                int e = Edge(v, v, c);
                faces.Add(Face(surf, !reversed, new[] { Fwd(e) }));  // northern hemisphere
                faces.Add(Face(surf, !reversed, new[] { Rev(e) }));  // southern hemisphere
            }
            else
            {
                double lat = Math.PI / 2 - capAngle; // latitude of the boundary circle
                Vec3 cc = center + z * (radius * Math.Sin(lat));
                double rc = radius * Math.Cos(lat);
                Vec3 p = cc + x * rc;
                int v = Vertex(p);
                int e = Edge(v, v, CircleCurve(cc, z, x, rc));
                faces.Add(Face(surf, !reversed, new[] { Fwd(e) }));
            }
            Shell(name, faces);
        }

        /// <summary>
        /// Freeform B-spline surface patch (non-rational, clamped knots). ctrl[i, j]: i along u, j along v.
        /// Bounded by its four iso-boundary curves.
        /// </summary>
        public void AddBSplineSurface(string name, int degU, int degV, Vec3[,] ctrl, double[] knotsU, double[] knotsV)
        {
            int nu = ctrl.GetLength(0), nv = ctrl.GetLength(1);
            var rows = new List<string>();
            for (int i = 0; i < nu; i++)
            {
                var ids = new List<string>();
                for (int j = 0; j < nv; j++) ids.Add(Ref(Point(ctrl[i, j])));
                rows.Add("(" + string.Join(",", ids) + ")");
            }
            string mu, ku, mv, kv;
            KnotLists(knotsU, out mu, out ku);
            KnotLists(knotsV, out mv, out kv);
            int surf = Add(string.Format("B_SPLINE_SURFACE_WITH_KNOTS('',{0},{1},({2}),.UNSPECIFIED.,.F.,.F.,.F.,({3}),({4}),({5}),({6}),.UNSPECIFIED.)",
                degU, degV, string.Join(",", rows), mu, mv, ku, kv));
            // boundary curves (iso lines of the control net share the surface knots)
            Func<IList<Vec3>, int, double[], int> bcurve = (pts, deg, knots) =>
            {
                var ids = pts.Select(p => Ref(Point(p))).ToList();
                string m, k; KnotLists(knots, out m, out k);
                return Add(string.Format("B_SPLINE_CURVE_WITH_KNOTS('',{0},({1}),.UNSPECIFIED.,.F.,.F.,({2}),({3}),.UNSPECIFIED.)", deg, string.Join(",", ids), m, k));
            };
            var v0 = Enumerable.Range(0, nu).Select(i => ctrl[i, 0]).ToList();
            var v1 = Enumerable.Range(0, nu).Select(i => ctrl[i, nv - 1]).ToList();
            var u0 = Enumerable.Range(0, nv).Select(j => ctrl[0, j]).ToList();
            var u1 = Enumerable.Range(0, nv).Select(j => ctrl[nu - 1, j]).ToList();
            int p00 = Vertex(ctrl[0, 0]), p10 = Vertex(ctrl[nu - 1, 0]), p11 = Vertex(ctrl[nu - 1, nv - 1]), p01 = Vertex(ctrl[0, nv - 1]);
            int eV0 = Edge(p00, p10, bcurve(v0, degU, knotsU));   // v = 0, u increasing
            int eU1 = Edge(p10, p11, bcurve(u1, degV, knotsV));   // u = 1, v increasing
            int eV1 = Edge(p01, p11, bcurve(v1, degU, knotsU));   // v = 1, traversed backwards
            int eU0 = Edge(p00, p01, bcurve(u0, degV, knotsV));   // u = 0, traversed backwards
            Shell(name, new List<int> { Face(surf, true, new[] { Fwd(eV0), Fwd(eU1), Rev(eV1), Rev(eU0) }) });
        }

        static void KnotLists(IList<double> knots, out string mults, out string values)
        {
            var kv = new List<double>(); var mult = new List<int>();
            foreach (double k in knots)
            {
                if (kv.Count > 0 && Math.Abs(kv[kv.Count - 1] - k) < 1e-12) mult[mult.Count - 1]++;
                else { kv.Add(k); mult.Add(1); }
            }
            mults = string.Join(",", mult.Select(m => m.ToString(CI)));
            values = string.Join(",", kv.Select(R));
        }

        // ------------------------------------------------------------------ wireframe

        public void AddPoint(string name, Vec3 p)
        {
            _wireItems.Add(Add(string.Format("CARTESIAN_POINT({0},({1},{2},{3}))", S(name), R(p.X), R(p.Y), R(p.Z))));
        }

        public void AddLine(string name, Vec3 a, Vec3 b)
        {
            int line = LineCurve(a, b);
            int tc = Add(string.Format("TRIMMED_CURVE({0},{1},({2}),({3}),.T.,.CARTESIAN.)",
                S(name), Ref(line), Ref(Point(a)), Ref(Point(b))));
            _wireItems.Add(tc);
        }

        public void AddCircle(string name, Vec3 center, Vec3 normal, double radius)
        {
            int c = Add(string.Format("CIRCLE({0},{1},{2})", S(name), Ref(Placement(center, normal, normal.AnyPerpendicular())), R(radius)));
            _wireItems.Add(c);
        }

        public void AddPolyline(string name, IList<Vec3> pts)
        {
            if (pts.Count < 2) return;
            var ids = new List<string>();
            foreach (var p in pts) ids.Add(Ref(Point(p)));
            _wireItems.Add(Add(string.Format("POLYLINE({0},({1}))", S(name), string.Join(",", ids))));
        }

        /// <summary>Non-rational B-spline curve (control points + clamped knot vector).</summary>
        public void AddBSpline(string name, int degree, IList<Vec3> ctrl, IList<double> knots)
        {
            // compress knots into distinct values + multiplicities
            var kv = new List<double>(); var mult = new List<int>();
            foreach (double k in knots)
            {
                if (kv.Count > 0 && Math.Abs(kv[kv.Count - 1] - k) < 1e-12) mult[mult.Count - 1]++;
                else { kv.Add(k); mult.Add(1); }
            }
            var ids = new List<string>();
            foreach (var p in ctrl) ids.Add(Ref(Point(p)));
            var ms = new List<string>(); foreach (int m in mult) ms.Add(m.ToString(CI));
            var ks = new List<string>(); foreach (double k in kv) ks.Add(R(k));
            _wireItems.Add(Add(string.Format("B_SPLINE_CURVE_WITH_KNOTS({0},{1},({2}),.UNSPECIFIED.,.F.,.F.,({3}),({4}),.UNSPECIFIED.)",
                S(name), degree, string.Join(",", ids), string.Join(",", ms), string.Join(",", ks))));
        }

        // ------------------------------------------------------------------ file

        public void Write(string path, string productName)
        {
            var body = new List<string>(_ents);
            int baseId = body.Count;
            Func<string, int> A = s => { body.Add(s); return body.Count; };

            int appCtx = A("APPLICATION_CONTEXT('automotive design')");
            A(string.Format("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{0})", appCtx));
            int prodCtx = A(string.Format("PRODUCT_CONTEXT('',#{0},'mechanical')", appCtx));
            int prod = A(string.Format("PRODUCT({0},{0},'',(#{1}))", S(productName), prodCtx));
            A(string.Format("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{0}))", prod));
            int pdf = A(string.Format("PRODUCT_DEFINITION_FORMATION('','',#{0})", prod));
            int pdCtx = A(string.Format("PRODUCT_DEFINITION_CONTEXT('part definition',#{0},'design')", appCtx));
            int pd = A(string.Format("PRODUCT_DEFINITION('design','',#{0},#{1})", pdf, pdCtx));
            int pds = A(string.Format("PRODUCT_DEFINITION_SHAPE('','',#{0})", pd));

            int lenU = A("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
            int angU = A("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
            int solU = A("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
            int unc = A(string.Format("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#{0},'distance_accuracy_value','confusion accuracy')", lenU));
            _ctx = A(string.Format("(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{0}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#{1},#{2},#{3}))REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))",
                unc, lenU, angU, solU));

            int o = A("CARTESIAN_POINT('',(0.,0.,0.))");
            int dz = A("DIRECTION('',(0.,0.,1.))");
            int dx = A("DIRECTION('',(1.,0.,0.))");
            int ax = A(string.Format("AXIS2_PLACEMENT_3D('',#{0},#{1},#{2})", o, dz, dx));

            int mainRep = A(string.Format("SHAPE_REPRESENTATION({0},(#{1}),#{2})", S(productName), ax, _ctx));
            A(string.Format("SHAPE_DEFINITION_REPRESENTATION(#{0},#{1})", pds, mainRep));

            if (_shellModels.Count > 0)
            {
                var items = new List<string> { "#" + ax };
                foreach (int s in _shellModels) items.Add("#" + s);
                int mssr = A(string.Format("MANIFOLD_SURFACE_SHAPE_REPRESENTATION('surfaces',({0}),#{1})", string.Join(",", items), _ctx));
                A(string.Format("SHAPE_REPRESENTATION_RELATIONSHIP('','',#{0},#{1})", mainRep, mssr));
            }
            if (_wireItems.Count > 0)
            {
                var items = new List<string>();
                foreach (int w in _wireItems) items.Add("#" + w);
                int gcs = A(string.Format("GEOMETRIC_CURVE_SET('reference geometry',({0}))", string.Join(",", items)));
                int gbw = A(string.Format("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('wireframe',(#{0},#{1}),#{2})", ax, gcs, _ctx));
                A(string.Format("SHAPE_REPRESENTATION_RELATIONSHIP('','',#{0},#{1})", mainRep, gbw));
            }

            var sb = new StringBuilder();
            sb.Append("ISO-10303-21;\nHEADER;\n");
            sb.Append("FILE_DESCRIPTION(('MicroScribe Inspector measured geometry'),'2;1');\n");
            sb.AppendFormat("FILE_NAME({0},'{1:yyyy-MM-ddTHH:mm:ss}',({2}),({3}),'MicroScribe Inspector','MicroScribe Inspector','');\n",
                S(Path.GetFileName(path)), DateTime.Now, S(Author), S(Organization));
            sb.Append("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
            for (int i = 0; i < body.Count; i++) sb.AppendFormat("#{0}={1};\n", i + 1, body[i]);
            sb.Append("ENDSEC;\nEND-ISO-10303-21;\n");
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        public int EntityCount { get { return _ents.Count; } }
    }
}
