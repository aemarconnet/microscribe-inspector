using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MicroScribe.Core;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.Export
{
    public sealed class ExportOptions
    {
        public bool Surfaces = true;
        public bool ReferenceGeometry = true;  // axes, centres, circles, lines, points
        public bool MeasuredPoints = false;    // every probed point as a STEP point
        public bool UsePartAlignment = true;
        public string Author = "";
    }

    /// <summary>Length formatting for reports (values are mm internally).</summary>
    public sealed class UnitFmt
    {
        public readonly bool Inch;
        public UnitFmt(bool inch) { Inch = inch; }
        public string U { get { return Inch ? "in" : "mm"; } }
        public double S(double mm) { return Inch ? mm / 25.4 : mm; }
        public string L(double mm) { return Inch ? (mm / 25.4).ToString("0.0000", CultureInfo.InvariantCulture) : mm.ToString("0.000", CultureInfo.InvariantCulture); }
        public static readonly UnitFmt Mm = new UnitFmt(false);
    }

    /// <summary>Builds CAD output (STEP, CSV, XYZ, report) from a project's solved features.</summary>
    public static class CadExport
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        /// <summary>Extent of a revolved feature computed from its points (in the given frame).</summary>
        public static void RevolvedExtent(Vec3 axisPoint, Vec3 axis, IEnumerable<Vec3> pts, double margin, bool forceFull,
            out Vec3 xRef, out double t0, out double t1, out double h0, out double h1, double radius)
        {
            Vec3 z = axis.Normalized();
            xRef = z.AnyPerpendicular();
            Vec3 y = z.Cross(xRef);
            var angles = new List<double>();
            h0 = double.MaxValue; h1 = double.MinValue;
            foreach (var p in pts)
            {
                Vec3 w = p - axisPoint;
                double h = w.Dot(z);
                h0 = Math.Min(h0, h); h1 = Math.Max(h1, h);
                angles.Add(Math.Atan2(w.Dot(y), w.Dot(xRef)));
            }
            if (angles.Count == 0) { t0 = 0; t1 = 2 * Math.PI; h0 = -10; h1 = 10; return; }
            bool full;
            GeoUtil.AngularRange(angles, 70 * Math.PI / 180, out t0, out t1, out full);
            if (forceFull) full = true;
            if (full) { t0 = 0; t1 = 2 * Math.PI; }
            else
            {
                double dm = radius > 1e-6 ? Math.Min(margin / radius, 0.5) : 0.1;
                t0 -= dm; t1 += dm;
                if (t1 - t0 >= 2 * Math.PI - 1e-6) { t0 = 0; t1 = 2 * Math.PI; }
            }
            h0 -= margin; h1 += margin;
            if (h1 - h0 < 1e-3) { h0 -= 0.5; h1 += 0.5; }
        }

        static List<Vec3> ActivePositions(Feature f, Mat4 m)
        {
            return f.ActivePoints.Select(p => m.TransformPoint(p.Position)).ToList();
        }

        /// <summary>Writes a STEP AP214 file (millimetres). Returns a short summary.</summary>
        public static string WriteStep(Project proj, string path, ExportOptions opt)
        {
            Mat4 m = opt.UsePartAlignment ? proj.AlignmentMatrix : Mat4.Identity();
            var w = new StepWriter { Author = opt.Author };
            int surfaces = 0, wires = 0;
            foreach (var f in proj.Features.Where(x => x.Export))
            {
                var g = GeoUtil.Transform(f.Geometry, m);
                var pts = ActivePositions(f, m);
                if (g == null) continue;
                double margin = f.ExportMargin;
                if (g is CurvePrim)
                {
                    var c = (CurvePrim)g;
                    if (c.Degree <= 1) w.AddPolyline(f.Name, c.Ctrl);
                    else w.AddBSpline(f.Name, c.Degree, c.Ctrl, c.Knots);
                    wires++;
                }
                else if (g is ProfilePrim)
                {
                    var pp = (ProfilePrim)g;
                    w.AddPolyline(f.Name, pp.Outline());
                    w.AddPoint(f.Name + " center", pp.Center);
                    if (opt.ReferenceGeometry) w.AddLine(f.Name + " axis", pp.Center - pp.XDir * (pp.Length / 2), pp.Center + pp.XDir * (pp.Length / 2));
                    wires += 3;
                }
                else if (g is CloudPrim)
                {
                    int k = 1;
                    foreach (var q in ((CloudPrim)g).Points) { w.AddPoint(f.Name + " " + (k++), q); wires++; }
                    continue;
                }
                else if (g is SurfacePrim && opt.Surfaces)
                {
                    var s = (SurfacePrim)g;
                    w.AddBSplineSurface(f.Name, s.DegU, s.DegV, s.Ctrl, s.KnotsU, s.KnotsV);
                    surfaces++;
                }
                else if (g is PlanePrim && opt.Surfaces)
                {
                    var pl = (PlanePrim)g;
                    var corners = PlaneBounds.Corners(f, (PlanePrim)f.Geometry).Select(m.TransformPoint).ToList();
                    w.AddPlane(f.Name, pl.Origin, pl.Normal, corners);
                    surfaces++;
                }
                else if (g is CylinderPrim && opt.Surfaces)
                {
                    var c = (CylinderPrim)g;
                    Vec3 xr; double t0, t1, h0, h1;
                    RevolvedExtent(c.AxisPoint, c.Axis, pts, margin, f.FullRevolution, out xr, out t0, out t1, out h0, out h1, c.Radius);
                    w.AddCylinder(f.Name, c.AxisPoint, c.Axis, xr, c.Radius, t0, t1, h0, h1, IsInternal(f));
                    surfaces++;
                    if (opt.ReferenceGeometry) { w.AddLine(f.Name + " axis", c.AxisPoint + c.Axis * h0, c.AxisPoint + c.Axis * h1); wires++; }
                }
                else if (g is ConePrim && opt.Surfaces)
                {
                    var c = (ConePrim)g;
                    Vec3 xr; double t0, t1, h0, h1;
                    RevolvedExtent(c.AxisPoint, c.Axis, pts, margin, f.FullRevolution, out xr, out t0, out t1, out h0, out h1, c.RadiusAtOrigin);
                    w.AddCone(f.Name, c.AxisPoint, c.Axis, xr, c.RadiusAtOrigin, c.HalfAngle, t0, t1, h0, h1, IsInternal(f));
                    surfaces++;
                    if (opt.ReferenceGeometry)
                    {
                        double hApex = -c.RadiusAtOrigin / Math.Tan(c.HalfAngle);
                        w.AddLine(f.Name + " axis", c.AxisPoint + c.Axis * Math.Max(h0, hApex), c.AxisPoint + c.Axis * h1);
                        if (c.HalfAngle > 1e-3) w.AddPoint(f.Name + " apex", c.Apex);
                        wires += 2;
                    }
                }
                else if (g is SpherePrim && opt.Surfaces)
                {
                    var s = (SpherePrim)g;
                    Vec3 pole; double cap;
                    SphereCap(s, pts, margin, out pole, out cap);
                    w.AddSphere(f.Name, s.Center, s.Radius, pole, cap, IsInternal(f));
                    surfaces++;
                    if (opt.ReferenceGeometry) { w.AddPoint(f.Name + " center", s.Center); wires++; }
                }
                if (opt.ReferenceGeometry)
                {
                    if (g is PointPrim) { w.AddPoint(f.Name, ((PointPrim)g).Position); wires++; }
                    else if (g is LinePrim)
                    {
                        var l = (LinePrim)g;
                        double a = l.TMin - margin, b = l.TMax + margin;
                        w.AddLine(f.Name, l.Origin + l.Direction * a, l.Origin + l.Direction * b); wires++;
                    }
                    else if (g is CirclePrim)
                    {
                        var c = (CirclePrim)g;
                        w.AddCircle(f.Name, c.Center, c.Normal, c.Radius);
                        w.AddPoint(f.Name + " center", c.Center);
                        wires += 2;
                    }
                }
                if (opt.MeasuredPoints)
                {
                    int k = 1;
                    foreach (var p in FeatureSolver.ContactPoints(f, f.ActivePoints.ToList())) { w.AddPoint(f.Name + " pt" + (k++), m.TransformPoint(p)); wires++; }
                }
            }
            w.Write(path, string.IsNullOrEmpty(proj.Title) ? "MicroScribe" : proj.Title);
            return string.Format("{0} surface bodies, {1} reference items", surfaces, wires);
        }

        /// <summary>Writes an IGES file (mm): surfaces as exact B-spline surfaces, curves, axes, centres, points.</summary>
        public static string WriteIges(Project proj, string path, ExportOptions opt, string software)
        {
            Mat4 m = opt.UsePartAlignment ? proj.AlignmentMatrix : Mat4.Identity();
            var w = new IgesWriter();
            int surfaces = 0;
            foreach (var f in proj.Features.Where(x => x.Export))
            {
                var g = GeoUtil.Transform(f.Geometry, m);
                if (g == null) continue;
                var pts = ActivePositions(f, m);
                double margin = f.ExportMargin;
                if (g is CurvePrim)
                {
                    var c = (CurvePrim)g;
                    w.AddCurve(f.Name, Math.Max(1, c.Degree), c.Ctrl, null, c.Knots, c.Closed);
                }
                else if (g is SurfacePrim && opt.Surfaces)
                {
                    var s = (SurfacePrim)g;
                    w.AddSurface(f.Name, s.DegU, s.DegV, s.Ctrl, null, s.KnotsU, s.KnotsV);
                    surfaces++;
                }
                else if (g is PlanePrim && opt.Surfaces)
                {
                    var c = PlaneBounds.Corners(f, (PlanePrim)f.Geometry).Select(m.TransformPoint).ToArray();
                    var net = new Vec3[2, 2]; net[0, 0] = c[0]; net[1, 0] = c[1]; net[0, 1] = c[3]; net[1, 1] = c[2];
                    w.AddSurface(f.Name, 1, 1, net, null, new double[] { 0, 0, 1, 1 }, new double[] { 0, 0, 1, 1 });
                    surfaces++;
                }
                else if ((g is CylinderPrim || g is ConePrim) && opt.Surfaces)
                {
                    var cy = g as CylinderPrim; var co = g as ConePrim;
                    Vec3 o = cy != null ? cy.AxisPoint : co.AxisPoint, ax = cy != null ? cy.Axis : co.Axis;
                    Vec3 xr; double t0, t1, h0, h1;
                    RevolvedExtent(o, ax, pts, margin, f.FullRevolution, out xr, out t0, out t1, out h0, out h1, cy != null ? cy.Radius : co.RadiusAtOrigin);
                    if (co != null)
                    {
                        // stop at the apex
                        double hApex = Math.Abs(Math.Tan(co.HalfAngle)) > 1e-9 ? -co.RadiusAtOrigin / Math.Tan(co.HalfAngle) : double.NaN;
                        if (!double.IsNaN(hApex)) { if (hApex < (h0 + h1) / 2) h0 = Math.Max(h0, hApex); else h1 = Math.Min(h1, hApex); }
                    }
                    Func<double, double> rad = h => cy != null ? cy.Radius : Math.Max(0, co.RadiusAt(h));
                    w.AddRevolved(f.Name, o, ax, xr, rad, t0, t1, h0, h1);
                    surfaces++;
                    if (opt.ReferenceGeometry) w.AddLine(f.Name + " axis", o + ax * h0, o + ax * h1);
                }
                else if (g is SpherePrim && opt.Surfaces)
                {
                    var s = (SpherePrim)g;
                    Vec3 pole; double cap;
                    SphereCap(s, pts, margin, out pole, out cap);
                    w.AddSphere(f.Name, s.Center, s.Radius, pole, cap);
                    surfaces++;
                    if (opt.ReferenceGeometry) w.AddPoint(f.Name + " center", s.Center);
                }
                else if (g is ProfilePrim)
                {
                    var pp = (ProfilePrim)g;
                    var outline = pp.Outline();
                    w.AddCurve(f.Name, 1, outline, null, Spline.ClampedUniform(outline.Count, 1, CurvePrim.ChordParams(outline)), true, pp.Normal);
                    w.AddPoint(f.Name + " center", pp.Center);
                }
                else if (g is CloudPrim) foreach (var q in ((CloudPrim)g).Points) w.AddPoint(f.Name, q);
                if (opt.ReferenceGeometry)
                {
                    if (g is PointPrim) w.AddPoint(f.Name, ((PointPrim)g).Position);
                    else if (g is LinePrim) { var l = (LinePrim)g; w.AddLine(f.Name, l.Origin + l.Direction * (l.TMin - margin), l.Origin + l.Direction * (l.TMax + margin)); }
                    else if (g is CirclePrim) { var c = (CirclePrim)g; w.AddCircle(f.Name, c.Center, c.Normal, c.Radius); w.AddPoint(f.Name + " center", c.Center); }
                }
                if (opt.MeasuredPoints && !(g is CloudPrim))
                    foreach (var q in FeatureSolver.ContactPoints(f, f.ActivePoints.ToList())) w.AddPoint(f.Name, m.TransformPoint(q));
            }
            w.Write(path, opt.Author, "", software);
            return string.Format("{0} surfaces, {1} entities", surfaces, w.Count);
        }

        /// <summary>Binary STL of all exported surfaces (mm, part frame unless told otherwise). Returns the triangle count.</summary>
        public static int WriteStl(Project proj, string path, ExportOptions opt)
        {
            Mat4 m = opt.UsePartAlignment ? proj.AlignmentMatrix : Mat4.Identity();
            var tri = new List<Vec3>();
            Action<Func<double, double, Vec3>, int, int, bool> grid = (S, nu, nv, flip) =>
            {
                for (int i = 0; i < nu; i++)
                    for (int j = 0; j < nv; j++)
                    {
                        Vec3 a = S((double)i / nu, (double)j / nv), b = S((double)(i + 1) / nu, (double)j / nv), c = S((double)(i + 1) / nu, (double)(j + 1) / nv), d = S((double)i / nu, (double)(j + 1) / nv);
                        if (flip) { tri.Add(a); tri.Add(c); tri.Add(b); tri.Add(a); tri.Add(d); tri.Add(c); }
                        else { tri.Add(a); tri.Add(b); tri.Add(c); tri.Add(a); tri.Add(c); tri.Add(d); }
                    }
            };
            foreach (var f in proj.Features.Where(x => x.Export && x.Geometry != null))
            {
                var g = GeoUtil.Transform(f.Geometry, m);
                var pts = ActivePositions(f, m);
                bool inside = IsInternal(f);
                if (g is SurfacePrim) { var s = (SurfacePrim)g; grid((u, v) => s.Evaluate(u, v), 64, 64, false); }
                else if (g is PlanePrim)
                {
                    var c = PlaneBounds.Corners(f, (PlanePrim)f.Geometry).Select(m.TransformPoint).ToArray();
                    grid((u, v) => c[0] + (c[1] - c[0]) * u + (c[3] - c[0]) * v, 1, 1, false);
                }
                else if (g is CylinderPrim || g is ConePrim)
                {
                    var cy = g as CylinderPrim; var co = g as ConePrim;
                    Vec3 o = cy != null ? cy.AxisPoint : co.AxisPoint, z = (cy != null ? cy.Axis : co.Axis).Normalized();
                    Vec3 xr; double t0, t1, h0, h1;
                    RevolvedExtent(o, z, pts, f.ExportMargin, f.FullRevolution, out xr, out t0, out t1, out h0, out h1, cy != null ? cy.Radius : co.RadiusAtOrigin);
                    Vec3 x = (xr - z * xr.Dot(z)).Normalized(), y = z.Cross(x);
                    Func<double, double> rad = h => cy != null ? cy.Radius : Math.Max(0, co.RadiusAt(h));
                    int nu = Math.Max(8, (int)Math.Ceiling((t1 - t0) / (4 * Math.PI / 180)));
                    grid((u, v) => { double t = t0 + (t1 - t0) * u, h = h0 + (h1 - h0) * v; return o + z * h + (x * Math.Cos(t) + y * Math.Sin(t)) * rad(h); }, nu, 1, inside);
                }
                else if (g is SpherePrim)
                {
                    var s = (SpherePrim)g;
                    Vec3 pole; double cap;
                    SphereCap(s, pts, f.ExportMargin, out pole, out cap);
                    Vec3 z = pole.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
                    grid((u, v) => { double t = 2 * Math.PI * u, ph = cap * v; return s.Center + (z * Math.Cos(ph) + (x * Math.Cos(t) + y * Math.Sin(t)) * Math.Sin(ph)) * s.Radius; }, 72, 24, !inside);
                }
            }
            using (var bw = new BinaryWriter(File.Create(path)))
            {
                var head = new byte[80];
                var txt = System.Text.Encoding.ASCII.GetBytes("MicroScribe Inspector surfaces (mm)");
                Array.Copy(txt, head, Math.Min(80, txt.Length));
                bw.Write(head);
                bw.Write((uint)(tri.Count / 3));
                for (int i = 0; i < tri.Count; i += 3)
                {
                    Vec3 n = (tri[i + 1] - tri[i]).Cross(tri[i + 2] - tri[i]);
                    n = n.Length > 1e-15 ? n.Normalized() : Vec3.Zero;
                    bw.Write((float)n.X); bw.Write((float)n.Y); bw.Write((float)n.Z);
                    for (int k = 0; k < 3; k++) { bw.Write((float)tri[i + k].X); bw.Write((float)tri[i + k].Y); bw.Write((float)tri[i + k].Z); }
                    bw.Write((ushort)0);
                }
            }
            return tri.Count / 3;
        }

        public static void SphereCap(SpherePrim s, IList<Vec3> pts, double margin, out Vec3 pole, out double cap)
        {
            pole = Vec3.Zero;
            foreach (var p in pts) pole += (p - s.Center).Normalized();
            pole = pole.Length > 1e-6 ? pole.Normalized() : Vec3.UnitZ;
            cap = 0;
            foreach (var p in pts) cap = Math.Max(cap, Constructions.AngleBetween(pole, p - s.Center, false));
            cap = Math.Min(Math.PI, cap + Math.Min(margin / Math.Max(s.Radius, 1e-6), 0.5));
            if (cap > Math.PI * 0.94) cap = Math.PI;
        }

        /// <summary>Bores, sockets and internal cones: face normals should point away from the material.</summary>
        static bool IsInternal(Feature f)
        {
            if (f.RawFit == null || f.RawFit is PlanePrim || f.RawFit is SurfacePrim || f.RawFit is CurvePrim) return false;
            return !FeatureSolver.IsOutside(f.RawFit, f.ActivePoints.ToList(), f.Compensation);
        }

        /// <summary>
        /// All points as CSV in the chosen units: ball centre (x, y, z), stylus direction, probe radius and
        /// the compensated contact point on the part (cx, cy, cz).
        /// </summary>
        public static void WriteCsv(Project proj, string path, bool partFrame, UnitFmt u)
        {
            Mat4 m = partFrame ? proj.AlignmentMatrix : Mat4.Identity();
            var sb = new StringBuilder();
            sb.AppendLine(string.Format("feature,type,index,section,x_{0},y_{0},z_{0},stylus_i,stylus_j,stylus_k,probe_radius_{0},excluded,cx_{0},cy_{0},cz_{0}", u.U));
            foreach (var f in proj.Features)
            {
                int k = 1;
                var contact = FeatureSolver.ContactPoints(f, f.Points);
                for (int i = 0; i < f.Points.Count; i++)
                {
                    var p = f.Points[i];
                    Vec3 q = m.TransformPoint(p.Position), d = m.TransformVector(p.Direction), c = m.TransformPoint(contact[i]);
                    sb.AppendLine(string.Format(CI, "\"{0}\",{1},{2},{3},{4},{5},{6},{7:0.000000},{8:0.000000},{9:0.000000},{10},{11},{12},{13},{14}",
                        f.Name.Replace("\"", "'"), f.Kind, k++, p.Section, u.L(q.X), u.L(q.Y), u.L(q.Z), d.X, d.Y, d.Z, u.L(p.ProbeRadius), p.Excluded ? 1 : 0,
                        u.L(c.X), u.L(c.Y), u.L(c.Z)));
                }
            }
            File.WriteAllText(path, sb.ToString());
        }

        /// <summary>Plain XYZ point cloud (space separated, probe-compensated contact points) in the chosen units.</summary>
        public static void WriteXyz(Project proj, string path, bool partFrame, UnitFmt u, IEnumerable<Feature> features = null)
        {
            Mat4 m = partFrame ? proj.AlignmentMatrix : Mat4.Identity();
            var sb = new StringBuilder();
            foreach (var f in features ?? proj.Features)
                foreach (var c in FeatureSolver.ContactPoints(f, f.ActivePoints.ToList()))
                {
                    Vec3 q = m.TransformPoint(c);
                    sb.AppendLine(u.L(q.X) + " " + u.L(q.Y) + " " + u.L(q.Z));
                }
            File.WriteAllText(path, sb.ToString());
        }

        /// <summary>Human-readable inspection report (features and dimensions). <paramref name="capability"/>: arm-check summary line.</summary>
        public static string Report(Project proj, bool partFrame, UnitFmt u, string capability = null)
        {
            Mat4 m = partFrame ? proj.AlignmentMatrix : Mat4.Identity();
            var sb = new StringBuilder();
            sb.AppendLine("MicroScribe Inspector report — " + proj.Title);
            sb.AppendLine("Date: " + DateTime.Now.ToString("yyyy-MM-dd HH:mm"));
            sb.AppendLine("Coordinate system: " + (partFrame ? proj.AlignmentDescription : "Machine"));
            sb.AppendLine("Units: " + (u.Inch ? "inch" : "mm") + ", degrees");
            if (!string.IsNullOrEmpty(capability)) sb.AppendLine(capability);
            sb.AppendLine(new string('-', 78));
            foreach (var f in proj.Features)
            {
                sb.AppendLine(string.Format("{0}  [{1}, {2} pts]", f.Name, f.Kind, f.ActivePoints.Count()));
                foreach (var line in Describe(GeoUtil.Transform(f.Geometry, m), f, u)) sb.AppendLine("    " + line);
            }
            if (proj.Dimensions.Count > 0)
            {
                sb.AppendLine(new string('-', 78));
                sb.AppendLine("MEASUREMENTS");
                var checks = proj.Dimensions.Select(d => new { d, t = Tolerance.Evaluate(d, DimensionSolver.Evaluate(proj, d), proj.AlignmentMatrix) })
                    .Where(x => x.t.Status != TolStatus.None).ToList();
                if (checks.Count > 0)
                {
                    int fail = checks.Count(x => x.t.Status == TolStatus.Fail);
                    sb.AppendLine(string.Format("  Tolerance check: {0} of {1} in tolerance{2}", checks.Count - fail, checks.Count, fail > 0 ? " — OUT OF TOLERANCE: " +
                        string.Join(", ", checks.Where(x => x.t.Status == TolStatus.Fail).Select(x => x.d.Name)) : ""));
                    sb.AppendLine();
                }
                foreach (var d in proj.Dimensions)
                    foreach (var line in DescribeDimension(proj, d, m, u)) sb.AppendLine("  " + line);
            }
            return sb.ToString();
        }

        /// <summary>"Nominal … tolerance … deviation … PASS/FAIL", or null when the dimension has no tolerance.</summary>
        public static string ToleranceLine(Project proj, Dimension d, DimResult r, UnitFmt u)
        {
            var t = Tolerance.Evaluate(d, r, proj.AlignmentMatrix);
            if (t.Status == TolStatus.None) return null;
            Func<double, string> V = x => t.IsAngle ? x.ToString("0.000", CI) + "°" : u.L(x);
            Func<double, string> S = x => (x >= 0 ? "+" : "−") + V(Math.Abs(x));
            string band = Math.Abs(d.TolPlus + d.TolMinus) < 1e-12 ? "±" + V(d.TolPlus) : S(d.TolPlus) + "/−" + V(-d.TolMinus);
            string verdict = t.Status == TolStatus.Pass ? "PASS" : "FAIL";
            if (Gdt.IsGdt(d.Kind))
            {
                string z = d.Kind == DimKind.Position || d.Kind == DimKind.Coaxiality ? "Ø" : "";
                return string.Format("Tolerance {0}{1}   Actual {0}{2}  {3}", z, u.L(d.TolPlus), u.L(t.Deviation), verdict);
            }
            if (d.Kind == DimKind.Coordinate)
                return string.Format("Nominal X {0} Y {1} Z {2}  {3}   Dev X {4} Y {5} Z {6}  {7}", u.L(d.NX), u.L(d.NY), u.L(d.NZ), band,
                    S(t.AxisDeviation.X), S(t.AxisDeviation.Y), S(t.AxisDeviation.Z), verdict);
            return string.Format("Nominal {0}  {1}   Dev {2}  {3}", V(d.Nominal), band, S(t.Deviation), verdict);
        }

        public static IEnumerable<string> DescribeDimension(Project proj, Dimension d, Mat4 m, UnitFmt u)
        {
            var r = DimensionSolver.Evaluate(proj, d);
            var lines = new List<string>();
            if (!r.Ok) { lines.Add((d.Name ?? d.Kind.ToString()) + ": " + r.Error); return lines; }
            string tol = ToleranceLine(proj, d, r, u);
            if (d.Kind == DimKind.Coordinate)
            {
                Vec3 p = m.TransformPoint(r.A);
                lines.Add(r.Title + ":  X " + u.L(p.X) + "  Y " + u.L(p.Y) + "  Z " + u.L(p.Z) + " " + u.U);
                if (tol != null) lines.Add("    " + tol);
                return lines;
            }
            lines.Add(r.Title + ":  " + (r.IsAngle ? r.Value.ToString("0.0000", CI) + "°" : u.L(r.Value) + " " + u.U));
            if (tol != null) lines.Add("    " + tol);
            if (d.Kind == DimKind.Distance)
            {
                Vec3 dv = m.TransformVector(r.B - r.A);
                lines.Add("    ΔX " + u.L(dv.X) + "  ΔY " + u.L(dv.Y) + "  ΔZ " + u.L(dv.Z));
            }
            foreach (var e in r.Extra)
            {
                string[] parts = e.Key.Split('|');
                if (parts.Length < 2) continue;
                lines.Add("    " + parts[0] + ": " + (parts[1] == "A" ? e.Value.ToString("0.0000", CI) + "°" : parts[1] == "N" ? e.Value.ToString("0", CI) : u.L(e.Value) + " " + u.U));
            }
            return lines;
        }

        public static IEnumerable<string> Describe(Primitive g, Feature f, UnitFmt u)
        {
            var lines = DescribeCore(g, f, u).ToList();
            var info = f != null && f.ConstructionInfo != null ? f.ConstructionInfo.Split('|') : null;
            Func<int, double> num = i => double.Parse(info[i], CI);
            if (info != null && info[0] == "Width" && info.Length >= 3)
                lines.Insert(0, "Width     " + u.L(num(1)) + " " + u.U + (num(2) > 0.01 ? string.Format(CI, "   (planes {0:0.000}° apart)", num(2)) : ""));
            if (info != null && info[0] == "Bolt circle" && info.Length >= 4)
                lines.Add(string.Format(CI, "{0:0} centres, every {1:0.###}°, largest off the circle {2} {3}", num(1), num(3), u.L(num(2)), u.U));
            return lines;
        }

        static IEnumerable<string> DescribeCore(Primitive g, Feature f, UnitFmt u)
        {
            var r = new List<string>();
            if (g == null) { r.Add(f.FitError ?? "not solved"); return r; }
            Func<Vec3, string> V = v => u.L(v.X) + ", " + u.L(v.Y) + ", " + u.L(v.Z);
            Func<Vec3, string> D = v => string.Format(CI, "{0:0.00000}, {1:0.00000}, {2:0.00000}", v.X, v.Y, v.Z);
            string un = " " + u.U;
            if (g is ProfilePrim)
            {
                var pp = (ProfilePrim)g;
                r.Add("Centre    " + V(pp.Center) + un);
                r.Add("Normal    " + D(pp.Normal));
                r.Add("Direction " + D(pp.XDir));
                r.Add((pp.Shape == FeatureKind.Ellipse ? "Axes      " : "Length    ") + u.L(pp.Length) + (pp.Shape == FeatureKind.Ellipse ? " × " : " × width ") + u.L(pp.Width) + un);
                if (pp.Shape == FeatureKind.Slot) r.Add("End radius " + u.L(pp.Width / 2) + un + "   centre distance " + u.L(Math.Max(0, pp.Length - pp.Width)) + un);
                if (pp.Stats != null && pp.Stats.N > 0) r.Add("Form      " + u.L(pp.Stats.Form) + "   RMS " + u.L(pp.Stats.Rms) + un);
                return r;
            }
            if (g is CloudPrim)
            {
                var c = (CloudPrim)g; Vec3 lo, hi; c.Bounds(out lo, out hi);
                r.Add("Point cloud, " + c.Points.Length + " points");
                if (c.Points.Length > 0) r.Add("Extent    " + u.L(hi.X - lo.X) + " × " + u.L(hi.Y - lo.Y) + " × " + u.L(hi.Z - lo.Z) + un);
                return r;
            }
            if (g is PointPrim) r.Add("Position  " + V(((PointPrim)g).Position) + un);
            else if (g is LinePrim) { var l = (LinePrim)g; r.Add("Point     " + V(l.Origin) + un); r.Add("Direction " + D(l.Direction)); r.Add("Length    " + u.L(l.TMax - l.TMin) + un); }
            else if (g is PlanePrim)
            {
                var p = (PlanePrim)g; r.Add("Point     " + V(p.Origin) + un); r.Add("Normal    " + D(p.Normal));
                if (f.Geometry is PlanePrim)
                {
                    Vec3 c, uu; double w, h;
                    PlaneBounds.Get(f, (PlanePrim)f.Geometry, out c, out uu, out w, out h);
                    r.Add("Size      " + u.L(w) + " × " + u.L(h) + un + (f.CustomBounds ? " (resized)" : ""));
                }
            }
            else if (g is CirclePrim) { var c = (CirclePrim)g; r.Add("Center    " + V(c.Center) + un); r.Add("Normal    " + D(c.Normal)); r.Add("Diameter  " + u.L(2 * c.Radius) + un); }
            else if (g is SpherePrim) { var s = (SpherePrim)g; r.Add("Center    " + V(s.Center) + un); r.Add("Diameter  " + u.L(2 * s.Radius) + un); }
            else if (g is CylinderPrim) { var c = (CylinderPrim)g; r.Add("Axis pt   " + V(c.AxisPoint) + un); r.Add("Axis dir  " + D(c.Axis)); r.Add("Diameter  " + u.L(2 * c.Radius) + un); r.Add("Length    " + u.L(c.HMax - c.HMin) + un); }
            else if (g is ConePrim)
            {
                var c = (ConePrim)g;
                if (c.HalfAngle > 1e-4) r.Add("Apex      " + V(c.Apex) + un);
                r.Add("Axis dir  " + D(c.Axis));
                r.Add(string.Format(CI, "Angle     {0:0.000}° included ({1:0.000}° half)", 2 * c.HalfAngle * 180 / Math.PI, c.HalfAngle * 180 / Math.PI));
                r.Add("Diameters " + u.L(2 * c.RadiusAt(c.HMin)) + " .. " + u.L(2 * c.RadiusAt(c.HMax)) + un);
            }
            else if (g is CurvePrim)
            {
                var c = (CurvePrim)g;
                r.Add((c.Degree <= 1 ? "Polyline" : "Spline (degree " + c.Degree + ")") + ", " + c.Ctrl.Length + (c.Degree <= 1 ? " vertices" : " control points") + (c.Closed ? ", closed" : ""));
                r.Add("Length    " + u.L(c.Length) + un);
            }
            else if (g is SurfacePrim)
            {
                var s = (SurfacePrim)g;
                r.Add(string.Format("Freeform B-spline, {0} curves, {1}×{2} control points, degree {3}×{4}", s.SectionCount, s.Ctrl.GetLength(0), s.Ctrl.GetLength(1), s.DegU, s.DegV));
            }
            if (g.Stats != null && g.Stats.N > 1)
            {
                string label = g is CurvePrim || g is SurfacePrim ? "Dev." : "Form";
                r.Add(label + "      " + u.L(g.Stats.Form) + "   RMS " + u.L(g.Stats.Rms) + "   (min " + u.L(g.Stats.Min) + ", max " + u.L(g.Stats.Max) + ")" + un);
            }
            return r;
        }
    }
}
