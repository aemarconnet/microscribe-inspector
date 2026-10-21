using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;
using MicroScribe.Geometry;

namespace MicroScribe.Model
{
    /// <summary>
    /// Basic geometric tolerances evaluated from the least-squares features (values in mm, zone widths).
    /// Form: flatness, straightness, roundness, cylindricity (and the form of spheres and cones).
    /// Orientation: parallelism, perpendicularity, angularity of a plane or axis to a datum plane or axis.
    /// Location: position (in the part frame, which acts as the datum reference frame) and coaxiality.
    /// Values use the probe-compensated contact points.
    /// </summary>
    public static class Gdt
    {
        public static bool IsGdt(DimKind k) { return k >= DimKind.Form; }

        /// <summary>Number of features to pick: the feature, then the datum for orientation and coaxiality.</summary>
        public static int Picks(DimKind k) { return k == DimKind.Form || k == DimKind.Position || k == DimKind.Profile ? 1 : 2; }

        /// <summary>Name of the characteristic for a feature (form depends on the geometry).</summary>
        public static string Name(DimKind k, Primitive g)
        {
            switch (k)
            {
                case DimKind.Form:
                    if (g is PlanePrim) return "Flatness";
                    if (g is LinePrim) return "Straightness";
                    if (g is CirclePrim) return "Roundness";
                    if (g is CylinderPrim) return "Cylindricity";
                    return "Form";
                case DimKind.Parallelism: return "Parallelism";
                case DimKind.Perpendicularity: return "Perpendicularity";
                case DimKind.Angularity: return "Angularity";
                case DimKind.Position: return "Position";
                case DimKind.Coaxiality: return "Coaxiality";
                case DimKind.Profile: return "Profile";
            }
            return k.ToString();
        }

        /// <summary>Short label for the 3D view.</summary>
        public static string Symbol(DimKind k, Primitive g)
        {
            switch (k)
            {
                case DimKind.Parallelism: return "∥";
                case DimKind.Perpendicularity: return "⊥";
                case DimKind.Angularity: return "∠";
                case DimKind.Position: return "Pos Ø";
                case DimKind.Coaxiality: return "◎ Ø";
                case DimKind.Profile: return "Profile";
            }
            return Name(k, g);
        }

        static bool IsAxis(Primitive g) { return g is CylinderPrim || g is ConePrim || g is LinePrim; }

        /// <summary>Ends of a feature's axis over its measured extent.</summary>
        static void AxisEnds(Primitive g, out Vec3 a, out Vec3 b)
        {
            if (g is CylinderPrim) { var c = (CylinderPrim)g; a = c.AxisPoint + c.Axis * c.HMin; b = c.AxisPoint + c.Axis * c.HMax; return; }
            if (g is ConePrim) { var c = (ConePrim)g; a = c.AxisPoint + c.Axis * c.HMin; b = c.AxisPoint + c.Axis * c.HMax; return; }
            if (g is LinePrim) { var l = (LinePrim)g; a = l.Start; b = l.End; return; }
            a = b = Constructions.KeyPoint(g);
        }

        /// <summary>
        /// Profile of a surface (or of a set of comparison points) against the nominal model: the
        /// bilateral zone 2 × the largest deviation of the probed points, ball radius removed.
        /// </summary>
        static void Profile(Project p, Dimension d, Feature f, DimResult r)
        {
            if (f == null) { r.Error = "Feature missing"; return; }
            var mesh = p.NominalModel;
            if (mesh == null) { r.Error = "Load a nominal model (Alignment tab) first"; return; }
            var pts = f.ActivePoints.ToList();
            if (pts.Count == 0) { r.Error = f.Name + " has no probed points"; return; }
            Mat4 m = p.AlignmentMatrix;
            var dev = pts.Select(q => ModelFit.Deviation(mesh, m, q.Position, f.Compensation == CompensationSide.None ? 0 : q.ProbeRadius)).ToList();
            double worst = dev.Max(x => Math.Abs(x));
            r.Value = 2 * worst;
            r.Title = "Profile of " + f.Name + " to the model";
            r.A = r.B = r.LabelAt = pts[dev.IndexOf(dev.OrderByDescending(Math.Abs).First())].Position;
            r.Extra.Add(new KeyValuePair<string, double>("Largest deviation|L", dev.OrderByDescending(Math.Abs).First()));
            r.Extra.Add(new KeyValuePair<string, double>("Min|L", dev.Min()));
            r.Extra.Add(new KeyValuePair<string, double>("Max|L", dev.Max()));
            r.Extra.Add(new KeyValuePair<string, double>("RMS|L", Math.Sqrt(dev.Sum(x => x * x) / dev.Count)));
            r.Extra.Add(new KeyValuePair<string, double>("Points|N", dev.Count));
            r.Ok = true;
        }

        static double DistanceToLine(Vec3 p, Vec3 o, Vec3 dir)
        {
            Vec3 w = p - o;
            return (w - dir * w.Dot(dir)).Length;
        }

        /// <summary>
        /// Minimum-zone form: the narrowest zone that holds all points (Chebyshev fit), searched from the
        /// least-squares feature. Falls back to the least-squares value for other shapes.
        /// </summary>
        public static double MinimumZone(Primitive g, IList<Vec3> pts)
        {
            Func<IEnumerable<double>, double> range = v => { var l = v.ToList(); return l.Max() - l.Min(); };
            if (g is PlanePrim)
            {
                var pl = (PlanePrim)g;
                Vec3 u = pl.Normal.AnyPerpendicular(), v = pl.Normal.Cross(u);
                return NelderMead(x => { Vec3 n = (pl.Normal + u * x[0] + v * x[1]).Normalized(); return range(pts.Select(q => q.Dot(n))); }, new double[2], 1e-3);
            }
            if (g is CirclePrim)
            {
                var c = (CirclePrim)g;
                Vec3 u = c.Normal.AnyPerpendicular(), v = c.Normal.Cross(u);
                return NelderMead(x =>
                {
                    Vec3 o = c.Center + u * x[0] + v * x[1];
                    return range(pts.Select(q => { Vec3 w = q - o; return (w - c.Normal * w.Dot(c.Normal)).Length; }));
                }, new double[2], Math.Max(1e-3, c.Radius * 1e-3));
            }
            if (g is SpherePrim)
            {
                var s = (SpherePrim)g;
                return NelderMead(x => range(pts.Select(q => (q - (s.Center + new Vec3(x[0], x[1], x[2]))).Length)), new double[3], Math.Max(1e-3, s.Radius * 1e-3));
            }
            if (g is CylinderPrim || g is LinePrim)
            {
                Vec3 a0 = g is CylinderPrim ? ((CylinderPrim)g).Axis : ((LinePrim)g).Direction;
                Vec3 o0 = g is CylinderPrim ? ((CylinderPrim)g).AxisPoint : ((LinePrim)g).Origin;
                Vec3 u = a0.AnyPerpendicular(), v = a0.Cross(u);
                bool line = g is LinePrim;
                return NelderMead(x =>
                {
                    Vec3 ax = (a0 + u * x[0] + v * x[1]).Normalized(), o = o0 + u * x[2] + v * x[3];
                    var rho = pts.Select(q => DistanceToLine(q, o, ax)).ToList();
                    return line ? 2 * rho.Max() : rho.Max() - rho.Min();
                }, new double[4], 1e-3);
            }
            return double.MaxValue;
        }

        /// <summary>Downhill simplex minimiser with a few restarts (the zone functions are not smooth).</summary>
        static double NelderMead(Func<double[], double> f, double[] x0, double step)
        {
            int n = x0.Length;
            double[] best = (double[])x0.Clone();
            double fbest = f(best);
            for (int restart = 0; restart < 4; restart++)
            {
                var sx = new double[n + 1][];
                var fx = new double[n + 1];
                for (int i = 0; i <= n; i++)
                {
                    sx[i] = (double[])best.Clone();
                    if (i > 0) sx[i][i - 1] += step;
                    fx[i] = f(sx[i]);
                }
                for (int it = 0; it < 400 * n; it++)
                {
                    var order = Enumerable.Range(0, n + 1).OrderBy(i => fx[i]).ToArray();
                    sx = order.Select(i => sx[i]).ToArray(); fx = order.Select(i => fx[i]).ToArray();
                    if (fx[n] - fx[0] < 1e-12) break;
                    var c = new double[n];
                    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) c[j] += sx[i][j] / n;
                    Func<double, double[]> along = t => c.Select((cj, j) => cj + t * (sx[n][j] - cj)).ToArray();
                    var xr = along(-1); double fr = f(xr);
                    if (fr < fx[0])
                    {
                        var xe = along(-2); double fe = f(xe);
                        if (fe < fr) { sx[n] = xe; fx[n] = fe; } else { sx[n] = xr; fx[n] = fr; }
                    }
                    else if (fr < fx[n - 1]) { sx[n] = xr; fx[n] = fr; }
                    else
                    {
                        var xc = along(fr < fx[n] ? -0.5 : 0.5); double fc = f(xc);
                        if (fc < Math.Min(fr, fx[n])) { sx[n] = xc; fx[n] = fc; }
                        else
                            for (int i = 1; i <= n; i++)
                            {
                                sx[i] = sx[i].Select((v, j) => sx[0][j] + 0.5 * (v - sx[0][j])).ToArray();
                                fx[i] = f(sx[i]);
                            }
                    }
                }
                int bi = Array.IndexOf(fx, fx.Min());
                if (fx[bi] < fbest - 1e-12) { fbest = fx[bi]; best = sx[bi]; } else if (restart > 0) break;
                step *= 0.5;
            }
            return fbest;
        }

        public static void Evaluate(Project p, Dimension d, DimResult r)
        {
            var feats = d.Refs.Select(e => p.Find(e.FeatureId)).ToList();
            if (d.Kind == DimKind.Profile) { Profile(p, d, feats[0], r); return; }
            if (feats.Any(f => f == null || f.Geometry == null)) { r.Error = "Feature missing or not solved"; return; }
            var f0 = feats[0];
            Primitive g = f0.Geometry;
            Vec3 key = Constructions.KeyPoint(g);
            r.A = r.B = r.LabelAt = key;
            string name = Name(d.Kind, g);
            switch (d.Kind)
            {
                case DimKind.Form:
                    {
                        if (!(g is PlanePrim || g is LinePrim || g is CirclePrim || g is CylinderPrim || g is SpherePrim || g is ConePrim))
                        { r.Error = "Form needs a plane, line, circle, cylinder, cone or sphere"; return; }
                        var cps = FeatureSolver.ContactPoints(f0, f0.ActivePoints.ToList());
                        if (cps.Count < Fitter.MinPoints(f0.Kind) + 1) { r.Error = "Not enough points to judge the form (" + cps.Count + ")"; return; }
                        var res = cps.Select(g.SignedDistance).ToList();
                        // a line's distances are unsigned: the zone is a cylinder around it
                        double lsq = g is LinePrim ? 2 * res.Max() : res.Max() - res.Min();
                        r.Value = lsq;
                        r.Title = name + " of " + f0.Name;
                        if (d.MinZone)
                        {
                            r.Value = Math.Min(lsq, MinimumZone(g, cps));
                            r.Title += " (minimum zone)";
                            r.Extra.Add(new KeyValuePair<string, double>("Least squares|L", lsq));
                        }
                        r.Extra.Add(new KeyValuePair<string, double>("Points|N", cps.Count));
                        break;
                    }
                case DimKind.Parallelism:
                case DimKind.Perpendicularity:
                case DimKind.Angularity:
                    {
                        var f1 = feats[1];
                        Primitive dg = f1.Geometry;
                        bool fPlane = g is PlanePrim, dPlane = dg is PlanePrim;
                        if (!(fPlane || IsAxis(g)) || !(dPlane || IsAxis(dg))) { r.Error = name + " needs a plane or axis, and a datum plane or axis"; return; }
                        Vec3 fd = Constructions.Direction(g).Value, dd = Constructions.Direction(dg).Value;
                        if (fd.Dot(dd) < 0) fd = -fd;
                        double theta = d.Kind == DimKind.Parallelism ? 0 : d.Kind == DimKind.Perpendicularity ? 90 : d.Nominal;
                        // ideal angle between the feature's direction and the datum's direction
                        double phi = (fPlane == dPlane ? theta : 90 - theta) * Math.PI / 180;
                        Vec3 w = fd - dd * fd.Dot(dd);
                        w = w.Length > 1e-9 ? w.Normalized() : dd.AnyPerpendicular();
                        Vec3 m = dd * Math.Cos(phi) + w * Math.Sin(phi);
                        if (fPlane)
                        {
                            // zone: two planes with normal m
                            var cps = FeatureSolver.ContactPoints(f0, f0.ActivePoints.ToList());
                            if (cps.Count < 3) { r.Error = "Not enough points"; return; }
                            var h = cps.Select(q => q.Dot(m)).ToList();
                            r.Value = h.Max() - h.Min();
                        }
                        else
                        {
                            // the derived axis must fit in a zone around a line along m
                            Vec3 a, b;
                            AxisEnds(g, out a, out b);
                            Vec3 e = (b - a) - m * (b - a).Dot(m);
                            r.Value = e.Length;
                        }
                        double actual = Constructions.AngleBetween(fd, dd, true) * 180 / Math.PI;
                        if (fPlane != dPlane) actual = 90 - actual;
                        r.Title = name + " of " + f0.Name + " to " + f1.Name + (d.Kind == DimKind.Angularity ? string.Format(System.Globalization.CultureInfo.InvariantCulture, " (nominal {0:0.###}°)", d.Nominal) : "");
                        r.Extra.Add(new KeyValuePair<string, double>("Angle between|A", actual));
                        r.B = Constructions.KeyPoint(dg);
                        break;
                    }
                case DimKind.Position:
                    {
                        if (!(g is PointPrim || g is CirclePrim || g is SpherePrim || IsAxis(g))) { r.Error = "Position needs a point, circle, sphere, cylinder, cone or line"; return; }
                        Mat4 part = p.AlignmentMatrix;
                        Vec3 k = part.TransformPoint(key), dev = k - new Vec3(d.NX, d.NY, d.NZ);
                        var dir = Constructions.Direction(g);
                        bool axial = dir.HasValue && !(g is SpherePrim);
                        if (axial)
                        {
                            // holes and pins: the deviation across the axis counts, not along it
                            Vec3 ax = part.TransformVector(dir.Value).Normalized();
                            dev = dev - ax * dev.Dot(ax);
                        }
                        r.Value = 2 * dev.Length;
                        r.Title = name + " of " + f0.Name + (axial ? " (across its axis)" : "");
                        r.Extra.Add(new KeyValuePair<string, double>("dX|L", dev.X));
                        r.Extra.Add(new KeyValuePair<string, double>("dY|L", dev.Y));
                        r.Extra.Add(new KeyValuePair<string, double>("dZ|L", dev.Z));
                        break;
                    }
                case DimKind.Coaxiality:
                    {
                        var f1 = feats[1];
                        Primitive dg = f1.Geometry;
                        var ddir = Constructions.Direction(dg);
                        if (!(IsAxis(dg) || dg is CirclePrim) || ddir == null || !(IsAxis(g) || g is CirclePrim))
                        { r.Error = "Coaxiality needs two features with an axis (cylinder, cone, line or circle)"; return; }
                        Vec3 a, b, o = Constructions.KeyPoint(dg), u = ddir.Value.Normalized();
                        AxisEnds(g, out a, out b);
                        r.Value = 2 * Math.Max(DistanceToLine(a, o, u), DistanceToLine(b, o, u));
                        r.Title = name + " of " + f0.Name + " to " + f1.Name;
                        r.B = o;
                        break;
                    }
            }
            r.Ok = true;
        }
    }
}
