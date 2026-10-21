using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using MicroScribe.Core;
using MicroScribe.Geometry;

namespace MicroScribe.Model
{
    /// <summary>Measurements; Form and later kinds are geometric tolerances (see Gdt).</summary>
    public enum DimKind { Distance, Angle, Diameter, Coordinate, Form, Parallelism, Perpendicularity, Angularity, Position, Coaxiality, Profile }

    /// <summary>Reference to a feature, or to one probed point of a feature (PointIndex ≥ 0).</summary>
    public sealed class EntityRef
    {
        public int FeatureId;
        public int PointIndex = -1;
    }

    /// <summary>A persistent measurement made by clicking entities in the 3D view.</summary>
    public sealed class Dimension
    {
        public int Id;
        public DimKind Kind;
        public List<EntityRef> Refs = new List<EntityRef>();
        public string Name;
        /// <summary>
        /// Nominal value and tolerance band (mm, or degrees for angles). Positions: nominal X/Y/Z in the
        /// part frame, and the band applies to each axis.
        /// </summary>
        public bool Toleranced;
        public double Nominal, TolPlus = 0.1, TolMinus = -0.1;
        public double NX, NY, NZ;
        /// <summary>Form tolerances: minimum-zone evaluation instead of least squares.</summary>
        public bool MinZone;
    }

    public enum TolStatus { None, Pass, Fail }

    /// <summary>Outcome of checking a dimension against its nominal value.</summary>
    public sealed class TolResult
    {
        public TolStatus Status;
        /// <summary>Measured − nominal (positions: the axis with the largest deviation).</summary>
        public double Deviation;
        /// <summary>Positions: deviation per axis in the part frame.</summary>
        public Vec3 AxisDeviation;
        public bool IsAngle;
    }

    public static class Tolerance
    {
        /// <summary>Checks a dimension against its nominal. <paramref name="partFrame"/>: machine → part (positions).</summary>
        public static TolResult Evaluate(Dimension d, DimResult r, Mat4 partFrame)
        {
            var t = new TolResult { IsAngle = r != null && r.IsAngle };
            if (!d.Toleranced || r == null || !r.Ok) return t;
            const double eps = 1e-9;
            Func<double, bool> inside = x => x <= d.TolPlus + eps && x >= d.TolMinus - eps;
            if (Gdt.IsGdt(d.Kind))
            {
                // geometric tolerance: the zone width must not exceed the tolerance
                t.Deviation = r.Value;
                t.Status = r.Value <= d.TolPlus + eps ? TolStatus.Pass : TolStatus.Fail;
                return t;
            }
            if (d.Kind == DimKind.Coordinate)
            {
                Vec3 p = partFrame.TransformPoint(r.A);
                t.AxisDeviation = new Vec3(p.X - d.NX, p.Y - d.NY, p.Z - d.NZ);
                var axes = new[] { t.AxisDeviation.X, t.AxisDeviation.Y, t.AxisDeviation.Z };
                t.Deviation = axes.OrderByDescending(Math.Abs).First();
                t.Status = axes.All(inside) ? TolStatus.Pass : TolStatus.Fail;
                return t;
            }
            t.Deviation = r.Value - d.Nominal;
            t.Status = inside(t.Deviation) ? TolStatus.Pass : TolStatus.Fail;
            return t;
        }

        /// <summary>Half the tolerance band (the "±" a gauge has to resolve).</summary>
        public static double HalfBand(Dimension d) { return Gdt.IsGdt(d.Kind) ? Math.Abs(d.TolPlus) / 2 : Math.Abs(d.TolPlus - d.TolMinus) / 2; }
    }

    /// <summary>Evaluated dimension (machine frame witness geometry; lengths in mm, angles in degrees).</summary>
    public sealed class DimResult
    {
        public bool Ok;
        public string Error;
        public double Value;
        public bool IsAngle;
        public string Title;
        /// <summary>Extra lines: components, gaps, etc. Lengths are tagged with {L:value} for unit formatting.</summary>
        public List<KeyValuePair<string, double>> Extra = new List<KeyValuePair<string, double>>();
        public Vec3 A, B, LabelAt;
        public bool HasLine;
        public Vec3[] Arc;   // optional polyline (angles)
    }

    public static class DimensionSolver
    {
        public static Primitive Resolve(Project p, EntityRef r, out string name)
        {
            name = "?";
            var f = p.Find(r.FeatureId);
            if (f == null) return null;
            if (r.PointIndex >= 0)
            {
                if (r.PointIndex >= f.Points.Count) return null;
                name = f.Name + " pt " + (r.PointIndex + 1);
                return new PointPrim { Position = FeatureSolver.ContactPoint(f, f.Points[r.PointIndex]) };
            }
            name = f.Name;
            return f.Geometry;
        }

        public static string RefName(Project p, EntityRef r) { string n; Resolve(p, r, out n); return n; }

        /// <summary>How many entity picks a kind needs (angle: 2 directional entities, or 3 points).</summary>
        public static int PicksNeeded(DimKind k, Project p, List<EntityRef> picked)
        {
            if (Gdt.IsGdt(k)) return Gdt.Picks(k);
            switch (k)
            {
                case DimKind.Diameter:
                case DimKind.Coordinate: return 1;
                case DimKind.Angle:
                    if (picked.Count >= 1)
                    {
                        string n;
                        var g = Resolve(p, picked[0], out n);
                        if (g is PointPrim) return 3; // vertex angle A-B-C
                    }
                    return 2;
            }
            return 2;
        }

        static double Radius(Primitive g)
        {
            if (g is CirclePrim) return ((CirclePrim)g).Radius;
            if (g is CylinderPrim) return ((CylinderPrim)g).Radius;
            if (g is SpherePrim) return ((SpherePrim)g).Radius;
            return -1;
        }

        public static DimResult Evaluate(Project p, Dimension d)
        {
            var r = new DimResult();
            if (Gdt.IsGdt(d.Kind))
            {
                try { Gdt.Evaluate(p, d, r); } catch (Exception ex) { r.Ok = false; r.Error = ex.Message; }
                return r;
            }
            var g = new List<Primitive>();
            var names = new List<string>();
            foreach (var e in d.Refs)
            {
                string n;
                var x = Resolve(p, e, out n);
                if (x == null) { r.Error = "Entity missing or not solved: " + n; return r; }
                g.Add(x); names.Add(n);
            }
            try
            {
                switch (d.Kind)
                {
                    case DimKind.Coordinate:
                        {
                            Vec3 k = Constructions.KeyPoint(g[0]);
                            r.A = r.B = r.LabelAt = k;
                            r.Title = "Position of " + names[0];
                            r.Value = double.NaN;
                            r.Extra.Add(new KeyValuePair<string, double>("X", double.NaN));
                            r.Ok = true;
                            return r;
                        }
                    case DimKind.Diameter:
                        {
                            var g0 = g[0];
                            double rad = Radius(g0);
                            if (g0 is ConePrim)
                            {
                                var c = (ConePrim)g0;
                                double hm = 0.5 * (c.HMin + c.HMax);
                                rad = c.RadiusAt(hm);
                                Vec3 cen = c.AxisPoint + c.Axis * hm;
                                Vec3 u = c.Axis.AnyPerpendicular();
                                r.A = cen - u * rad; r.B = cen + u * rad;
                                r.Title = "Cone Ø at mid height (" + names[0] + ")";
                                r.Extra.Add(new KeyValuePair<string, double>("Ø at top|L", 2 * c.RadiusAt(c.HMax)));
                                r.Extra.Add(new KeyValuePair<string, double>("Ø at bottom|L", 2 * c.RadiusAt(c.HMin)));
                                r.Extra.Add(new KeyValuePair<string, double>("Included angle|A", 2 * c.HalfAngle * 180 / Math.PI));
                            }
                            else
                            {
                                if (rad < 0) { r.Error = names[0] + " has no diameter"; return r; }
                                Vec3 cen = Constructions.KeyPoint(g0);
                                Vec3 n = Constructions.Direction(g0) ?? Vec3.UnitZ;
                                if (g0 is CylinderPrim) cen = ((CylinderPrim)g0).AxisPoint + n * ((CylinderPrim)g0).HMax;
                                Vec3 u = n.AnyPerpendicular();
                                r.A = cen - u * rad; r.B = cen + u * rad;
                                r.Title = "Diameter of " + names[0];
                                r.Extra.Add(new KeyValuePair<string, double>("Radius|L", rad));
                            }
                            r.Value = 2 * rad;
                            r.HasLine = true;
                            r.LabelAt = (r.A + r.B) * 0.5;
                            r.Ok = true;
                            return r;
                        }
                    case DimKind.Angle:
                        {
                            if (g.Count == 3 && g.All(x => x is PointPrim))
                            {
                                Vec3 a = ((PointPrim)g[0]).Position, b = ((PointPrim)g[1]).Position, c = ((PointPrim)g[2]).Position;
                                double ang = Constructions.AngleBetween(a - b, c - b, false) * 180 / Math.PI;
                                r.Value = ang; r.IsAngle = true;
                                r.Title = "Angle " + names[0] + " – " + names[1] + " – " + names[2];
                                r.Arc = ArcPoints(b, a - b, c - b, Math.Min((a - b).Length, (c - b).Length) * 0.4);
                                r.A = a; r.B = c; r.LabelAt = r.Arc[r.Arc.Length / 2];
                                r.Ok = true;
                                return r;
                            }
                            var m = Constructions.Angle(g[0], g[1]);
                            if (m == null) { r.Error = "Angle needs two features with a direction (plane, line, axis, circle) or three points"; return r; }
                            r.Value = m.Value; r.IsAngle = true;
                            r.Title = m.Label + ": " + names[0] + " / " + names[1];
                            Vec3 k0 = Constructions.KeyPoint(g[0]), k1 = Constructions.KeyPoint(g[1]);
                            r.A = k0; r.B = k1; r.LabelAt = (k0 + k1) * 0.5;
                            r.Extra.Add(new KeyValuePair<string, double>("Supplement|A", 180 - m.Value));
                            r.Ok = true;
                            return r;
                        }
                    default:
                        {
                            var m = Constructions.Distance(g[0], g[1]);
                            r.Value = m.Value;
                            r.Title = m.Label + ": " + names[0] + " ↔ " + names[1];
                            r.A = m.A; r.B = m.B; r.HasLine = true;
                            r.LabelAt = (m.A + m.B) * 0.5;
                            double r0 = Radius(g[0]), r1 = Radius(g[1]);
                            bool c0 = r0 > 0 && !(g[0] is CirclePrim && g[1] is PlanePrim), c1 = r1 > 0 && !(g[1] is CirclePrim && g[0] is PlanePrim);
                            if (c0 && c1)
                            {
                                r.Extra.Add(new KeyValuePair<string, double>("Gap (min, surface to surface)|L", m.Value - r0 - r1));
                                r.Extra.Add(new KeyValuePair<string, double>("Max (outside to outside)|L", m.Value + r0 + r1));
                            }
                            else if (c0 ^ c1)
                            {
                                double rr = c0 ? r0 : r1;
                                r.Extra.Add(new KeyValuePair<string, double>("To surface (min)|L", m.Value - rr));
                                r.Extra.Add(new KeyValuePair<string, double>("To surface (max)|L", m.Value + rr));
                            }
                            r.Ok = true;
                            return r;
                        }
                }
            }
            catch (Exception ex) { r.Error = ex.Message; return r; }
        }

        static Vec3[] ArcPoints(Vec3 c, Vec3 a, Vec3 b, double radius)
        {
            Vec3 x = a.Normalized();
            Vec3 n = a.Cross(b);
            if (n.Length < 1e-9) n = x.AnyPerpendicular();
            Vec3 y = n.Normalized().Cross(x);
            double ang = Constructions.AngleBetween(a, b, false);
            var pts = new Vec3[25];
            for (int i = 0; i < pts.Length; i++)
            {
                double t = ang * i / (pts.Length - 1);
                pts[i] = c + (x * Math.Cos(t) + y * Math.Sin(t)) * radius;
            }
            return pts;
        }
    }
}
