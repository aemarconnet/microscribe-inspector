using System;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>Result of a feature-to-feature measurement.</summary>
    public sealed class Measurement
    {
        public string Label;
        public double Value;
        public bool IsAngle;
        /// <summary>Optional witness points for drawing a dimension line.</summary>
        public Vec3 A, B;
        public bool HasWitness;
    }

    /// <summary>Geometric relations between primitives.</summary>
    public static class Constructions
    {
        /// <summary>Representative point of a primitive (centre, origin, apex...).</summary>
        public static Vec3 KeyPoint(Primitive p)
        {
            if (p is PointPrim) return ((PointPrim)p).Position;
            if (p is LinePrim) return ((LinePrim)p).Origin;
            if (p is PlanePrim) return ((PlanePrim)p).Origin;
            if (p is CirclePrim) return ((CirclePrim)p).Center;
            if (p is SpherePrim) return ((SpherePrim)p).Center;
            if (p is CylinderPrim) return ((CylinderPrim)p).AxisPoint;
            if (p is ConePrim) return ((ConePrim)p).AxisPoint;
            if (p is ProfilePrim) return ((ProfilePrim)p).Center;
            return Vec3.Zero;
        }

        /// <summary>Direction of a primitive, if it has one (line dir, plane normal, axis).</summary>
        public static Vec3? Direction(Primitive p)
        {
            if (p is LinePrim) return ((LinePrim)p).Direction;
            if (p is PlanePrim) return ((PlanePrim)p).Normal;
            if (p is CirclePrim) return ((CirclePrim)p).Normal;
            if (p is CylinderPrim) return ((CylinderPrim)p).Axis;
            if (p is ConePrim) return ((ConePrim)p).Axis;
            if (p is ProfilePrim) return ((ProfilePrim)p).Normal;
            return null;
        }

        /// <summary>True if the primitive behaves as a line for distance purposes (line or axis).</summary>
        static bool IsLinear(Primitive p) { return p is LinePrim || p is CylinderPrim || p is ConePrim; }

        static bool IsPointLike(Primitive p) { return p is PointPrim || p is CirclePrim || p is SpherePrim || p is ProfilePrim; }

        public static Measurement Distance(Primitive a, Primitive b)
        {
            var m = new Measurement { HasWitness = true };
            if (IsPointLike(a) && IsPointLike(b))
            {
                m.A = KeyPoint(a); m.B = KeyPoint(b);
                m.Value = (m.B - m.A).Length;
                m.Label = "Point-point distance";
                return m;
            }
            if (b is PlanePrim && !(a is PlanePrim)) { var t = a; a = b; b = t; }
            if (a is PlanePrim)
            {
                var pl = (PlanePrim)a;
                if (b is PlanePrim)
                {
                    var pb = (PlanePrim)b;
                    m.B = pb.Origin; m.A = pl.Project(pb.Origin);
                    m.Value = Math.Abs(pl.SignedDistance(pb.Origin));
                    m.Label = string.Format("Plane-plane distance (planes {0:0.00}° apart)", AngleBetween(pl.Normal, pb.Normal, true) * 180 / Math.PI);
                    return m;
                }
                Vec3 k = KeyPoint(b);
                m.B = k; m.A = pl.Project(k);
                m.Value = Math.Abs(pl.SignedDistance(k));
                m.Label = "Distance to plane";
                return m;
            }
            if (IsLinear(a) && IsLinear(b))
            {
                Vec3 p1 = KeyPoint(a), d1 = Direction(a).Value, p2 = KeyPoint(b), d2 = Direction(b).Value;
                double ang = AngleBetween(d1, d2, true);
                if (ang < 2 * Math.PI / 180)
                {
                    // Nearly parallel (typical for bores/bosses): measure at the features, not at the
                    // mathematically closest points, which can lie far outside the part.
                    Vec3 mid = (p1 + p2) * 0.5;
                    Vec3 f1 = p1 + d1 * (mid - p1).Dot(d1), f2 = p2 + d2 * (mid - p2).Dot(d2);
                    m.A = f1; m.B = f2; m.Value = (f2 - f1).Length;
                    m.Label = string.Format(System.Globalization.CultureInfo.InvariantCulture, "Distance between parallel axes (at the features; {0:0.000}° apart)", ang * 180 / Math.PI);
                    return m;
                }
                Vec3 c1, c2;
                ClosestPointsLines(p1, d1, p2, d2, out c1, out c2);
                m.A = c1; m.B = c2; m.Value = (c2 - c1).Length;
                m.Label = "Closest distance between lines";
                return m;
            }
            if (IsLinear(b)) { var t = a; a = b; b = t; }
            if (IsLinear(a))
            {
                Vec3 p = KeyPoint(a), d = Direction(a).Value, k = KeyPoint(b);
                Vec3 foot = p + d * (k - p).Dot(d);
                m.A = foot; m.B = k; m.Value = (k - foot).Length;
                m.Label = "Distance to axis/line";
                return m;
            }
            m.A = KeyPoint(a); m.B = KeyPoint(b); m.Value = (m.B - m.A).Length;
            m.Label = "Distance";
            return m;
        }

        /// <summary>Angle between direction vectors; unsignedAxis folds to [0, 90°].</summary>
        public static double AngleBetween(Vec3 a, Vec3 b, bool unsignedAxis)
        {
            double c = a.Normalized().Dot(b.Normalized());
            c = Math.Max(-1, Math.Min(1, c));
            double ang = Math.Acos(c);
            if (unsignedAxis && ang > Math.PI / 2) ang = Math.PI - ang;
            return ang;
        }

        public static Measurement Angle(Primitive a, Primitive b)
        {
            var da = Direction(a); var db = Direction(b);
            if (!da.HasValue || !db.HasValue) return null;
            double ang = AngleBetween(da.Value, db.Value, true);
            string label;
            if (a is PlanePrim && b is PlanePrim) label = "Angle between planes";
            else if (a is PlanePrim || b is PlanePrim)
            {
                // line/axis to plane: 90° - angle to normal
                ang = Math.PI / 2 - ang;
                label = "Angle line/axis to plane";
            }
            else label = "Angle between lines/axes";
            return new Measurement { Label = label, Value = ang * 180 / Math.PI, IsAngle = true };
        }

        public static void ClosestPointsLines(Vec3 p1, Vec3 d1, Vec3 p2, Vec3 d2, out Vec3 c1, out Vec3 c2)
        {
            Vec3 r = p1 - p2;
            double a = d1.Dot(d1), b = d1.Dot(d2), c = d2.Dot(d2), d = d1.Dot(r), e = d2.Dot(r);
            double den = a * c - b * b;
            double s, t;
            if (Math.Abs(den) < 1e-12) { s = 0; t = e / c; }
            else { s = (b * e - c * d) / den; t = (a * e - b * d) / den; }
            c1 = p1 + d1 * s; c2 = p2 + d2 * t;
        }

        // ------------------------------------------------------------------ intersections

        /// <summary>Plane ∩ plane → line (null if parallel).</summary>
        public static LinePrim Intersect(PlanePrim a, PlanePrim b)
        {
            Vec3 d = a.Normal.Cross(b.Normal);
            if (d.Length < 1e-9) return null;
            d = d.Normalized();
            // point on both planes closest to the midpoint of the two origins
            Vec3 n1 = a.Normal, n2 = b.Normal;
            double h1 = n1.Dot(a.Origin), h2 = n2.Dot(b.Origin);
            var A = new double[3, 3] { { n1.X, n1.Y, n1.Z }, { n2.X, n2.Y, n2.Z }, { d.X, d.Y, d.Z } };
            Vec3 mid = (a.Origin + b.Origin) * 0.5;
            double[] x;
            if (!LinAlg.Solve(A, new[] { h1, h2, d.Dot(mid) }, out x)) return null;
            var l = new LinePrim { Origin = new Vec3(x[0], x[1], x[2]), Direction = d, TMin = -50, TMax = 50 };
            return l;
        }

        /// <summary>Line/axis ∩ plane → point (null if parallel).</summary>
        public static PointPrim Intersect(Vec3 linePt, Vec3 lineDir, PlanePrim pl)
        {
            double den = lineDir.Dot(pl.Normal);
            if (Math.Abs(den) < 1e-9) return null;
            double t = (pl.Origin - linePt).Dot(pl.Normal) / den;
            return new PointPrim { Position = linePt + lineDir * t };
        }

        /// <summary>Three planes → point.</summary>
        public static PointPrim Intersect(PlanePrim a, PlanePrim b, PlanePrim c)
        {
            var A = new double[3, 3] {
                { a.Normal.X, a.Normal.Y, a.Normal.Z },
                { b.Normal.X, b.Normal.Y, b.Normal.Z },
                { c.Normal.X, c.Normal.Y, c.Normal.Z } };
            double[] x;
            if (!LinAlg.Solve(A, new[] { a.Normal.Dot(a.Origin), b.Normal.Dot(b.Origin), c.Normal.Dot(c.Origin) }, out x)) return null;
            return new PointPrim { Position = new Vec3(x[0], x[1], x[2]) };
        }

        /// <summary>Generic intersection of two primitives, returning a point or line, or null.</summary>
        public static Primitive Intersect(Primitive a, Primitive b)
        {
            if (a is PlanePrim && b is PlanePrim) return Intersect((PlanePrim)a, (PlanePrim)b);
            if (b is PlanePrim) { var t = a; a = b; b = t; }
            if (a is PlanePrim && (IsLinear(b) || b is CirclePrim))
                return Intersect(KeyPoint(b), Direction(b).Value, (PlanePrim)a);
            if (IsLinear(a) && IsLinear(b))
            {
                Vec3 c1, c2;
                ClosestPointsLines(KeyPoint(a), Direction(a).Value, KeyPoint(b), Direction(b).Value, out c1, out c2);
                return new PointPrim { Position = (c1 + c2) * 0.5 };
            }
            return null;
        }

        // ------------------------------------------------------------------ alignment

        /// <summary>
        /// 3-2-1 style alignment. primary sets Z (plane normal / axis), secondary sets X
        /// (projected onto the primary plane), origin from a point-like feature or intersection.
        /// Returns the transform from machine (arm) coordinates to part coordinates.
        /// </summary>
        public static Mat4 Alignment(Primitive primary, Primitive secondary, Primitive origin, bool flipZ = false, bool flipX = false)
        {
            Vec3 z = Direction(primary) ?? Vec3.UnitZ;
            if (flipZ) z = -z;
            z = z.Normalized();
            Vec3 x;
            var sd = secondary != null ? Direction(secondary) : null;
            if (sd.HasValue)
            {
                Vec3 s = sd.Value;
                if (secondary is PlanePrim)
                    s = z.Cross(s).Length > 1e-6 ? s : z.AnyPerpendicular(); // plane normal used as X direction
                x = s - z * s.Dot(z);
                if (x.Length < 1e-6) x = z.AnyPerpendicular();
            }
            else if (secondary != null)
            {
                // point-like secondary: X points from origin toward it
                Vec3 o0 = origin != null ? KeyPoint(origin) : KeyPoint(primary);
                Vec3 s = KeyPoint(secondary) - o0;
                x = s - z * s.Dot(z);
                if (x.Length < 1e-6) x = z.AnyPerpendicular();
            }
            else x = z.AnyPerpendicular();
            x = x.Normalized();
            if (flipX) x = -x;
            Vec3 y = z.Cross(x).Normalized();

            Vec3 o;
            if (origin != null) o = KeyPoint(origin);
            else o = KeyPoint(primary);
            // Put the origin on the primary plane if the primary is a plane.
            if (primary is PlanePrim) o = ((PlanePrim)primary).Project(o);

            Mat4 partToMachine = Mat4.FromFrame(o, x, y, z);
            return partToMachine.RigidInverse();
        }
    }
}
