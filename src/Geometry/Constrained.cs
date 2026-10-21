using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;
using MicroScribe.Geometry;

namespace MicroScribe.Model
{
    /// <summary>
    /// Fits with constraints, for features that are known to have a size or a direction: a fixed diameter
    /// (circle, cylinder, sphere) or cone angle, and a direction parallel or perpendicular to a datum
    /// (plane, line, cylinder, cone). Directions follow the GD&amp;T sense: a plane parallel to a datum plane,
    /// an axis perpendicular to a datum plane, and so on.
    /// </summary>
    public static class ConstrainedFit
    {
        public static bool CanFixSize(FeatureKind k) { return k == FeatureKind.Circle || k == FeatureKind.Cylinder || k == FeatureKind.Sphere || k == FeatureKind.Cone; }
        public static bool CanFixDirection(FeatureKind k) { return k == FeatureKind.Plane || k == FeatureKind.Line || k == FeatureKind.Cylinder || k == FeatureKind.Cone; }

        /// <summary>Applies the feature's constraints to a free fit of its ball centres (returns the free fit when there are none).</summary>
        public static Primitive Apply(Project proj, Feature f, Primitive free, Vec3[] p, IList<MeasuredPoint> pts)
        {
            if (free == null) return null;
            bool size = f.FixSize && CanFixSize(f.Kind) && f.FixedSize > 0;
            Vec3? dir = null;
            bool equal = true;
            if (CanFixDirection(f.Kind) && (f.DirectionRelation == "parallel" || f.DirectionRelation == "perpendicular"))
            {
                var datum = proj.Find(f.DirectionDatumId);
                if (datum != null && datum != f && datum.Geometry != null && (datum.Geometry is PlanePrim || datum.Geometry is LinePrim || datum.Geometry is CylinderPrim || datum.Geometry is ConePrim))
                {
                    dir = Constructions.Direction(datum.Geometry).Value.Normalized();
                    bool fPlane = f.Kind == FeatureKind.Plane, dPlane = datum.Geometry is PlanePrim;
                    equal = (fPlane == dPlane) == (f.DirectionRelation == "parallel");
                }
            }
            if (!size && !dir.HasValue) return free;
            if (f.Kind == FeatureKind.Cone && dir.HasValue && !equal) dir = null;   // cones: only an axis along the datum
            // a fixed size is the size of the part: the ball centres sit one radius off it
            double r = pts.Count > 0 ? pts.Average(q => q.ProbeRadius) : 0;
            double dr = r > 0 && f.Compensation != CompensationSide.None && f.Kind != FeatureKind.Cone
                ? (FeatureSolver.IsOutside(free, pts.ToList(), f.Compensation) ? -r : r) : 0;
            double rawR = f.FixedSize / 2 - dr;
            Primitive g = null;
            switch (f.Kind)
            {
                case FeatureKind.Plane: g = Plane(p, dir.Value, equal, (PlanePrim)free); break;
                case FeatureKind.Line: g = Line(p, dir.Value, equal, (LinePrim)free); break;
                case FeatureKind.Sphere: g = Sphere(p, rawR, (SpherePrim)free); break;
                case FeatureKind.Circle: g = Circle(p, rawR, (CirclePrim)free); break;
                case FeatureKind.Cylinder: g = Cylinder(p, (CylinderPrim)free, size ? rawR : double.NaN, dir, equal); break;
                case FeatureKind.Cone: g = Cone(p, (ConePrim)free, size ? f.FixedSize / 2 * Math.PI / 180 : double.NaN, dir); break;
            }
            if (g == null) return free;
            g.Stats = FitStats.From(g.Residuals(p));
            return g;
        }

        static double Quad(double[,] s, Vec3 a, Vec3 b)
        {
            double[] x = { a.X, a.Y, a.Z }, y = { b.X, b.Y, b.Z };
            double q = 0;
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) q += x[i] * s[i, j] * y[j];
            return q;
        }

        /// <summary>Unit vector in span(u, v) with the smallest (min = true) or largest spread of the points.</summary>
        static Vec3 InSubspace(double[,] s, Vec3 u, Vec3 v, bool min)
        {
            double a = Quad(s, u, u), b = Quad(s, u, v), c = Quad(s, v, v);
            double t = 0.5 * Math.Atan2(2 * b, a - c) + (min ? Math.PI / 2 : 0);   // major axis, or the minor one
            return (u * Math.Cos(t) + v * Math.Sin(t)).Normalized();
        }

        static PlanePrim Plane(Vec3[] p, Vec3 dd, bool equal, PlanePrim free)
        {
            double[,] s;
            Vec3 c = LinAlg.Scatter(p, out s);
            Vec3 n;
            if (equal) n = dd;
            else { Vec3 u = dd.AnyPerpendicular(), v = dd.Cross(u).Normalized(); n = InSubspace(s, u, v, true); }
            if (n.Dot(free.Normal) < 0) n = -n;
            return new PlanePrim { Origin = c, Normal = n };
        }

        static LinePrim Line(Vec3[] p, Vec3 dd, bool equal, LinePrim free)
        {
            double[,] s;
            Vec3 c = LinAlg.Scatter(p, out s);
            Vec3 d;
            if (equal) d = dd;
            else { Vec3 u = dd.AnyPerpendicular(), v = dd.Cross(u).Normalized(); d = InSubspace(s, u, v, false); }
            if (d.Dot(free.Direction) < 0) d = -d;
            var l = new LinePrim { Origin = c, Direction = d };
            l.TMin = p.Min(q => (q - c).Dot(d)); l.TMax = p.Max(q => (q - c).Dot(d));
            return l;
        }

        static SpherePrim Sphere(Vec3[] p, double R, SpherePrim free)
        {
            var x = new LevenbergMarquardt().Minimize((q, res) =>
            {
                Vec3 c = new Vec3(q[0], q[1], q[2]);
                for (int i = 0; i < p.Length; i++) res[i] = (p[i] - c).Length - R;
            }, new[] { free.Center.X, free.Center.Y, free.Center.Z }, p.Length);
            return new SpherePrim { Center = new Vec3(x[0], x[1], x[2]), Radius = R };
        }

        static CirclePrim Circle(Vec3[] p, double R, CirclePrim free)
        {
            Vec3 n = free.Normal, u = n.AnyPerpendicular(), v = n.Cross(u).Normalized();
            var x = new LevenbergMarquardt().Minimize((q, res) =>
            {
                Vec3 c = free.Center + u * q[0] + v * q[1];
                for (int i = 0; i < p.Length; i++) { Vec3 w = p[i] - c; res[i] = (w - n * w.Dot(n)).Length - R; }
            }, new double[2], p.Length);
            return new CirclePrim { Center = free.Center + u * x[0] + v * x[1], Normal = n, Radius = R };
        }

        /// <summary>Cylinder with a given axis direction (radius free when R is NaN): a circle fit across the axis.</summary>
        static CylinderPrim CylinderAlong(Vec3[] p, Vec3 axis, double R, Vec3 start)
        {
            Vec3 u = axis.AnyPerpendicular(), v = axis.Cross(u).Normalized();
            Vec3 c0 = start - axis * (start - p[0]).Dot(axis);
            bool fixedR = !double.IsNaN(R);
            double r0 = fixedR ? R : p.Average(q => { Vec3 w = q - c0; return (w - axis * w.Dot(axis)).Length; });
            var x = new LevenbergMarquardt().Minimize((q, res) =>
            {
                Vec3 c = c0 + u * q[0] + v * q[1];
                double rr = fixedR ? R : q[2];
                for (int i = 0; i < p.Length; i++) { Vec3 w = p[i] - c; res[i] = (w - axis * w.Dot(axis)).Length - rr; }
            }, fixedR ? new double[2] : new[] { 0, 0, r0 }, p.Length);
            return Finish(new CylinderPrim { Axis = axis, AxisPoint = c0 + u * x[0] + v * x[1], Radius = fixedR ? R : Math.Abs(x[2]) }, p);
        }

        static CylinderPrim Finish(CylinderPrim c, Vec3[] p)
        {
            Vec3 cen = Vec3.Zero; foreach (var q in p) cen += q; cen = cen / p.Length;
            c.AxisPoint = c.AxisPoint + c.Axis * (cen - c.AxisPoint).Dot(c.Axis);
            c.HMin = p.Min(q => (q - c.AxisPoint).Dot(c.Axis)); c.HMax = p.Max(q => (q - c.AxisPoint).Dot(c.Axis));
            return c;
        }

        static double Rms(Primitive g, Vec3[] p) { return Math.Sqrt(p.Sum(q => { double d = g.SignedDistance(q); return d * d; }) / p.Length); }

        static CylinderPrim Cylinder(Vec3[] p, CylinderPrim free, double R, Vec3? dir, bool equal)
        {
            if (dir.HasValue && equal)
                return CylinderAlong(p, free.Axis.Dot(dir.Value) < 0 ? -dir.Value : dir.Value, R, free.AxisPoint);
            if (dir.HasValue)
            {
                // axis in the plane perpendicular to the datum direction: search the angle, then refine
                Vec3 u = dir.Value.AnyPerpendicular(), v = dir.Value.Cross(u).Normalized();
                Func<double, CylinderPrim> at = t => CylinderAlong(p, (u * Math.Cos(t) + v * Math.Sin(t)).Normalized(), R, free.AxisPoint);
                double bt = 0, best = double.MaxValue;
                for (int k = 0; k < 90; k++) { double t = k * Math.PI / 90, e = Rms(at(t), p); if (e < best) { best = e; bt = t; } }
                double lo = bt - Math.PI / 90, hi = bt + Math.PI / 90;
                for (int k = 0; k < 40; k++)
                {
                    double m1 = lo + (hi - lo) * 0.382, m2 = lo + (hi - lo) * 0.618;
                    if (Rms(at(m1), p) < Rms(at(m2), p)) hi = m2; else lo = m1;
                }
                var cyl = at((lo + hi) / 2);
                if (cyl.Axis.Dot(free.Axis) < 0) cyl.Axis = -cyl.Axis;
                return Finish(cyl, p);
            }
            // fixed radius, free axis: small rotation of the free axis and a shift across it
            Vec3 a0 = free.Axis, o0 = free.AxisPoint, e1 = a0.AnyPerpendicular(), e2 = a0.Cross(e1).Normalized();
            var x = new LevenbergMarquardt().Minimize((q, res) =>
            {
                Vec3 ax = (a0 + e1 * q[0] + e2 * q[1]).Normalized(), o = o0 + e1 * q[2] + e2 * q[3];
                for (int i = 0; i < p.Length; i++) { Vec3 w = p[i] - o; res[i] = (w - ax * w.Dot(ax)).Length - R; }
            }, new double[4], p.Length);
            return Finish(new CylinderPrim { Axis = (a0 + e1 * x[0] + e2 * x[1]).Normalized(), AxisPoint = o0 + e1 * x[2] + e2 * x[3], Radius = R }, p);
        }

        /// <summary>Cone with a fixed half angle and/or an axis along the datum direction.</summary>
        static ConePrim Cone(Vec3[] p, ConePrim free, double half, Vec3? dir)
        {
            bool fixA = !double.IsNaN(half);
            if (fixA) half = Math.Abs(half) * (free.HalfAngle < 0 ? -1 : 1);   // keep the opening direction of the free fit
            Vec3 a0 = dir.HasValue ? (free.Axis.Dot(dir.Value) < 0 ? -dir.Value : dir.Value) : free.Axis;
            Vec3 o0 = free.AxisPoint, e1 = a0.AnyPerpendicular(), e2 = a0.Cross(e1).Normalized();
            bool fixAxis = dir.HasValue;
            // parameters: [axis tilt a, b (free axis only)] shift c, e, radius at origin, [half angle]
            var start = new List<double>();
            if (!fixAxis) { start.Add(0); start.Add(0); }
            start.Add(0); start.Add(0); start.Add(free.RadiusAtOrigin);
            if (!fixA) start.Add(free.HalfAngle);
            Func<double[], ConePrim> make = q =>
            {
                int k = 0;
                Vec3 ax = fixAxis ? a0 : (a0 + e1 * q[k++] + e2 * q[k++]).Normalized();
                Vec3 o = o0 + e1 * q[k++] + e2 * q[k++];
                double r0 = q[k++];
                return new ConePrim { Axis = ax, AxisPoint = o, RadiusAtOrigin = r0, HalfAngle = fixA ? half : q[k] };
            };
            var x = new LevenbergMarquardt().Minimize((q, res) => { var c = make(q); for (int i = 0; i < p.Length; i++) res[i] = c.SignedDistance(p[i]); }, start.ToArray(), p.Length);
            var cone = make(x);
            cone.HMin = p.Min(q => (q - cone.AxisPoint).Dot(cone.Axis)); cone.HMax = p.Max(q => (q - cone.AxisPoint).Dot(cone.Axis));
            return cone;
        }
    }
}
