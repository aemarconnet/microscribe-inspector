using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>Warnings about poorly conditioned measurements (point distribution, not just residuals).</summary>
    public static class FitQuality
    {
        public static List<string> Warnings(FeatureKind kind, Primitive raw, IList<Vec3> pts)
        {
            var w = new List<string>();
            if (raw == null || pts.Count == 0) return w;
            int n = pts.Count;
            if (n == Fitter.MinPoints(kind) && kind != FeatureKind.Point && kind != FeatureKind.Line)
                w.Add("Minimum number of points — no redundancy, form error is not checked.");
            if (raw is PlanePrim)
            {
                double[,] s; LinAlg.Scatter(pts.ToArray(), out s);
                double[] ev; Vec3[] vec; LinAlg.SymmetricEigen3(s, out ev, out vec);
                double spread1 = Math.Sqrt(Math.Max(ev[1], 0) / n), spread2 = Math.Sqrt(Math.Max(ev[2], 0) / n);
                if (spread1 < 0.08 * spread2) w.Add("Points are nearly in a line — spread them over the face.");
            }
            Vec3? axis = null; Vec3 center = Vec3.Zero; double radius = 0;
            if (raw is CirclePrim) { var c = (CirclePrim)raw; axis = c.Normal; center = c.Center; radius = c.Radius; }
            if (raw is CylinderPrim) { var c = (CylinderPrim)raw; axis = c.Axis; center = c.AxisPoint; radius = c.Radius; }
            if (raw is ConePrim) { var c = (ConePrim)raw; axis = c.Axis; center = c.AxisPoint; radius = Math.Abs(c.RadiusAtOrigin); }
            if (axis.HasValue)
            {
                Vec3 z = axis.Value.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
                var ang = pts.Select(p => Math.Atan2((p - center).Dot(y), (p - center).Dot(x))).ToList();
                double t0, t1; bool full;
                GeoUtil.AngularRange(ang, 1.2, out t0, out t1, out full);
                double cover = full ? 360 : (t1 - t0) * 180 / Math.PI;
                if (cover < 90) w.Add(string.Format("Points cover only {0:0}° of the circumference — diameter and centre are uncertain.", cover));
                else if (cover < 150) w.Add(string.Format("Points cover {0:0}° of the circumference — spread them wider if you can.", cover));
                if (raw is CylinderPrim || raw is ConePrim)
                {
                    var hs = pts.Select(p => (p - center).Dot(z)).ToList();
                    double span = hs.Max() - hs.Min();
                    if (span < Math.Max(0.5, 0.25 * radius)) w.Add("Points span very little height — probe at least two rings to define the axis.");
                }
            }
            if (raw is SpherePrim)
            {
                var s = (SpherePrim)raw;
                Vec3 pole = Vec3.Zero;
                foreach (var p in pts) pole += (p - s.Center).Normalized();
                if (pole.Length > 1e-9)
                {
                    pole = pole.Normalized();
                    double cap = pts.Max(p => Constructions.AngleBetween(pole, p - s.Center, false)) * 180 / Math.PI;
                    if (cap < 30) w.Add(string.Format("Points cover a cap of only ±{0:0}° — the diameter is uncertain.", cap));
                }
            }
            return w;
        }
    }
}
