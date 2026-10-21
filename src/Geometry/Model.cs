using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Serialization;
using MicroScribe.Core;
using MicroScribe.Geometry;

namespace MicroScribe.Model
{
    /// <summary>A probed point: tip centre plus stylus direction at the moment of capture (machine frame, mm).</summary>
    public sealed class MeasuredPoint
    {
        public double X, Y, Z;
        public double DX, DY, DZ;
        /// <summary>Probe ball radius at capture time (0 for a pointed tip).</summary>
        public double ProbeRadius;
        public bool Excluded;
        public DateTime Time;
        /// <summary>Freeform surfaces: index of the traced section curve this point belongs to.</summary>
        public int Section;

        [XmlIgnore] public Vec3 Position { get { return new Vec3(X, Y, Z); } set { X = value.X; Y = value.Y; Z = value.Z; } }
        [XmlIgnore] public Vec3 Direction { get { return new Vec3(DX, DY, DZ); } set { DX = value.X; DY = value.Y; DZ = value.Z; } }

        public MeasuredPoint() { }
        public MeasuredPoint(Vec3 p, Vec3 d, double probeRadius)
        {
            Position = p; Direction = d; ProbeRadius = probeRadius; Time = DateTime.Now;
        }
    }

    public enum CompensationSide { Auto, Outside, Inside, None }

    public enum FeatureOrigin { Measured, Constructed }

    /// <summary>A measured or constructed feature.</summary>
    public sealed class Feature
    {
        public int Id;
        public string Name;
        public FeatureKind Kind;
        public FeatureOrigin Origin;
        public List<MeasuredPoint> Points = new List<MeasuredPoint>();
        public bool Visible = true;
        public bool Export = true;
        public CompensationSide Compensation = CompensationSide.Auto;
        /// <summary>Extra length added around the measured extent when exporting surfaces (mm).</summary>
        public double ExportMargin = 2.0;
        /// <summary>For circles: project onto this plane feature (0 = own best-fit plane).</summary>
        public int ProjectionPlaneId;
        /// <summary>For cylinders/cones: force a full 360° surface on export.</summary>
        public bool FullRevolution;
        /// <summary>For constructions: ids of the source features and the operation.</summary>
        public List<int> SourceIds = new List<int>();
        public string Construction;
        public string Notes = "";

        // --- curves
        public CurveMode CurveMode = CurveMode.Spline;
        /// <summary>Spline smoothing: max deviation from the traced points (mm); 0 = pass through every point.</summary>
        public double CurveTolerance = 0.1;
        public bool Closed;

        // --- planes: user-resized rectangle (machine frame), re-projected onto the plane after refits
        public bool CustomBounds;
        public double BCX, BCY, BCZ, BUX = 1, BUY, BUZ, BW = 50, BH = 50;
        [XmlIgnore] public Vec3 BoundsCenter { get { return new Vec3(BCX, BCY, BCZ); } set { BCX = value.X; BCY = value.Y; BCZ = value.Z; } }
        [XmlIgnore] public Vec3 BoundsU { get { return new Vec3(BUX, BUY, BUZ); } set { BUX = value.X; BUY = value.Y; BUZ = value.Z; } }

        // --- freeform surfaces
        /// <summary>Control points along each traced curve (0 = automatic). Fewer = smoother.</summary>
        public int SurfaceCtrlU;

        /// <summary>Fit of the probe-tip centres (before compensation).</summary>
        [XmlIgnore] public Primitive RawFit;
        /// <summary>Final geometry after probe compensation (what is reported and exported).</summary>
        [XmlIgnore] public Primitive Geometry;
        [XmlIgnore] public string FitError;

        [XmlIgnore] public IEnumerable<MeasuredPoint> ActivePoints { get { return Points.Where(p => !p.Excluded); } }

        [XmlIgnore] public string KindLabel { get { return Kind.ToString(); } }

        public override string ToString() { return Name; }
    }

    /// <summary>Whole measurement session.</summary>
    public sealed class Project
    {
        public string Title = "Untitled";
        public int NextId = 1;
        public List<Feature> Features = new List<Feature>();
        /// <summary>Machine→part alignment (row-major 16 doubles), identity when not aligned.</summary>
        public double[] Alignment;
        public string AlignmentDescription = "Machine (arm base)";
        public AlignmentRecipe AlignmentRecipe = new AlignmentRecipe();
        [XmlIgnore] public string AlignmentError;
        public string Units = "mm";
        public string Notes = "";
        public List<Dimension> Dimensions = new List<Dimension>();
        public int NextDimId = 1;

        [XmlIgnore]
        public Mat4 AlignmentMatrix
        {
            get
            {
                var m = Mat4.Identity();
                if (Alignment != null && Alignment.Length == 16) Array.Copy(Alignment, m.M, 16);
                return m;
            }
            set { Alignment = (double[])value.M.Clone(); }
        }

        public Feature Find(int id) { return Features.FirstOrDefault(f => f.Id == id); }

        public Feature NewFeature(FeatureKind kind, string baseName = null)
        {
            string b = baseName ?? kind.ToString();
            int n = 1;
            string name = b + " " + n;
            while (Features.Any(f => f.Name == name)) name = b + " " + (++n);
            var feat = new Feature { Id = NextId++, Name = name, Kind = kind };
            Features.Add(feat);
            return feat;
        }
    }

    /// <summary>Fitting + probe compensation for features.</summary>
    public static class FeatureSolver
    {
        public static void Solve(Project proj, Feature f)
        {
            f.FitError = null;
            if (f.Origin == FeatureOrigin.Constructed) { SolveConstruction(proj, f); return; }
            var pts = f.ActivePoints.ToList();
            int need = Fitter.MinPoints(f.Kind);
            if (pts.Count < need)
            {
                f.RawFit = f.Geometry = null;
                f.FitError = string.Format("{0} of {1} points", pts.Count, need);
                return;
            }
            if (f.Kind == FeatureKind.Curve)
            {
                try { f.RawFit = f.Geometry = CurvePrim.Build(pts.Select(q => q.Position).ToList(), f.CurveMode, f.CurveTolerance, f.Closed); }
                catch (Exception ex) { f.RawFit = f.Geometry = null; f.FitError = ex.Message; }
                return;
            }
            if (f.Kind == FeatureKind.Surface)
            {
                var sections = pts.GroupBy(q => q.Section).OrderBy(g => g.Key).Select(g => g.Select(q => q.Position).ToList()).ToList();
                string err;
                f.RawFit = f.Geometry = Loft.Build(sections, f.SurfaceCtrlU, out err);
                f.FitError = err;
                return;
            }
            Vec3[] p = pts.Select(q => q.Position).ToArray();
            Vec3[] d = pts.Select(q => q.Direction).ToArray();
            PlanePrim proj2 = null;
            if (f.Kind == FeatureKind.Circle && f.ProjectionPlaneId != 0)
            {
                var pf = proj.Find(f.ProjectionPlaneId);
                if (pf != null) proj2 = pf.Geometry as PlanePrim;
            }
            try
            {
                Primitive raw = f.Kind == FeatureKind.Plane ? Fitter.FitPlane(p, d) : Fitter.Fit(f.Kind, p, d, proj2);
                f.RawFit = raw;
                if (raw == null) { f.Geometry = null; f.FitError = "Fit failed"; return; }
                double r = pts.Average(q => q.ProbeRadius);
                f.Geometry = Compensate(raw, f, pts, r);
            }
            catch (Exception ex)
            {
                f.RawFit = f.Geometry = null;
                f.FitError = ex.Message;
            }
        }

        /// <summary>
        /// Converts a fit of ball-centre positions into the probed surface by offsetting by the ball radius.
        /// Auto side: the material lies in the direction the stylus points.
        /// </summary>
        public static Primitive Compensate(Primitive raw, Feature f, List<MeasuredPoint> pts, double r)
        {
            if (r <= 0 || f.Compensation == CompensationSide.None || raw is PointPrim || raw is LinePrim) return raw;
            Vec3 avgDir = Vec3.Zero;
            foreach (var q in pts) avgDir += q.Direction;
            Primitive g = raw.Clone();
            if (g is PlanePrim)
            {
                var pl = (PlanePrim)g;
                // Normal is oriented toward the probe; material is on the -normal side.
                double sign = f.Compensation == CompensationSide.Inside ? 1 : -1;
                if (f.Compensation == CompensationSide.Auto && avgDir.Dot(pl.Normal) > 0) sign = 1;
                pl.Origin = pl.Origin + pl.Normal * (sign * r);
            }
            else
            {
                bool outside = IsOutside(raw, pts, f.Compensation);
                double dr = outside ? -r : r;
                if (g is CylinderPrim) ((CylinderPrim)g).Radius += dr;
                else if (g is SpherePrim) ((SpherePrim)g).Radius += dr;
                else if (g is CirclePrim) ((CirclePrim)g).Radius += dr;
                else if (g is ConePrim)
                {
                    var c = (ConePrim)g;
                    c.RadiusAtOrigin += dr / Math.Cos(c.HalfAngle);
                }
            }
            g.Stats = raw.Stats;
            return g;
        }

        /// <summary>True when probing a boss/ball from outside (stylus pointing toward the axis/centre).</summary>
        public static bool IsOutside(Primitive raw, List<MeasuredPoint> pts, CompensationSide side)
        {
            if (side == CompensationSide.Outside) return true;
            if (side == CompensationSide.Inside) return false;
            double vote = 0;
            foreach (var q in pts)
            {
                Vec3 radial;
                if (raw is SpherePrim) radial = q.Position - ((SpherePrim)raw).Center;
                else if (raw is CirclePrim)
                {
                    var c = (CirclePrim)raw; Vec3 w = q.Position - c.Center; radial = w - c.Normal * w.Dot(c.Normal);
                }
                else if (raw is CylinderPrim)
                {
                    var c = (CylinderPrim)raw; Vec3 w = q.Position - c.AxisPoint; radial = w - c.Axis * w.Dot(c.Axis);
                }
                else
                {
                    var c = (ConePrim)raw; Vec3 w = q.Position - c.AxisPoint; radial = w - c.Axis * w.Dot(c.Axis);
                }
                vote += -q.Direction.Dot(radial.Normalized());
            }
            return vote >= 0;
        }

        static void SolveConstruction(Project proj, Feature f)
        {
            var src = f.SourceIds.Select(id => proj.Find(id)).ToList();
            if (src.Any(s => s == null || s.Geometry == null))
            {
                f.Geometry = null; f.FitError = "Source feature missing or not solved"; return;
            }
            Primitive g = null;
            switch (f.Construction)
            {
                case "intersect":
                    g = src.Count == 3 && src.All(s => s.Geometry is PlanePrim)
                        ? Constructions.Intersect((PlanePrim)src[0].Geometry, (PlanePrim)src[1].Geometry, (PlanePrim)src[2].Geometry)
                        : Constructions.Intersect(src[0].Geometry, src[1].Geometry);
                    break;
                case "midpoint":
                    g = new PointPrim { Position = (Constructions.KeyPoint(src[0].Geometry) + Constructions.KeyPoint(src[1].Geometry)) * 0.5 };
                    break;
                case "axis":
                    {
                        var s = src[0].Geometry;
                        var dir = Constructions.Direction(s);
                        if (dir.HasValue)
                        {
                            double h0 = -25, h1 = 25;
                            if (s is CylinderPrim) { h0 = ((CylinderPrim)s).HMin; h1 = ((CylinderPrim)s).HMax; }
                            if (s is ConePrim) { h0 = ((ConePrim)s).HMin; h1 = ((ConePrim)s).HMax; }
                            g = new LinePrim { Origin = Constructions.KeyPoint(s), Direction = dir.Value, TMin = h0, TMax = h1 };
                        }
                        break;
                    }
                case "center":
                    g = new PointPrim { Position = Constructions.KeyPoint(src[0].Geometry) };
                    break;
                case "line2":
                    {
                        Vec3 a = Constructions.KeyPoint(src[0].Geometry), b = Constructions.KeyPoint(src[1].Geometry);
                        g = new LinePrim { Origin = a, Direction = (b - a).Normalized(), TMin = 0, TMax = (b - a).Length };
                        break;
                    }
                case "loft":
                    {
                        // freeform surface through the traced curves, in the given order
                        var sections = src.Select(s => s.Geometry as CurvePrim).Where(c => c != null)
                            .Select(c => c.Samples.ToList()).ToList();
                        var raw = src.Select(s => s.ActivePoints.Select(q => q.Position).ToList()).ToList();
                        string err;
                        var surf = Loft.Build(sections, f.SurfaceCtrlU, out err);
                        if (surf == null) { f.FitError = err; f.Geometry = f.RawFit = null; return; }
                        surf.Stats = FitStats.From(raw.SelectMany(x => x).Select(x => surf.SignedDistance(x)).ToArray());
                        f.Kind = FeatureKind.Surface;
                        f.RawFit = f.Geometry = surf;
                        return;
                    }
            }
            if (g == null) { f.FitError = "Construction not possible (parallel?)"; f.Geometry = null; return; }
            if (g is LinePrim && f.Construction == "intersect")
            {
                // size the intersection line by the probed extent of its source features
                var l = (LinePrim)g;
                var pts = src.SelectMany(s => s.ActivePoints.Select(q => q.Position)).ToList();
                if (pts.Count > 0)
                {
                    double t0 = pts.Min(q => (q - l.Origin).Dot(l.Direction)), t1 = pts.Max(q => (q - l.Origin).Dot(l.Direction));
                    l.TMin = t0; l.TMax = t1;
                }
            }
            if (g is LinePrim) f.Kind = FeatureKind.Line;
            if (g is PointPrim) f.Kind = FeatureKind.Point;
            g.Stats = FitStats.From(new double[0]);
            f.RawFit = f.Geometry = g;
        }

        public static void SolveAll(Project proj)
        {
            // measured first (circles last: they may project onto planes), then constructions, then alignment
            foreach (var f in proj.Features.Where(x => x.Origin == FeatureOrigin.Measured && x.Kind != FeatureKind.Circle)) Solve(proj, f);
            foreach (var f in proj.Features.Where(x => x.Origin == FeatureOrigin.Measured && x.Kind == FeatureKind.Circle)) Solve(proj, f);
            foreach (var f in proj.Features.Where(x => x.Origin == FeatureOrigin.Constructed)) Solve(proj, f);
            AlignmentSolver.Apply(proj);
        }
    }

    /// <summary>Rectangular extent of plane features (used for display, resizing and export).</summary>
    public static class PlaneBounds
    {
        /// <summary>Four corners (machine frame, CCW about the plane normal).</summary>
        public static Vec3[] Corners(Feature f, PlanePrim pl)
        {
            Vec3 c, u; double w, h;
            Get(f, pl, out c, out u, out w, out h);
            Vec3 v = pl.Normal.Cross(u);
            return new[] { c - u * (w / 2) - v * (h / 2), c + u * (w / 2) - v * (h / 2), c + u * (w / 2) + v * (h / 2), c - u * (w / 2) + v * (h / 2) };
        }

        /// <summary>Centre, in-plane U axis and size of the rectangle (custom, or fitted to the points + margin).</summary>
        public static void Get(Feature f, PlanePrim pl, out Vec3 center, out Vec3 u, out double w, out double h)
        {
            if (f.CustomBounds)
            {
                center = pl.Project(f.BoundsCenter);
                u = f.BoundsU - pl.Normal * f.BoundsU.Dot(pl.Normal);
                u = u.Length > 1e-9 ? u.Normalized() : pl.Normal.AnyPerpendicular();
                w = Math.Max(0.1, f.BW); h = Math.Max(0.1, f.BH);
                return;
            }
            FitToPoints(f, pl, f.ExportMargin, out center, out u, out w, out h);
        }

        public static void FitToPoints(Feature f, PlanePrim pl, double margin, out Vec3 center, out Vec3 u, out double w, out double h)
        {
            Vec3 a = pl.Normal.AnyPerpendicular(), b = pl.Normal.Cross(a);
            var p2 = f.ActivePoints.Select(q => q.Position).Select(p => new[] { (p - pl.Origin).Dot(a), (p - pl.Origin).Dot(b) }).ToList();
            if (p2.Count == 0) p2.Add(new[] { 0.0, 0.0 });
            var r = GeoUtil.MinAreaRectangle(p2, margin);
            Func<double[], Vec3> P = q => pl.Origin + a * q[0] + b * q[1];
            Vec3 c0 = P(r[0]), c1 = P(r[1]), c3 = P(r[3]);
            u = (c1 - c0).Normalized();
            w = (c1 - c0).Length; h = (c3 - c0).Length;
            center = (P(r[0]) + P(r[2])) * 0.5;
            // make (u, normal x u) right-handed with the polygon order
            if (pl.Normal.Cross(u).Dot(c3 - c0) < 0) u = -u;
        }

        /// <summary>Stores an explicit rectangle (switches the feature to custom bounds).</summary>
        public static void Set(Feature f, Vec3 center, Vec3 u, double w, double h)
        {
            f.CustomBounds = true;
            f.BoundsCenter = center; f.BoundsU = u.Normalized();
            f.BW = Math.Max(0.1, w); f.BH = Math.Max(0.1, h);
        }

        /// <summary>
        /// Extends/trims the rectangle so one edge lies on the intersection line with another plane.
        /// The rectangle is rotated to be parallel to that line. Returns false if the planes are parallel.
        /// </summary>
        public static bool ExtendTo(Feature f, PlanePrim pl, PlanePrim other)
        {
            var line = Constructions.Intersect(pl, other);
            if (line == null) return false;
            Vec3 c, u0; double w0, h0;
            Get(f, pl, out c, out u0, out w0, out h0);
            var corners = Corners(f, pl);
            Vec3 u = line.Direction - pl.Normal * line.Direction.Dot(pl.Normal);
            u = u.Normalized();
            Vec3 v = pl.Normal.Cross(u);
            // current extent in the new (u, v) axes
            double umin = double.MaxValue, umax = double.MinValue, vmin = double.MaxValue, vmax = double.MinValue;
            foreach (var p in corners)
            {
                double s = (p - c).Dot(u), t = (p - c).Dot(v);
                umin = Math.Min(umin, s); umax = Math.Max(umax, s); vmin = Math.Min(vmin, t); vmax = Math.Max(vmax, t);
            }
            double tl = (line.Origin - c).Dot(v); // line position across the rectangle
            if (Math.Abs(tl - vmin) < Math.Abs(tl - vmax)) vmin = tl; else vmax = tl;
            if (vmax - vmin < 0.1) return false;
            Vec3 center = c + u * ((umin + umax) / 2) + v * ((vmin + vmax) / 2);
            Set(f, center, u, umax - umin, vmax - vmin);
            return true;
        }
    }
}
