using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;
using MicroScribe.Geometry;

namespace MicroScribe.Model
{
    public enum OrientationMode { Machine, Features, PrincipalAxes }
    public enum OriginMode { Keep, Feature, BoundingBox }
    public enum Anchor { Min, Center, Max }

    /// <summary>
    /// How the part coordinate system is built. Re-evaluated whenever features change, so a
    /// "corner" or "centre" origin follows the measured part as points are added.
    /// Result: machine → part transform = Translate(-anchor - offset) · QuarterTurns · Orientation.
    /// </summary>
    public sealed class AlignmentRecipe
    {
        public OrientationMode Orientation = OrientationMode.Machine;
        public int PrimaryId, SecondaryId, OriginFeatureId;
        public bool FlipPrimary, FlipSecondary;
        /// <summary>Extra quarter turns applied after the base orientation (about part X, Y, Z).</summary>
        public int TurnsX, TurnsY, TurnsZ;

        public OriginMode Origin = OriginMode.Keep;
        public Anchor AnchorX = Anchor.Center, AnchorY = Anchor.Center, AnchorZ = Anchor.Min;
        /// <summary>Features used for the bounding box (empty = all measured features).</summary>
        public List<int> BoxFeatureIds = new List<int>();
        public double OffsetX, OffsetY, OffsetZ;

        public string Describe(Project p)
        {
            Func<int, string> N = id => { var f = p.Find(id); return f != null ? f.Name : "?"; };
            string o;
            switch (Orientation)
            {
                case OrientationMode.Features:
                    o = "Z: " + N(PrimaryId) + (SecondaryId != 0 ? ", X: " + N(SecondaryId) : ""); break;
                case OrientationMode.PrincipalAxes: o = "principal axes"; break;
                default: o = "machine axes"; break;
            }
            if (TurnsX != 0 || TurnsY != 0 || TurnsZ != 0) o += string.Format(" +rot({0},{1},{2})×90°", TurnsX, TurnsY, TurnsZ);
            string org;
            switch (Origin)
            {
                case OriginMode.Feature: org = "origin " + N(OriginFeatureId); break;
                case OriginMode.BoundingBox:
                    org = AnchorX == Anchor.Center && AnchorY == Anchor.Center && AnchorZ == Anchor.Center
                        ? "origin box centre"
                        : string.Format("origin box {0}/{1}/{2}", AnchorX, AnchorY, AnchorZ);
                    break;
                default: org = Orientation == OrientationMode.Machine ? "machine origin" : "origin from features"; break;
            }
            if (OffsetX != 0 || OffsetY != 0 || OffsetZ != 0) org += string.Format(System.Globalization.CultureInfo.InvariantCulture, " +({0:0.###},{1:0.###},{2:0.###})", OffsetX, OffsetY, OffsetZ);
            return o + "; " + org;
        }
    }

    public static class AlignmentSolver
    {
        public static Mat4 RotX(int quarter) { return Rot(quarter, 0); }
        public static Mat4 RotY(int quarter) { return Rot(quarter, 1); }
        public static Mat4 RotZ(int quarter) { return Rot(quarter, 2); }

        static Mat4 Rot(int quarter, int axis)
        {
            int q = ((quarter % 4) + 4) % 4;
            double c = new[] { 1, 0, -1, 0 }[q], s = new[] { 0, 1, 0, -1 }[q];
            var m = Mat4.Identity();
            int a = (axis + 1) % 3, b = (axis + 2) % 3;
            m[a, a] = c; m[a, b] = -s; m[b, a] = s; m[b, b] = c;
            return m;
        }

        /// <summary>Points that define the part's bounding box (machine frame).</summary>
        public static List<Vec3> BoxPoints(Project p, AlignmentRecipe r)
        {
            var ids = r.BoxFeatureIds;
            var feats = p.Features.Where(f => ids == null || ids.Count == 0 ? f.Origin == FeatureOrigin.Measured : ids.Contains(f.Id));
            var pts = new List<Vec3>();
            foreach (var f in feats)
            {
                pts.AddRange(f.ActivePoints.Select(q => q.Position));
                if (f.Geometry is SpherePrim)
                {
                    // include the full ball, not just the probed cap
                    var s = (SpherePrim)f.Geometry;
                    foreach (var d in new[] { Vec3.UnitX, Vec3.UnitY, Vec3.UnitZ })
                    { pts.Add(s.Center + d * s.Radius); pts.Add(s.Center - d * s.Radius); }
                }
            }
            return pts;
        }

        /// <summary>Orientation-only part (rotation + translation from features) before anchoring.</summary>
        public static Mat4 BaseOrientation(Project p, AlignmentRecipe r, out string error)
        {
            error = null;
            Mat4 m = Mat4.Identity();
            if (r.Orientation == OrientationMode.Features)
            {
                var prim = p.Find(r.PrimaryId);
                if (prim == null || prim.Geometry == null || !Constructions.Direction(prim.Geometry).HasValue)
                { error = "Primary feature needs a direction (plane, line, cylinder, cone, circle)"; return m; }
                var sec = p.Find(r.SecondaryId);
                var org = p.Find(r.OriginFeatureId);
                m = Constructions.Alignment(prim.Geometry, sec != null ? sec.Geometry : null,
                    org != null && r.Origin == OriginMode.Feature ? org.Geometry : null, r.FlipPrimary, r.FlipSecondary);
            }
            else if (r.Orientation == OrientationMode.PrincipalAxes)
            {
                var pts = BoxPoints(p, r);
                if (pts.Count >= 3)
                {
                    double[,] s;
                    Vec3 c = LinAlg.Scatter(pts.ToArray(), out s);
                    double[] ev; Vec3[] vec;
                    LinAlg.SymmetricEigen3(s, out ev, out vec);
                    Vec3 x = vec[2], z = vec[0];
                    // keep the result close to the machine frame (deterministic signs)
                    if (z.Z < 0) z = -z;
                    if (x.X < 0) x = -x;
                    Vec3 y = z.Cross(x).Normalized();
                    x = y.Cross(z).Normalized();
                    if (r.FlipPrimary) { z = -z; y = -y; }
                    m = Mat4.FromFrame(c, x, y, z).RigidInverse();
                }
            }
            Mat4 turns = RotZ(r.TurnsZ) * RotY(r.TurnsY) * RotX(r.TurnsX);
            // turns rotate the part frame: part' = turns^-1 · part
            return turns.RigidInverse() * m;
        }

        /// <summary>Full machine→part transform; also returns the bounding box in the final frame.</summary>
        public static Mat4 Solve(Project p, AlignmentRecipe r, out Vec3 boxMin, out Vec3 boxMax, out string error)
        {
            Mat4 m = BaseOrientation(p, r, out error);
            var pts = BoxPoints(p, r).Select(q => m.TransformPoint(q)).ToList();
            Bounds(pts, out boxMin, out boxMax);
            Vec3 anchor = Vec3.Zero;
            if (r.Origin == OriginMode.BoundingBox && pts.Count > 0)
            {
                anchor = new Vec3(Pick(r.AnchorX, boxMin.X, boxMax.X), Pick(r.AnchorY, boxMin.Y, boxMax.Y), Pick(r.AnchorZ, boxMin.Z, boxMax.Z));
            }
            else if (r.Origin == OriginMode.Feature && r.Orientation != OrientationMode.Features)
            {
                var of = p.Find(r.OriginFeatureId);
                if (of != null && of.Geometry != null) anchor = m.TransformPoint(Constructions.KeyPoint(of.Geometry));
            }
            anchor += new Vec3(r.OffsetX, r.OffsetY, r.OffsetZ);
            Mat4 t = Mat4.Translation(-anchor.X, -anchor.Y, -anchor.Z);
            boxMin -= anchor; boxMax -= anchor;
            return t * m;
        }

        static double Pick(Anchor a, double min, double max)
        {
            return a == Anchor.Min ? min : a == Anchor.Max ? max : 0.5 * (min + max);
        }

        public static void Bounds(IEnumerable<Vec3> pts, out Vec3 min, out Vec3 max)
        {
            min = new Vec3(double.MaxValue, double.MaxValue, double.MaxValue);
            max = new Vec3(double.MinValue, double.MinValue, double.MinValue);
            bool any = false;
            foreach (var q in pts)
            {
                any = true;
                min = new Vec3(Math.Min(min.X, q.X), Math.Min(min.Y, q.Y), Math.Min(min.Z, q.Z));
                max = new Vec3(Math.Max(max.X, q.X), Math.Max(max.Y, q.Y), Math.Max(max.Z, q.Z));
            }
            if (!any) { min = max = Vec3.Zero; }
        }

        /// <summary>Applies the project's recipe (call after solving features).</summary>
        public static void Apply(Project p)
        {
            if (p.AlignmentRecipe == null) p.AlignmentRecipe = new AlignmentRecipe();
            Vec3 a, b; string err;
            p.AlignmentMatrix = Solve(p, p.AlignmentRecipe, out a, out b, out err);
            p.AlignmentError = err;
            p.AlignmentDescription = p.AlignmentRecipe.Describe(p);
        }
    }
}
