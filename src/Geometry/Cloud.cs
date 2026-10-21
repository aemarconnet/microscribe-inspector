using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>A scanned point cloud (contact points, probe ball removed). No surface is fitted.</summary>
    public sealed class CloudPrim : Primitive
    {
        public Vec3[] Points = new Vec3[0];
        public override FeatureKind Kind { get { return FeatureKind.Cloud; } }
        /// <summary>Distance to the nearest cloud point (unsigned).</summary>
        public override double SignedDistance(Vec3 p)
        {
            double best = double.MaxValue;
            foreach (var q in Points) best = Math.Min(best, (q - p).LengthSquared);
            return Points.Length > 0 ? Math.Sqrt(best) : double.NaN;
        }
        public override Primitive Clone() { var c = (CloudPrim)MemberwiseClone(); c.Points = (Vec3[])Points.Clone(); return c; }
        public CloudPrim Transformed(Mat4 m) { var c = (CloudPrim)Clone(); c.Points = Points.Select(m.TransformPoint).ToArray(); return c; }
        public void Bounds(out Vec3 min, out Vec3 max)
        {
            min = new Vec3(double.MaxValue, double.MaxValue, double.MaxValue); max = new Vec3(double.MinValue, double.MinValue, double.MinValue);
            foreach (var p in Points)
            {
                min = new Vec3(Math.Min(min.X, p.X), Math.Min(min.Y, p.Y), Math.Min(min.Z, p.Z));
                max = new Vec3(Math.Max(max.X, p.X), Math.Max(max.Y, p.Y), Math.Max(max.Z, p.Z));
            }
        }
    }

    /// <summary>Neighbour searches on scanned points through a uniform grid (voxel hash).</summary>
    public sealed class PointGrid
    {
        readonly double _cell;
        readonly Dictionary<long, List<int>> _cells = new Dictionary<long, List<int>>();
        readonly IList<Vec3> _pts;

        public PointGrid(IList<Vec3> pts, double cell)
        {
            _pts = pts; _cell = Math.Max(1e-6, cell);
            for (int i = 0; i < pts.Count; i++)
            {
                long k = Key(pts[i]);
                List<int> l;
                if (!_cells.TryGetValue(k, out l)) _cells[k] = l = new List<int>();
                l.Add(i);
            }
        }

        long Key(long x, long y, long z) { return ((x & 0x1FFFFF) << 42) | ((y & 0x1FFFFF) << 21) | (z & 0x1FFFFF); }
        long Key(Vec3 p) { return Key((long)Math.Floor(p.X / _cell), (long)Math.Floor(p.Y / _cell), (long)Math.Floor(p.Z / _cell)); }

        /// <summary>Indices of the points within <paramref name="r"/> (r ≤ cell size) of p.</summary>
        public List<int> Near(Vec3 p, double r)
        {
            var res = new List<int>();
            long cx = (long)Math.Floor(p.X / _cell), cy = (long)Math.Floor(p.Y / _cell), cz = (long)Math.Floor(p.Z / _cell);
            double r2 = r * r;
            for (long x = cx - 1; x <= cx + 1; x++)
                for (long y = cy - 1; y <= cy + 1; y++)
                    for (long z = cz - 1; z <= cz + 1; z++)
                    {
                        List<int> l;
                        if (!_cells.TryGetValue(Key(x, y, z), out l)) continue;
                        foreach (int i in l) if ((_pts[i] - p).LengthSquared <= r2) res.Add(i);
                    }
            return res;
        }
    }

    public enum ScanMode { Distance, Time, Sections }

    /// <summary>
    /// Decides which samples a hard-probe scan keeps: every <see cref="Spacing"/> mm of travel, every
    /// <see cref="IntervalMs"/>, or a point wherever the stylus crosses one of a family of parallel
    /// planes (part-frame axis <see cref="Axis"/>, every <see cref="SectionSpacing"/> mm) — interpolated
    /// between the two samples on either side of the plane.
    /// </summary>
    public sealed class ScanSampler
    {
        public ScanMode Mode;
        public double Spacing = 1, IntervalMs = 100, SectionSpacing = 5;
        public int Axis;
        public Mat4 ToPart = Mat4.Identity();

        Vec3 _lastKept; double _lastKeptMs = double.NaN;
        Vec3 _prevTip, _prevDir; double _prevC = double.NaN;

        public sealed class Sample { public Vec3 Tip, Direction; public int Section = -1; }

        public void Reset() { _lastKeptMs = double.NaN; _prevC = double.NaN; }

        /// <summary>Machine-frame normal of the section planes.</summary>
        public Vec3 PlaneNormal
        {
            get { Vec3 a = Axis == 0 ? Vec3.UnitX : Axis == 1 ? Vec3.UnitY : Vec3.UnitZ; return ToPart.RigidInverse().TransformVector(a).Normalized(); }
        }

        double Coord(Vec3 tip) { Vec3 q = ToPart.TransformPoint(tip); return Axis == 0 ? q.X : Axis == 1 ? q.Y : q.Z; }

        /// <summary>Feeds one arm sample (time in ms); returns the points to record.</summary>
        public List<Sample> Feed(Vec3 tip, Vec3 dir, double ms)
        {
            var res = new List<Sample>();
            if (Mode == ScanMode.Sections)
            {
                double s = Math.Max(0.01, SectionSpacing), c = Coord(tip);
                if (!double.IsNaN(_prevC))
                {
                    long k0 = (long)Math.Floor(_prevC / s), k1 = (long)Math.Floor(c / s);
                    if (k0 != k1 && Math.Abs(c - _prevC) > 1e-12)
                    {
                        long lo = Math.Min(k0, k1) + 1, hi = Math.Max(k0, k1);
                        for (long j = lo; j <= hi; j++)
                        {
                            long jj = k1 > k0 ? j : hi - (j - lo);   // in travel order
                            double t = (jj * s - _prevC) / (c - _prevC);
                            res.Add(new Sample { Tip = _prevTip + (tip - _prevTip) * t, Direction = (_prevDir + (dir - _prevDir) * t).Normalized(), Section = (int)jj });
                        }
                    }
                }
                _prevTip = tip; _prevDir = dir; _prevC = c;
                return res;
            }
            double iv = Math.Max(10, IntervalMs);
            bool keep = double.IsNaN(_lastKeptMs)
                || (Mode == ScanMode.Distance && (tip - _lastKept).Length >= Math.Max(0.05, Spacing))
                || (Mode == ScanMode.Time && ms >= _lastKeptMs + iv);
            if (keep)
            {
                // time mode keeps a fixed rhythm (the next point is due one interval after the last due time)
                _lastKeptMs = Mode == ScanMode.Time && !double.IsNaN(_lastKeptMs) ? _lastKeptMs + iv * Math.Floor((ms - _lastKeptMs) / iv) : ms;
                _lastKept = tip;
                res.Add(new Sample { Tip = tip, Direction = dir });
            }
            return res;
        }
    }

    public static class CloudTools
    {
        /// <summary>
        /// Contact points of scanned ball centres: each is moved by its ball radius along the local surface
        /// normal (plane through its neighbours within <paramref name="radius"/>), on the side the stylus
        /// points to; isolated points fall back to the stylus direction.
        /// </summary>
        public static List<Vec3> Compensate(IList<Vec3> centres, IList<Vec3> dirs, IList<double> radii, double radius)
        {
            var res = new List<Vec3>(centres.Count);
            var grid = new PointGrid(centres, radius);
            for (int i = 0; i < centres.Count; i++)
            {
                double r = radii[i];
                if (r <= 0) { res.Add(centres[i]); continue; }
                Vec3 d = dirs[i].Normalized(), n = d;
                var nb = grid.Near(centres[i], radius);
                if (nb.Count >= 5)
                {
                    double[,] s;
                    LinAlg.Scatter(nb.Select(j => centres[j]).ToArray(), out s);
                    double[] ev; Vec3[] vec;
                    LinAlg.SymmetricEigen3(s, out ev, out vec);
                    // a flat neighbourhood (smallest spread well below the others) gives a usable normal
                    if (ev[0] < 0.25 * ev[1]) { n = vec[0]; if (n.Dot(d) < 0) n = -n; }
                }
                res.Add(centres[i] + n * r);
            }
            return res;
        }

        /// <summary>Indices to keep so that no two kept points are closer than <paramref name="spacing"/> (scan order wins).</summary>
        public static List<int> Thin(IList<Vec3> pts, double spacing)
        {
            var kept = new List<Vec3>();
            var keep = new List<int>();
            var grid = new Dictionary<long, List<Vec3>>();
            double c = Math.Max(1e-6, spacing);
            Func<long, long, long, long> key = (x, y, z) => ((x & 0x1FFFFF) << 42) | ((y & 0x1FFFFF) << 21) | (z & 0x1FFFFF);
            for (int i = 0; i < pts.Count; i++)
            {
                Vec3 p = pts[i];
                long cx = (long)Math.Floor(p.X / c), cy = (long)Math.Floor(p.Y / c), cz = (long)Math.Floor(p.Z / c);
                bool close = false;
                for (long x = cx - 1; x <= cx + 1 && !close; x++)
                    for (long y = cy - 1; y <= cy + 1 && !close; y++)
                        for (long z = cz - 1; z <= cz + 1 && !close; z++)
                        {
                            List<Vec3> l;
                            if (grid.TryGetValue(key(x, y, z), out l) && l.Any(q => (q - p).LengthSquared < spacing * spacing)) close = true;
                        }
                if (close) continue;
                List<Vec3> cell;
                long k = key(cx, cy, cz);
                if (!grid.TryGetValue(k, out cell)) grid[k] = cell = new List<Vec3>();
                cell.Add(p);
                keep.Add(i);
            }
            return keep;
        }

        /// <summary>Indices of points with fewer than <paramref name="minNeighbours"/> others within <paramref name="radius"/> (stray hits).</summary>
        public static List<int> Isolated(IList<Vec3> pts, double radius, int minNeighbours)
        {
            var grid = new PointGrid(pts, radius);
            var res = new List<int>();
            for (int i = 0; i < pts.Count; i++) if (grid.Near(pts[i], radius).Count - 1 < minNeighbours) res.Add(i);
            return res;
        }

        /// <summary>
        /// Common direction along the sections of a section-plane scan (longest in-plane extent). The
        /// crossings arrive in the order the stylus swept back and forth; sorting each section along this
        /// direction gives curves that can be lofted.
        /// </summary>
        public static Vec3 SweepDirection(IEnumerable<Vec3> pts, Vec3 planeNormal)
        {
            Vec3 n = planeNormal.Normalized();
            var flat = pts.Select(p => p - n * p.Dot(n)).ToArray();
            if (flat.Length < 3) return n.AnyPerpendicular();
            double[,] s2;
            LinAlg.Scatter(flat, out s2);
            double[] ev; Vec3[] vec;
            LinAlg.SymmetricEigen3(s2, out ev, out vec);
            return vec[2];
        }
    }
}
