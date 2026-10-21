using System;
using System.Collections.Generic;
using System.Linq;
using MicroScribe.Core;

namespace MicroScribe.Geometry
{
    /// <summary>Tip (pivot) calibration and automatic primitive classification.</summary>
    public static class Calibration
    {
        /// <summary>
        /// Pivot calibration: the tip is held in a fixed divot while the stylus is swung around.
        /// Solves R_i·t + p_i = c for the tip offset t (stylus frame) and pivot point c (base frame).
        /// </summary>
        public static bool Pivot(IList<Mat4> stylusFrames, out Vec3 tipOffset, out Vec3 pivot, out double rms)
        {
            tipOffset = pivot = Vec3.Zero; rms = double.NaN;
            int n = stylusFrames.Count;
            if (n < 4) return false;
            var rows = new double[3 * n][];
            var rhs = new double[3 * n];
            for (int i = 0; i < n; i++)
            {
                Mat4 m = stylusFrames[i];
                for (int r = 0; r < 3; r++)
                {
                    rows[3 * i + r] = new double[] { m[r, 0], m[r, 1], m[r, 2], r == 0 ? -1 : 0, r == 1 ? -1 : 0, r == 2 ? -1 : 0 };
                    rhs[3 * i + r] = -m[r, 3];
                }
            }
            double[] x;
            if (!LinAlg.LeastSquares(rows, rhs, out x)) return false;
            tipOffset = new Vec3(x[0], x[1], x[2]);
            pivot = new Vec3(x[3], x[4], x[5]);
            double ss = 0;
            foreach (var m in stylusFrames) ss += (m.TransformPoint(tipOffset) - pivot).LengthSquared;
            rms = Math.Sqrt(ss / n);
            return true;
        }

        /// <summary>
        /// Picks the most plausible primitive for a set of probed points: the candidate with the lowest
        /// BIC-style score n·ln(rms²) + k·ln(n), with a tolerance floor so ideal data does not over-fit.
        /// </summary>
        public static FeatureKind Classify(Vec3[] pts, Vec3[] dirs, double noiseFloor, out Primitive best)
        {
            best = null;
            int n = pts.Length;
            if (n < 3) { best = n == 1 ? (Primitive)Fitter.FitPoint(pts) : Fitter.FitLine(pts); return n == 1 ? FeatureKind.Point : FeatureKind.Line; }
            var cands = new List<KeyValuePair<Primitive, int>>();
            Action<Primitive, int> add = (p, k) => { if (p != null && p.Stats != null && !double.IsNaN(p.Stats.Rms)) cands.Add(new KeyValuePair<Primitive, int>(p, k)); };
            // collinear points are a line
            var line = Fitter.FitLine(pts);
            double span = line.TMax - line.TMin;
            if (line.Stats.Rms < Math.Max(noiseFloor * 3, span * 0.002)) { best = line; return FeatureKind.Line; }
            add(Fitter.FitPlane(pts, dirs), 3);
            if (n >= 4) add(Fitter.FitSphere(pts), 4);
            if (n >= 5) add(Fitter.FitCylinder(pts, dirs), 5);
            if (n >= 7) add(Fitter.FitCone(pts, dirs), 6);
            double bestScore = double.MaxValue;
            foreach (var c in cands)
            {
                var p = c.Key;
                // reject degenerate huge radii (these are planes in disguise)
                if (p is SpherePrim && ((SpherePrim)p).Radius > 50 * span) continue;
                if (p is CylinderPrim && ((CylinderPrim)p).Radius > 50 * span) continue;
                if (p is ConePrim && Math.Abs(((ConePrim)p).HalfAngle) < 0.5 * Math.PI / 180) continue; // that is a cylinder
                double rms = Math.Max(p.Stats.Rms, noiseFloor);
                double score = n * Math.Log(rms * rms) + c.Value * Math.Log(n) * 2;
                if (score < bestScore) { bestScore = score; best = p; }
            }
            return best != null ? best.Kind : FeatureKind.Plane;
        }
    }
}
