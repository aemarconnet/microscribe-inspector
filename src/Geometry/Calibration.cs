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
        /// Probe qualification on a reference sphere: the fit of the ball centres gives the effective ball
        /// radius as (fitted radius − reference radius) when probed from outside. Returns NaN for a fit
        /// that is not a sphere.
        /// </summary>
        public static double QualifiedBallRadius(Primitive ballCentreFit, double referenceDiameter)
        {
            var s = ballCentreFit as SpherePrim;
            if (s == null || referenceDiameter <= 0) return double.NaN;
            return s.Radius - referenceDiameter / 2;
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

    /// <summary>Result of an arm check (single-point articulation or length test), in mm.</summary>
    public sealed class ArmCheckResult
    {
        public int N;
        /// <summary>Single point: largest distance of a reading from the mean. Length: largest |error|.</summary>
        public double MaxDeviation;
        /// <summary>Single point: largest distance between two readings. Length: max − min of the errors.</summary>
        public double Range;
        /// <summary>Two standard deviations (single point: of the distances from the mean, n − 1).</summary>
        public double TwoSigma;
        /// <summary>Length test: mean error (measured − reference).</summary>
        public double MeanError;
        public Vec3 Mean;
    }

    /// <summary>
    /// Arm performance checks in the spirit of ASME B89.4.22: the single-point articulation test (the tip
    /// stays in one cone seat while the arm is moved through different poses) and a length test (a known
    /// distance, e.g. between two cone seats on a bar, measured at several places in the working volume).
    /// </summary>
    public static class ArmCheck
    {
        /// <summary>
        /// Largest distance between any two tip readings: how far the stylus moved during a capture.
        /// </summary>
        public static double Movement(IList<Vec3> tips)
        {
            double m = 0;
            for (int i = 0; i < tips.Count; i++)
                for (int j = i + 1; j < tips.Count; j++) m = Math.Max(m, (tips[i] - tips[j]).Length);
            return m;
        }

        /// <summary>
        /// True when the stylus was held still: every tip reading within <paramref name="radius"/> of the
        /// mean and every stylus direction within <paramref name="maxAngleDeg"/> of the mean direction.
        /// </summary>
        public static bool IsStill(IList<Vec3> tips, IList<Vec3> dirs, double radius, double maxAngleDeg, out Vec3 tip, out Vec3 dir)
        {
            tip = dir = Vec3.Zero;
            if (tips.Count == 0) return false;
            foreach (var p in tips) tip += p;
            foreach (var d in dirs) dir += d;
            tip = tip / tips.Count;
            dir = dir.Normalized();
            Vec3 c = tip, a = dir;
            double maxAng = maxAngleDeg * Math.PI / 180;
            return tips.All(p => (p - c).Length <= radius) && dirs.All(d => Constructions.AngleBetween(d, a, false) <= maxAng);
        }

        public static ArmCheckResult SinglePoint(IList<Vec3> readings)
        {
            var r = new ArmCheckResult { N = readings.Count };
            if (r.N == 0) return r;
            Vec3 c = Vec3.Zero;
            foreach (var p in readings) c += p;
            c = c / r.N;
            r.Mean = c;
            double ss = 0;
            foreach (var p in readings)
            {
                double d = (p - c).Length;
                r.MaxDeviation = Math.Max(r.MaxDeviation, d);
                ss += d * d;
            }
            for (int i = 0; i < r.N; i++)
                for (int j = i + 1; j < r.N; j++) r.Range = Math.Max(r.Range, (readings[i] - readings[j]).Length);
            r.TwoSigma = r.N > 1 ? 2 * Math.Sqrt(ss / (r.N - 1)) : 0;
            return r;
        }

        public static ArmCheckResult Length(IList<double> measured, double reference)
        {
            var r = new ArmCheckResult { N = measured.Count };
            if (r.N == 0) return r;
            var e = measured.Select(m => m - reference).ToList();
            r.MeanError = e.Average();
            r.MaxDeviation = e.Max(x => Math.Abs(x));
            r.Range = e.Max() - e.Min();
            r.TwoSigma = r.N > 1 ? 2 * Math.Sqrt(e.Sum(x => (x - r.MeanError) * (x - r.MeanError)) / (r.N - 1)) : 0;
            return r;
        }
    }
}
