using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MicroScribe.Core;
using MicroScribe.Export;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.Tests
{
    /// <summary>Inspection against nominal values: tolerances, GD&amp;T, reports.</summary>
    static class InspectionTests
    {
        public static void Register()
        {
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("tolerances: pass/fail, positions, report, XML", TestTolerances));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("GD&T: form, orientation, position, coaxiality", TestGdt));
        }

        static Feature Pts(Project p, FeatureKind k, IEnumerable<Vec3> pts, Func<Vec3, Vec3> dir = null)
        {
            var f = p.NewFeature(k);
            foreach (var q in pts) f.Points.Add(new MeasuredPoint(q, dir != null ? dir(q) : -Vec3.UnitZ, 0));
            FeatureSolver.Solve(p, f);
            Check(f.Geometry != null, "{0} solved ({1})", f.Name, f.FitError);
            return f;
        }

        static IEnumerable<Vec3> Cylinder(Vec3 basePt, Vec3 axis, double r, double h)
        {
            axis = axis.Normalized();
            Vec3 u = axis.AnyPerpendicular(), v = axis.Cross(u);
            for (int ring = 0; ring < 3; ring++)
                for (int i = 0; i < 12; i++)
                {
                    double t = i * Math.PI / 6;
                    yield return basePt + axis * (h * ring / 2) + (u * Math.Cos(t) + v * Math.Sin(t)) * r;
                }
        }

        static double Value(Project p, DimKind k, params Feature[] f)
        {
            var d = new Dimension { Kind = k };
            foreach (var x in f) d.Refs.Add(new EntityRef { FeatureId = x.Id });
            return Value(p, d);
        }

        static double Value(Project p, Dimension d)
        {
            var r = DimensionSolver.Evaluate(p, d);
            Check(r.Ok, "{0}: {1}", d.Kind, r.Error);
            return r.Value;
        }

        static void TestGdt()
        {
            var p = new Project();
            // flatness: 5×5 grid with one bump of 0.05 → range of residuals 0.05
            var grid = new List<Vec3>();
            for (int i = 0; i < 5; i++) for (int j = 0; j < 5; j++) grid.Add(new Vec3(i * 10, j * 10, i == 2 && j == 2 ? 0.05 : 0));
            var datum = Pts(p, FeatureKind.Plane, grid);
            double flat = Value(p, DimKind.Form, datum);
            Check(Math.Abs(flat - 0.05) < 1e-6, "flatness {0}", flat);
            // roundness: radial bumps of +0.03 / −0.02
            var circ = Pts(p, FeatureKind.Circle, Enumerable.Range(0, 24).Select(i =>
            {
                double t = i * Math.PI / 12, rr = 10 + (i == 3 ? 0.03 : i == 15 ? -0.02 : 0);
                return new Vec3(rr * Math.Cos(t), rr * Math.Sin(t), 5);
            }));
            double round = Value(p, DimKind.Form, circ);
            // the least-squares centre moves toward the bump, so the residual range is a little under 0.05
            Check(round > 0.035 && round < 0.05 + 1e-9, "roundness {0}", round);
            // straightness: one point 0.02 off a straight line → cylinder zone ≈ 2 × max distance
            var line = Pts(p, FeatureKind.Line, Enumerable.Range(0, 11).Select(i => new Vec3(i * 5, i == 5 ? 0.02 : 0, 0)));
            double straight = Value(p, DimKind.Form, line);
            Check(straight > 0.03 && straight < 0.041, "straightness {0}", straight);

            // parallelism: top tilted 0.04 over 40 mm
            var top = Pts(p, FeatureKind.Plane, new[] { new Vec3(0, 0, 10), new Vec3(40, 0, 10.04), new Vec3(40, 30, 10.04), new Vec3(0, 30, 10), new Vec3(20, 15, 10.02) });
            double par = Value(p, DimKind.Parallelism, top, datum);
            Check(Math.Abs(par - 0.04) < 1e-4, "parallelism {0}", par);
            // perpendicularity of a wall leaning 0.04 over 20 mm
            var wall = Pts(p, FeatureKind.Plane, new[] { 0.0, 30 }.SelectMany(y => new[] { 0.0, 10, 20 }.Select(z => new Vec3(0.002 * z, y, z))), q => Vec3.UnitX);
            double perp = Value(p, DimKind.Perpendicularity, wall, datum);
            Check(Math.Abs(perp - 0.04) < 1e-4, "perpendicularity {0}", perp);
            // angularity at the right nominal is the same as perpendicularity; a wrong nominal is large
            var ang = new Dimension { Kind = DimKind.Angularity, Nominal = 90, Refs = { new EntityRef { FeatureId = wall.Id }, new EntityRef { FeatureId = datum.Id } } };
            Check(Math.Abs(Value(p, ang) - perp) < 1e-9, "angularity 90° = perpendicularity");
            ang.Nominal = 80;
            Check(Value(p, ang) > 3, "angularity at a wrong nominal angle is large");
            // axis perpendicularity: a pin tilted 0.001 rad over 20 mm → Ø0.02
            var pin = Pts(p, FeatureKind.Cylinder, Cylinder(new Vec3(60, 60, 0), new Vec3(0.001, 0, 1), 5, 20), q => (new Vec3(60, 60, q.Z) - q).Normalized());
            double pinPerp = Value(p, DimKind.Perpendicularity, pin, datum);
            Check(Math.Abs(pinPerp - 0.02) < 0.002, "axis perpendicularity {0}", pinPerp);

            // position: a point 0.1 off its nominal → Ø0.2; a hole is judged across its axis only
            var pt = Pts(p, FeatureKind.Point, new[] { new Vec3(10.1, 20, 0) });
            var pos = new Dimension { Kind = DimKind.Position, NX = 10, NY = 20, NZ = 0, Refs = { new EntityRef { FeatureId = pt.Id } } };
            Check(Math.Abs(Value(p, pos) - 0.2) < 1e-9, "point position");
            var hole = Pts(p, FeatureKind.Cylinder, Cylinder(new Vec3(30.1, 40, 0), Vec3.UnitZ, 4, 15), q => (q - new Vec3(30.1, 40, q.Z)).Normalized());
            var hp = new Dimension { Kind = DimKind.Position, NX = 30, NY = 40, NZ = 123, Refs = { new EntityRef { FeatureId = hole.Id } } };
            double hv = Value(p, hp);
            Check(Math.Abs(hv - 0.2) < 1e-3, "hole position across its axis {0}", hv);

            // coaxiality: two parallel axes 0.05 apart → Ø0.1
            var shaft = Pts(p, FeatureKind.Cylinder, Cylinder(new Vec3(100, 0, 0), Vec3.UnitZ, 8, 20), q => (new Vec3(100, 0, q.Z) - q).Normalized());
            var step = Pts(p, FeatureKind.Cylinder, Cylinder(new Vec3(100.05, 0, 25), Vec3.UnitZ, 5, 10), q => (new Vec3(100.05, 0, q.Z) - q).Normalized());
            double coax = Value(p, DimKind.Coaxiality, step, shaft);
            Check(Math.Abs(coax - 0.1) < 2e-3, "coaxiality {0}", coax);

            // tolerance zone check and report line
            var fd = new Dimension { Kind = DimKind.Form, Name = "Flat", Toleranced = true, TolPlus = 0.1, Refs = { new EntityRef { FeatureId = datum.Id } } };
            Check(Tolerance.Evaluate(fd, DimensionSolver.Evaluate(p, fd), Mat4.Identity()).Status == TolStatus.Pass, "flatness 0.05 within 0.1");
            fd.TolPlus = 0.04;
            Check(Tolerance.Evaluate(fd, DimensionSolver.Evaluate(p, fd), Mat4.Identity()).Status == TolStatus.Fail, "flatness 0.05 exceeds 0.04");
            string line2 = CadExport.ToleranceLine(p, fd, DimensionSolver.Evaluate(p, fd), new UnitFmt(false));
            Check(line2 == "Tolerance 0.040   Actual 0.050  FAIL", "GD&T tolerance line '{0}'", line2);
            Check(Gdt.Name(DimKind.Form, datum.Geometry) == "Flatness" && Gdt.Name(DimKind.Form, pin.Geometry) == "Cylindricity", "form names");

            // minimum zone: the line through the bump can move halfway toward it → zone Ø0.02 instead of ≈0.036
            var mzd = new Dimension { Kind = DimKind.Form, MinZone = true, Refs = { new EntityRef { FeatureId = line.Id } } };
            double mzs = Value(p, mzd);
            Check(Math.Abs(mzs - 0.02) < 1e-3 && mzs < straight, "minimum-zone straightness {0} (least squares {1})", mzs, straight);
            var rng = new Random(5);
            var noisy = Pts(p, FeatureKind.Plane, Enumerable.Range(0, 60).Select(i => new Vec3(rng.NextDouble() * 80, rng.NextDouble() * 60, 0.001 * i % 0.017)));
            var mzf = new Dimension { Kind = DimKind.Form, MinZone = true, Refs = { new EntityRef { FeatureId = noisy.Id } } };
            double lsqf = Value(p, DimKind.Form, noisy), mzv = Value(p, mzf);
            Check(mzv <= lsqf + 1e-12 && mzv > 0.5 * lsqf, "minimum-zone flatness {0} vs least squares {1}", mzv, lsqf);
            mzd.Refs[0].FeatureId = pin.Id;
            Check(Value(p, mzd) < 1e-3, "minimum-zone cylindricity of a perfect pin");
        }

        static void Check(bool c, string m, params object[] a) { Program.Check(c, m, a); }

        static Feature Plane(Project p, double z)
        {
            var f = p.NewFeature(FeatureKind.Plane);
            foreach (var xy in new[] { new[] { 0.0, 0 }, new[] { 40.0, 0 }, new[] { 40.0, 30 }, new[] { 0.0, 30 }, new[] { 20.0, 15 } })
                f.Points.Add(new MeasuredPoint(new Vec3(xy[0], xy[1], z), -Vec3.UnitZ, 0));
            return f;
        }

        static void TestTolerances()
        {
            var p = new Project();
            var a = Plane(p, 0); var b = Plane(p, 10);
            var pt = p.NewFeature(FeatureKind.Point);
            pt.Points.Add(new MeasuredPoint(new Vec3(1, 2, 3), -Vec3.UnitZ, 0));
            FeatureSolver.SolveAll(p);
            var dist = new Dimension { Id = 1, Kind = DimKind.Distance, Name = "Height", Refs = { new EntityRef { FeatureId = a.Id }, new EntityRef { FeatureId = b.Id } } };
            var pos = new Dimension { Id = 2, Kind = DimKind.Coordinate, Name = "Hole", Refs = { new EntityRef { FeatureId = pt.Id } } };
            p.Dimensions.Add(dist); p.Dimensions.Add(pos);
            var m = Mat4.Identity();

            Check(Tolerance.Evaluate(dist, DimensionSolver.Evaluate(p, dist), m).Status == TolStatus.None, "no tolerance → None");
            dist.Toleranced = true; dist.Nominal = 10.05; dist.TolPlus = 0.1; dist.TolMinus = -0.1;
            var t = Tolerance.Evaluate(dist, DimensionSolver.Evaluate(p, dist), m);
            Check(t.Status == TolStatus.Pass && Math.Abs(t.Deviation + 0.05) < 1e-9, "10 vs 10.05±0.1 passes (dev {0})", t.Deviation);
            dist.Nominal = 10.2;
            Check(Tolerance.Evaluate(dist, DimensionSolver.Evaluate(p, dist), m).Status == TolStatus.Fail, "10 vs 10.2±0.1 fails");
            dist.Nominal = 9.9; dist.TolPlus = 0.1; dist.TolMinus = 0;
            Check(Tolerance.Evaluate(dist, DimensionSolver.Evaluate(p, dist), m).Status == TolStatus.Pass, "upper edge of a unilateral band passes");

            pos.Toleranced = true; pos.NX = 1; pos.NY = 2; pos.NZ = 3.2; pos.TolPlus = 0.1; pos.TolMinus = -0.1;
            t = Tolerance.Evaluate(pos, DimensionSolver.Evaluate(p, pos), m);
            Check(t.Status == TolStatus.Fail && Math.Abs(t.AxisDeviation.Z + 0.2) < 1e-9 && Math.Abs(t.Deviation + 0.2) < 1e-9, "position fails in Z");
            // the nominal is in the part frame: shift the part frame by -0.2 in Z and it passes
            p.AlignmentMatrix = Mat4.Translation(0, 0, 0.2);
            Check(Tolerance.Evaluate(pos, DimensionSolver.Evaluate(p, pos), p.AlignmentMatrix).Status == TolStatus.Pass, "position uses the part frame");

            string rep = CadExport.Report(p, true, new UnitFmt(false), "Arm check: test");
            Check(rep.Contains("Tolerance check: 2 of 2 in tolerance"), "report summary:\n{0}", rep);
            Check(rep.Contains("Nominal 9.900  +0.100/−0.000   Dev +0.100  PASS"), "report tolerance line:\n{0}", rep);
            Check(rep.Contains("Nominal X 1.000 Y 2.000 Z 3.200  ±0.100"), "report position line:\n{0}", rep);
            Check(rep.Contains("Arm check: test"), "report carries the capability line");
            dist.Nominal = 12;
            rep = CadExport.Report(p, true, new UnitFmt(false));
            Check(rep.Contains("OUT OF TOLERANCE: Height"), "report lists the failures:\n{0}", rep);

            string path = Path.Combine(Path.GetTempPath(), "msinspector_tol.msproj");
            ProjectIO.Save(p, path);
            var q = ProjectIO.Load(path);
            var d2 = q.Dimensions.First(x => x.Name == "Height");
            Check(d2.Toleranced && d2.Nominal == 12 && d2.TolPlus == 0.1 && d2.TolMinus == 0, "tolerance survives the project file");
            Check(q.Dimensions.First(x => x.Name == "Hole").NZ == 3.2, "nominal position survives the project file");
        }
    }
}
