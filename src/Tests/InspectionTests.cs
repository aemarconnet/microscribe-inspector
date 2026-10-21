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
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("nominal model: STL, distances, pairs, best fit, profile", TestModel));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("HTML report and results log", TestReport));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("inspection plan runs and SPC", TestPlanSpc));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("constructions: offset, mid-plane, projection, mirror, bolt circle", TestConstructions));
            Program.ExtraTests.Add(new KeyValuePair<string, Action>("constrained fits: fixed size, parallel, perpendicular", TestConstrained));
        }

        static void TestConstrained()
        {
            var p = new Project();
            var datum = Plane(p, 0);
            FeatureSolver.Solve(p, datum);
            // a wall leaning 0.04 over 20 mm, forced square to the base
            var wall = Pts(p, FeatureKind.Plane, new[] { 0.0, 30 }.SelectMany(y => new[] { 0.0, 10, 20 }.Select(z => new Vec3(0.002 * z, y, z))), q => Vec3.UnitX);
            Check(Math.Abs(((PlanePrim)wall.Geometry).Normal.Z) > 1e-3, "free wall leans");
            wall.DirectionRelation = "perpendicular"; wall.DirectionDatumId = datum.Id;
            FeatureSolver.Solve(p, wall);
            var wn = ((PlanePrim)wall.Geometry).Normal;
            Check(Math.Abs(wn.Z) < 1e-12 && Math.Abs(wn.X) > 0.999, "wall forced perpendicular ({0})", wn);
            Check(wall.Geometry.Stats.Form > 0.03, "the lean shows up as form ({0})", wall.Geometry.Stats.Form);
            // a top face forced parallel to the base
            var top = Pts(p, FeatureKind.Plane, new[] { new Vec3(0, 0, 10), new Vec3(40, 0, 10.04), new Vec3(40, 30, 10.04), new Vec3(0, 30, 10), new Vec3(20, 15, 10.02) });
            top.DirectionRelation = "parallel"; top.DirectionDatumId = datum.Id;
            FeatureSolver.Solve(p, top);
            Check((((PlanePrim)top.Geometry).Normal - Vec3.UnitZ).Length < 1e-12, "top forced parallel");
            // a pin tilted 0.001 rad, forced perpendicular to the base: axis exactly along Z
            var pin = Pts(p, FeatureKind.Cylinder, Cylinder(new Vec3(60, 60, 0), new Vec3(0.001, 0, 1), 5, 20), q => (new Vec3(60, 60, q.Z) - q).Normalized());
            pin.DirectionRelation = "perpendicular"; pin.DirectionDatumId = datum.Id;
            FeatureSolver.Solve(p, pin);
            var pc = (CylinderPrim)pin.Geometry;
            Check(Math.Abs(Math.Abs(pc.Axis.Z) - 1) < 1e-12 && Math.Abs(pc.Radius - 5) < 0.01, "pin axis {0}, R {1}", pc.Axis, pc.Radius);
            // fixed diameter
            pin.DirectionRelation = ""; pin.FixSize = true; pin.FixedSize = 10.1;
            FeatureSolver.Solve(p, pin);
            Check(Math.Abs(((CylinderPrim)pin.Geometry).Radius - 5.05) < 1e-12 && Math.Abs(pin.Geometry.Stats.Rms - 0.05) < 0.005, "fixed diameter, rms {0}", pin.Geometry.Stats.Rms);
            // a cross hole along X, tilted 0.01 rad toward Z, forced perpendicular to a vertical datum axis
            var shaft = Pts(p, FeatureKind.Cylinder, Cylinder(new Vec3(0, 0, 0), Vec3.UnitZ, 8, 30), q => (new Vec3(0, 0, q.Z) - q).Normalized());
            Vec3 xa = new Vec3(1, 0, 0.01).Normalized();
            var cross = Pts(p, FeatureKind.Cylinder, Cylinder(new Vec3(-15, 0, 15), xa, 3, 30), q => { Vec3 w = q - new Vec3(-15, 0, 15); return -(w - xa * w.Dot(xa)).Normalized(); });
            cross.DirectionRelation = "perpendicular"; cross.DirectionDatumId = shaft.Id;
            FeatureSolver.Solve(p, cross);
            var cc = (CylinderPrim)cross.Geometry;
            Check(Math.Abs(cc.Axis.Z) < 1e-9 && Math.Abs(Math.Abs(cc.Axis.X) - 1) < 1e-4, "cross hole axis {0}", cc.Axis);
            // a 90° countersink measured as 89°: fixed angle gives 45° exactly
            var cs = p.NewFeature(FeatureKind.Cone);
            double half = 44.5 * Math.PI / 180;
            for (int ring = 0; ring < 4; ring++) for (int i = 0; i < 10; i++)
            {
                double depth = 1 + ring * 1.5, rr = 10 - depth * Math.Tan(half), t = i * Math.PI / 5 + ring * 0.3;
                Vec3 q = new Vec3(rr * Math.Cos(t), rr * Math.Sin(t), -depth);
                cs.Points.Add(new MeasuredPoint(q, (new Vec3(0, 0, q.Z) - q).Normalized(), 0));
            }
            FeatureSolver.Solve(p, cs);
            cs.FixSize = true; cs.FixedSize = 90;
            FeatureSolver.Solve(p, cs);
            Check(Math.Abs(Math.Abs(((ConePrim)cs.Geometry).HalfAngle) - Math.PI / 4) < 1e-12, "fixed cone angle");
            // a Ø20 bore probed with a Ø3 ball, fixed at Ø20: the compensated bore is exactly Ø20
            var bore = p.NewFeature(FeatureKind.Cylinder);
            foreach (var q in Cylinder(new Vec3(100, 0, 0), Vec3.UnitZ, 8.5, 15))
            {
                Vec3 radial = (q - new Vec3(100, 0, q.Z)).Normalized();
                bore.Points.Add(new MeasuredPoint(q + radial * 0.01, (radial - Vec3.UnitZ).Normalized(), 1.5));
            }
            bore.FixSize = true; bore.FixedSize = 20;
            FeatureSolver.Solve(p, bore);
            Check(Math.Abs(((CylinderPrim)bore.Geometry).Radius - 10) < 1e-12, "fixed bore with ball compensation: R {0}", ((CylinderPrim)bore.Geometry).Radius);
        }

        static Feature Cons(Project p, string op, params Feature[] src)
        {
            var f = p.NewFeature(FeatureKind.Point);
            f.Origin = FeatureOrigin.Constructed; f.Construction = op;
            f.SourceIds.AddRange(src.Select(s => s.Id));
            return f;
        }

        static void TestConstructions()
        {
            var p = new Project();
            var bottom = Plane(p, 0); var top = Plane(p, 10.2);
            var pt = Pts(p, FeatureKind.Point, new[] { new Vec3(3, 4, 7) });
            var ln = Pts(p, FeatureKind.Line, Enumerable.Range(0, 5).Select(i => new Vec3(i * 10, 0, 7)));
            var ball = Pts(p, FeatureKind.Sphere, Enumerable.Range(0, 12).Select(i =>
            {
                double t = i * 0.9, ph = 0.3 + 0.1 * i;
                return new Vec3(5, 0, 3) + new Vec3(Math.Cos(t) * Math.Sin(ph), Math.Sin(t) * Math.Sin(ph), Math.Cos(ph)) * 8;
            }), q => (new Vec3(5, 0, 3) - q).Normalized());
            var holes = Enumerable.Range(0, 4).Select(i => Pts(p, FeatureKind.Point, new[] { new Vec3(10 + 25 * Math.Cos(i * Math.PI / 2 + 0.3), 10 + 25 * Math.Sin(i * Math.PI / 2 + 0.3), 0) })).ToArray();
            var yz = Pts(p, FeatureKind.Plane, new[] { new Vec3(0, 0, 0), new Vec3(0, 10, 0), new Vec3(0, 10, 10), new Vec3(0, 0, 10), new Vec3(0, 5, 5) }, q => Vec3.UnitX);

            var mid = Cons(p, "midplane", bottom, top);
            var off = Cons(p, "offset", bottom); off.ConstructValue = 5;
            var proj = Cons(p, "project", pt, bottom);
            var par = Cons(p, "parallel", pt, bottom);
            var perp = Cons(p, "perpendicular", ln, bottom);
            var mir = Cons(p, "mirror", ball, yz);
            var bolt = Cons(p, "boltcircle", holes);
            FeatureSolver.SolveAll(p);

            var mp = (PlanePrim)mid.Geometry;
            Check(Math.Abs(Math.Abs(mp.Normal.Z) - 1) < 1e-9 && Math.Abs(mp.Origin.Z - 5.1) < 1e-9, "mid-plane at z = {0}", mp.Origin.Z);
            Check(CadExport.Describe(mid.Geometry, mid, new UnitFmt(false)).First() == "Width     10.200 mm", "mid-plane width line");
            Check(mid.Kind == FeatureKind.Plane && PlaneBounds.Corners(mid, mp).Max(c => c.X) > 39, "constructed plane is sized by its sources");
            Check(Math.Abs(((PlanePrim)off.Geometry).SignedDistance(new Vec3(0, 0, 5))) < 1e-9, "offset plane");
            Check((((PointPrim)proj.Geometry).Position - new Vec3(3, 4, 0)).Length < 1e-9, "projection");
            Check(Math.Abs(((PlanePrim)par.Geometry).SignedDistance(new Vec3(100, -50, 7))) < 1e-9, "parallel plane through the point");
            var pp = (PlanePrim)perp.Geometry;
            Check(Math.Abs(Math.Abs(pp.Normal.Y) - 1) < 1e-9 && Math.Abs(pp.SignedDistance(new Vec3(33, 0, 7))) < 1e-9, "perpendicular plane through the line");
            var ms = (SpherePrim)mir.Geometry;
            Check((ms.Center - new Vec3(-5, 0, 3)).Length < 1e-6 && Math.Abs(ms.Radius - 8) < 1e-6 && mir.Kind == FeatureKind.Sphere, "mirrored sphere at {0}", ms.Center);
            var bc = (CirclePrim)bolt.Geometry;
            Check(Math.Abs(bc.Radius - 25) < 1e-6 && (bc.Center - new Vec3(10, 10, 0)).Length < 1e-6, "bolt circle R{0}", bc.Radius);
            Check(CadExport.Describe(bolt.Geometry, bolt, new UnitFmt(false)).Last().StartsWith("4 centres, every 90°"), "bolt circle info");
        }

        static void TestPlanSpc()
        {
            var plan = new Project { Title = "Plan", PartNumber = "PN-7" };
            var a = Plane(plan, 0); var b = Plane(plan, 10);
            a.Notes = "Top face, spread the points";
            var cons = plan.NewFeature(FeatureKind.Line);
            cons.Origin = FeatureOrigin.Constructed; cons.Construction = "intersect"; cons.SourceIds.AddRange(new[] { a.Id, b.Id });
            plan.Dimensions.Add(new Dimension { Id = 1, Kind = DimKind.Distance, Name = "Height", Toleranced = true, Nominal = 10, TolPlus = 0.1, TolMinus = -0.1,
                Refs = { new EntityRef { FeatureId = a.Id }, new EntityRef { FeatureId = b.Id } } });
            plan.AlignmentRecipe.Orientation = OrientationMode.Features; plan.AlignmentRecipe.PrimaryId = a.Id;
            FeatureSolver.SolveAll(plan);
            var steps = InspectionPlan.Steps(plan);
            Check(steps.Count == 2 && steps[0].Id == a.Id && InspectionPlan.PlannedPoints(steps[0]) == 5, "steps and planned points");
            var run = InspectionPlan.NewRun(plan, "S009");
            Check(run.Features.Count == 3 && run.Features.All(f => f.Points.Count == 0) && run.Find(a.Id).Notes == a.Notes, "run copies the features without points");
            Check(run.Dimensions.Count == 1 && run.Dimensions[0].Toleranced && run.PartSerial == "S009" && run.PartNumber == "PN-7" && run.AlignmentRecipe.PrimaryId == a.Id, "run keeps measurements, alignment and part number");
            Check(plan.Features.All(f => f.Kind == FeatureKind.Line || f.Points.Count == 5), "the plan keeps its points");
            // measure the run: the same dimension evaluates on the new points
            foreach (var src in steps) foreach (var q in src.Points) run.Find(src.Id).Points.Add(new MeasuredPoint(q.Position + new Vec3(0, 0, src.Id == b.Id ? 0.05 : 0), q.Direction, 0));
            FeatureSolver.SolveAll(run);
            var t = Tolerance.Evaluate(run.Dimensions[0], DimensionSolver.Evaluate(run, run.Dimensions[0]), run.AlignmentMatrix);
            Check(t.Status == TolStatus.Pass && Math.Abs(t.Deviation - 0.05) < 1e-9, "run evaluates its own points ({0})", t.Deviation);
            Check(InspectionPlan.NextSerial("S009") == "S010" && InspectionPlan.NextSerial("12") == "13" && InspectionPlan.NextSerial("A") == "A-2" && InspectionPlan.NextSerial("") == "1", "serial numbers");

            var st = Spc.Compute(new[] { 10.0, 10.1, 9.9, 10.05, 9.95 }, 9.8, 10.2);
            double sigma = Math.Sqrt(0.025 / 4);
            Check(st.N == 5 && Math.Abs(st.Mean - 10) < 1e-9 && Math.Abs(st.Sigma - sigma) < 1e-9, "mean and sigma");
            Check(Math.Abs(st.Cp - 0.4 / (6 * sigma)) < 1e-9 && Math.Abs(st.Cpk - st.Cp) < 1e-9 && st.OutOfTolerance == 0, "Cp {0} Cpk {1}", st.Cp, st.Cpk);
            var z = Spc.Compute(new[] { 0.02, 0.03, 0.04 }, double.NaN, 0.1);
            Check(double.IsNaN(z.Cp) && Math.Abs(z.Cpk - (0.1 - 0.03) / (3 * 0.01)) < 1e-9, "one-sided Cpk for a zone tolerance");

            string log = Path.Combine(Path.GetTempPath(), "msinspector_spc.csv");
            if (File.Exists(log)) File.Delete(log);
            foreach (var serial in new[] { "S1", "S2", "S3" }) Results.AppendCsv(log, run, new ReportInfo { PartNumber = "PN-7", Serial = serial }, new UnitFmt(true));
            var entries = ResultsLog.Read(log);
            Check(entries.Count == 3 && entries.All(e => e.Key == "Height · " + entries[0].Characteristic), "log entries {0}", entries.Count);
            Check(Math.Abs(entries[0].Actual - 10.05) < 1e-3 && Math.Abs(entries[0].Lsl - 9.9) < 1e-3 && Math.Abs(entries[0].Usl - 10.1) < 1e-3 && entries[2].Serial == "S3", "log values back in mm");
        }

        static void TestReport()
        {
            var p = new Project { Title = "Bracket <rev A>" };
            var a = Plane(p, 0); var b = Plane(p, 10);
            var pt = p.NewFeature(FeatureKind.Point);
            pt.Points.Add(new MeasuredPoint(new Vec3(1, 2, 3), -Vec3.UnitZ, 0));
            FeatureSolver.SolveAll(p);
            p.Dimensions.Add(new Dimension { Id = 1, Kind = DimKind.Distance, Name = "Height", Toleranced = true, Nominal = 10, TolPlus = 0.1, TolMinus = -0.1,
                Refs = { new EntityRef { FeatureId = a.Id }, new EntityRef { FeatureId = b.Id } } });
            p.Dimensions.Add(new Dimension { Id = 2, Kind = DimKind.Coordinate, Name = "Hole", Toleranced = true, NX = 1, NY = 2, NZ = 3.5, TolPlus = 0.1, TolMinus = -0.1,
                Refs = { new EntityRef { FeatureId = pt.Id } } });
            p.Dimensions.Add(new Dimension { Id = 3, Kind = DimKind.Form, Name = "Flat", Refs = { new EntityRef { FeatureId = a.Id } } });
            var rows = Results.Rows(p);
            Check(rows.Count == 5, "rows {0}", rows.Count);
            Check(rows[0].Status == TolStatus.Pass && rows.Count(r => r.Status == TolStatus.Fail) == 1 && rows[3].Characteristic.StartsWith("Z") && rows[3].Status == TolStatus.Fail, "row statuses");
            Check(rows[4].IsZone && rows[4].Status == TolStatus.None, "untoleranced form row");
            var info = new ReportInfo { PartNumber = "PN-42", Serial = "S001", Operator = "QA" };
            string html = HtmlReport.Build(p, new UnitFmt(false), info, new[] { new KeyValuePair<string, byte[]>("view", new byte[] { 137, 80, 78, 71 }) });
            Check(html.Contains("FAIL — 1 of 4 toleranced"), "banner");
            Check(html.Contains("Bracket &lt;rev A&gt;") && !html.Contains("<rev A>"), "title is escaped");
            Check(html.Contains("data:image/png;base64,iVBORw") && html.Contains("PN-42") && html.Contains("class=\"st PASS\">PASS"), "image, header and status cells");
            string log = Path.Combine(Path.GetTempPath(), "msinspector_log.csv");
            if (File.Exists(log)) File.Delete(log);
            Results.AppendCsv(log, p, info, new UnitFmt(false));
            info.Serial = "S002";
            Results.AppendCsv(log, p, info, new UnitFmt(false));
            var lines = File.ReadAllLines(log);
            Check(lines.Length == 11 && lines[0].StartsWith("date,project,part_number") && lines.Count(l => l.Contains("\"S002\"")) == 5, "results log has one header and 10 rows ({0})", lines.Length);
            Check(lines[1].Contains(",10.000,0.100,-0.100,10.000,0.000,PASS"), "log row: {0}", lines[1]);
        }

        /// <summary>Box 0..L × 0..W × 0..H as outward-facing triangles.</summary>
        static List<Vec3> Box(double L, double W, double H)
        {
            Vec3[] v = { new Vec3(0, 0, 0), new Vec3(L, 0, 0), new Vec3(L, W, 0), new Vec3(0, W, 0), new Vec3(0, 0, H), new Vec3(L, 0, H), new Vec3(L, W, H), new Vec3(0, W, H) };
            int[][] quads = { new[] { 0, 3, 2, 1 }, new[] { 4, 5, 6, 7 }, new[] { 0, 1, 5, 4 }, new[] { 3, 7, 6, 2 }, new[] { 0, 4, 7, 3 }, new[] { 1, 2, 6, 5 } };
            var c = new List<Vec3>();
            foreach (var q in quads) { c.Add(v[q[0]]); c.Add(v[q[1]]); c.Add(v[q[2]]); c.Add(v[q[0]]); c.Add(v[q[2]]); c.Add(v[q[3]]); }
            return c;
        }

        static void WriteBinaryStl(string path, List<Vec3> corners)
        {
            using (var w = new BinaryWriter(File.Create(path)))
            {
                w.Write(new byte[80]);
                w.Write((uint)(corners.Count / 3));
                for (int i = 0; i < corners.Count; i += 3)
                {
                    for (int k = 0; k < 3; k++) w.Write(0f);
                    for (int j = 0; j < 3; j++) { w.Write((float)corners[i + j].X); w.Write((float)corners[i + j].Y); w.Write((float)corners[i + j].Z); }
                    w.Write((ushort)0);
                }
            }
        }

        static Mat4 RotZ(double a, double tx, double ty, double tz)
        {
            var m = Mat4.Identity();
            m[0, 0] = Math.Cos(a); m[0, 1] = -Math.Sin(a); m[1, 0] = Math.Sin(a); m[1, 1] = Math.Cos(a);
            m[0, 3] = tx; m[1, 3] = ty; m[2, 3] = tz;
            return m;
        }

        static void TestModel()
        {
            var corners = Box(100, 60, 30);
            string bin = Path.Combine(Path.GetTempPath(), "msinspector_box.stl"), asc = Path.Combine(Path.GetTempPath(), "msinspector_box_ascii.stl");
            WriteBinaryStl(bin, corners);
            var sb = new System.Text.StringBuilder("solid box\n");
            for (int i = 0; i < corners.Count; i += 3)
            {
                sb.Append(" facet normal 0 0 0\n  outer loop\n");
                for (int j = 0; j < 3; j++) sb.AppendFormat(System.Globalization.CultureInfo.InvariantCulture, "   vertex {0} {1} {2}\n", corners[i + j].X / 25.4, corners[i + j].Y / 25.4, corners[i + j].Z / 25.4);
                sb.Append("  endloop\n endfacet\n");
            }
            sb.Append("endsolid box\n");
            File.WriteAllText(asc, sb.ToString());
            var mesh = TriMesh.LoadStl(bin);
            var inch = TriMesh.LoadStl(asc, 25.4);
            Check(mesh.TriangleCount == 12 && mesh.V.Length == 8, "binary STL: {0} triangles, {1} vertices", mesh.TriangleCount, mesh.V.Length);
            Check(inch.TriangleCount == 12 && (inch.Max - new Vec3(100, 60, 30)).Length < 1e-4, "ASCII STL in inches");
            Check(Math.Abs(mesh.SignedDistance(new Vec3(50, 30, 40)) - 10) < 1e-9, "10 above the top");
            Check(Math.Abs(mesh.SignedDistance(new Vec3(50, 30, 15)) + 15) < 1e-9, "15 inside");
            Check(Math.Abs(mesh.SignedDistance(new Vec3(-3, -4, 15)) - 5) < 1e-9, "5 off an edge");
            // inside-out exports are turned the right way round
            var flipped = new List<Vec3>();
            for (int i = 0; i < corners.Count; i += 3) { flipped.Add(corners[i]); flipped.Add(corners[i + 2]); flipped.Add(corners[i + 1]); }
            Check(TriMesh.FromCorners(flipped).SignedDistance(new Vec3(50, 30, 40)) > 0, "inverted normals are corrected");

            // the part sits on the table turned 30° and moved: machine = truth⁻¹ · model
            Mat4 truth = RotZ(30 * Math.PI / 180, -250, 80, 12);          // machine → model
            Mat4 toMachine = truth.RigidInverse();
            var modelPts = new[] { new Vec3(0, 0, 30), new Vec3(100, 0, 30), new Vec3(100, 60, 30), new Vec3(0, 60, 0) };
            double rms;
            var pairs = ModelFit.FromPairs(modelPts.Select(toMachine.TransformPoint).ToList(), modelPts, out rms);
            Check(rms < 1e-9 && (pairs.TransformPoint(toMachine.TransformPoint(new Vec3(5, 6, 7))) - new Vec3(5, 6, 7)).Length < 1e-9, "point pairs recover the transform (rms {0})", rms);

            // ball centres (r = 1.5) on five faces, then a best fit from a start that is 1° and 2 mm off
            var rng = new Random(11);
            var centres = new List<Vec3>(); var radii = new List<double>();
            Action<Func<double, double, Vec3>, Vec3, int> face = (at, n, k) =>
            {
                for (int i = 0; i < k; i++) { centres.Add(toMachine.TransformPoint(at(rng.NextDouble(), rng.NextDouble()) + n * 1.5)); radii.Add(1.5); }
            };
            face((u, v) => new Vec3(5 + 90 * u, 5 + 50 * v, 30), Vec3.UnitZ, 30);
            face((u, v) => new Vec3(5 + 90 * u, 0, 3 + 24 * v), -Vec3.UnitY, 15);
            face((u, v) => new Vec3(0, 5 + 50 * u, 3 + 24 * v), -Vec3.UnitX, 15);
            face((u, v) => new Vec3(100, 5 + 50 * u, 3 + 24 * v), Vec3.UnitX, 10);
            face((u, v) => new Vec3(5 + 90 * u, 60, 3 + 24 * v), Vec3.UnitY, 10);
            Mat4 start = RotZ(1 * Math.PI / 180, 2, -1.5, 0.8) * truth;
            var fit = ModelFit.BestFit(mesh, centres, radii, start);
            Check(fit != null && fit.Rms < 1e-6, "best fit rms {0}", fit != null ? fit.Rms : double.NaN);
            double err = new[] { new Vec3(0, 0, 0), new Vec3(100, 60, 30) }.Max(q => (fit.Transform.TransformPoint(toMachine.TransformPoint(q)) - q).Length);
            Check(err < 1e-4, "best fit recovers the part position ({0})", err);

            // profile of the top face with one point 0.05 high → zone 0.1
            var p = new Project { NominalModel = mesh };
            p.AlignmentMatrix = truth;
            var top = p.NewFeature(FeatureKind.Plane);
            for (int i = 0; i < 12; i++)
            {
                Vec3 s = new Vec3(10 + 7 * i, 10 + 3 * i, 30 + (i == 4 ? 0.05 : 0) + 1.5);
                top.Points.Add(new MeasuredPoint(toMachine.TransformPoint(s), toMachine.TransformVector(-Vec3.UnitZ), 1.5));
            }
            FeatureSolver.Solve(p, top);
            var prof = new Dimension { Kind = DimKind.Profile, Refs = { new EntityRef { FeatureId = top.Id } } };
            var pr = DimensionSolver.Evaluate(p, prof);
            Check(pr.Ok && Math.Abs(pr.Value - 0.1) < 1e-6, "profile {0} ({1})", pr.Value, pr.Error);
            p.NominalModel = null;
            Check(!DimensionSolver.Evaluate(p, prof).Ok, "profile needs a model");
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
