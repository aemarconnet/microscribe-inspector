using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using MicroScribe.Core;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    /// <summary>2D sketches (lines and arcs) from traced profiles; DXF and SolidWorks curve export.</summary>
    public sealed partial class MainWindow
    {
        int _sketchShown;   // feature whose sketch is drawn in the view

        /// <summary>The curve's sketch on its chosen plane (own best-fit plane by default); null when not possible.</summary>
        Sketch2D SketchOf(Feature f)
        {
            var pts = FeatureSolver.ContactPoints(f, f.ActivePoints.ToList());
            if (pts.Count < 2) return null;
            PlanePrim plane = null;
            var pf = f.SketchPlaneId != 0 ? _proj.Find(f.SketchPlaneId) : null;
            if (pf != null) plane = pf.Geometry as PlanePrim;
            if (plane == null)
            {
                if (pts.Count < 3) return null;
                plane = Fitter.FitPlane(pts.ToArray());
            }
            return Sketch2D.Build(pts, plane, _proj.AlignmentMatrix, f.SketchTolerance, f.Closed);
        }

        UIElement SketchCard(Feature f)
        {
            var sp = new StackPanel();
            sp.Children.Add(Ui.Header("2D SKETCH (lines and arcs)"));
            var pb = new ComboBox { Width = 170 };
            pb.Items.Add("Curve's own best-fit plane");
            var planes = _proj.Features.Where(x => x.Kind == FeatureKind.Plane && x.Geometry is PlanePrim).ToList();
            foreach (var pl in planes) pb.Items.Add(pl.Name);
            var cur = planes.FirstOrDefault(x => x.Id == f.SketchPlaneId);
            pb.SelectedIndex = cur != null ? planes.IndexOf(cur) + 1 : 0;
            pb.SelectionChanged += (s, e) => { f.SketchPlaneId = pb.SelectedIndex > 0 ? planes[pb.SelectedIndex - 1].Id : 0; _dirty = true; RebuildFeaturePanel(); RebuildToolLayer(); };
            sp.Children.Add(Ui.Row(Lbl("Sketch plane"), pb));
            sp.Children.Add(Ui.Row(Lbl("Fit tolerance"), Ui.LenBox(f.SketchTolerance, v => { f.SketchTolerance = Math.Max(0.001, v); _dirty = true; RebuildFeaturePanel(); RebuildToolLayer(); }), Ui.UnitLabel()));
            var sk = SketchOf(f);
            if (sk == null || sk.Segments.Count == 0) { sp.Children.Add(Ui.Text("Trace the profile first.", 11.5, null, Ui.Muted)); return sp; }
            sp.Children.Add(Ui.Text(string.Format("{0} line{1}, {2} arc{3} · max deviation {4}", sk.Lines, sk.Lines == 1 ? "" : "s", sk.Arcs, sk.Arcs == 1 ? "" : "s", Ui.LU(sk.MaxDeviation)), 11.5, FontWeights.SemiBold));
            var show = new CheckBox { Content = "Show the sketch in the view", IsChecked = _sketchShown == f.Id, Margin = new Thickness(0, 4, 0, 2) };
            show.Click += (s, e) => { _sketchShown = show.IsChecked == true ? f.Id : 0; RebuildToolLayer(); };
            sp.Children.Add(show);
            var w = new WrapPanel();
            w.Children.Add(Ui.Btn("Export DXF…", () => ExportSketchDxf(f), "Lines and arcs in the sketch plane (axes follow the part frame)"));
            w.Children.Add(Ui.Btn("SolidWorks curve (.sldcrv)…", () => ExportSldcrv(f), "3D points of the curve for Insert > Curve > Curve Through XYZ Points"));
            sp.Children.Add(w);
            sp.Children.Add(Ui.Text("Corners become sharp line intersections; lower the tolerance for more, shorter pieces. The DXF is in the current units.", 11, null, Ui.Muted));
            return sp;
        }

        void ExportSketchDxf(Feature f)
        {
            var sk = SketchOf(f);
            if (sk == null || sk.Segments.Count == 0) { Status("Nothing to export"); return; }
            string path = ExportPath(".dxf", "DXF drawing (*.dxf)|*.dxf");
            if (path == null) return;
            File.WriteAllText(path, sk.ToDxf(Ui.Inch), new UTF8Encoding(false));
            Status(string.Format("Exported {0}: {1} lines, {2} arcs in {3}", Path.GetFileName(path), sk.Lines, sk.Arcs, Ui.Inch ? "inches" : "millimetres"));
        }

        /// <summary>
        /// The curve's points in the part frame, one "x y z" per line — for SolidWorks' Curve Through XYZ
        /// Points (values are read in the document's units, so the file uses the current display units).
        /// </summary>
        void ExportSldcrv(Feature f)
        {
            var c = f.Geometry as CurvePrim;
            if (c == null) { Status("Trace the curve first"); return; }
            string path = ExportPath(".sldcrv", "SolidWorks curve (*.sldcrv)|*.sldcrv|Text (*.txt)|*.txt");
            if (path == null) return;
            Mat4 m = _proj.AlignmentMatrix;
            var pts = (c.Degree <= 1 ? c.Ctrl : c.Samples).Select(m.TransformPoint).ToList();
            var sb = new StringBuilder();
            foreach (var p in pts) sb.Append(Ui.L(p.X)).Append(' ').Append(Ui.L(p.Y)).Append(' ').Append(Ui.L(p.Z)).Append("\r\n");
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
            Status(string.Format("Exported {0} points to {1} ({2}; open a part in the same units)", pts.Count, Path.GetFileName(path), Ui.Inch ? "inches" : "millimetres"));
        }

        /// <summary>Draws the shown sketch (tool layer).</summary>
        void AddSketchOverlay(Model3DGroup g, Mat4 M)
        {
            var f = _sketchShown != 0 ? _proj.Find(_sketchShown) : null;
            if (f == null || f.Kind != FeatureKind.Curve) return;
            var sk = SketchOf(f);
            if (sk == null || sk.Segments.Count == 0) return;
            var mb = new MeshBuilder();
            foreach (var s in sk.Segments)
            {
                var one = new Sketch2D { Origin = sk.Origin, U = sk.U, V = sk.V, Segments = new List<Seg2> { s } };
                mb.Polyline(one.Polyline3D().Select(M.TransformPoint).ToList(), _marker * 0.3);
                mb.Sphere(M.TransformPoint(sk.To3D(s.X0, s.Y0)), _marker * 0.9, 8, 6);
            }
            g.Children.Add(SceneView.Model(mb, Materials.Emissive(Color.FromRgb(251, 146, 60))));
        }
    }
}
