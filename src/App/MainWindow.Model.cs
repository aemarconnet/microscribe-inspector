using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using MicroScribe.Core;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.App
{
    /// <summary>Nominal CAD model (STL): loading, display, point-pair and best-fit alignment, deviations.</summary>
    public sealed partial class MainWindow
    {
        readonly List<Vec3> _modelPts = new List<Vec3>();   // picked on the model (model coordinates)
        readonly List<Vec3> _pairPts = new List<Vec3>();    // probed partners (machine frame, contact points)
        bool _pairProbing;
        string _modelLayerKey;

        static readonly Color[] DevColors =
        {
            Color.FromRgb(37, 99, 235), Color.FromRgb(56, 189, 248), Color.FromRgb(34, 197, 94), Color.FromRgb(251, 146, 60), Color.FromRgb(220, 38, 38)
        };

        /// <summary>Colour band of a deviation: below −2R, −2R..−R, within ±R, R..2R, above 2R.</summary>
        int DevBin(double d)
        {
            double r = Math.Max(1e-6, _s.DeviationRange);
            return d < -2 * r ? 0 : d < -r ? 1 : d <= r ? 2 : d <= 2 * r ? 3 : 4;
        }

        bool ModelAligned { get { return _proj.NominalModel != null; } }

        // ------------------------------------------------------------------ loading

        void LoadModelDialog()
        {
            var dlg = new Microsoft.Win32.OpenFileDialog { Filter = "STL model (*.stl)|*.stl|All files|*.*", Title = "Nominal model" };
            if (dlg.ShowDialog(this) != true) return;
            LoadModel(dlg.FileName, _proj.ModelScale <= 0 ? 1 : _proj.ModelScale, false);
        }

        bool LoadModel(string path, double scale, bool quiet)
        {
            try
            {
                Mouse.OverrideCursor = Cursors.Wait;
                var mesh = TriMesh.LoadStl(path, scale);
                _proj.NominalModel = mesh;
                _proj.ModelPath = path;
                _proj.ModelScale = scale;
                _dirty = true;
                Vec3 size = mesh.Max - mesh.Min;
                Status(string.Format("Model {0}: {1:N0} triangles, {2} × {3} × {4}", Path.GetFileName(path), mesh.TriangleCount, Ui.L(size.X), Ui.L(size.Y), Ui.LU(size.Z)));
                RebuildAll();
                return true;
            }
            catch (Exception ex)
            {
                if (!quiet) MessageBox.Show(this, "Could not read the model:\n" + ex.Message, "Nominal model", MessageBoxButton.OK, MessageBoxImage.Warning);
                else Status("Nominal model not loaded: " + ex.Message);
                return false;
            }
            finally { Mouse.OverrideCursor = null; }
        }

        /// <summary>After opening a project: reload its model (absolute path, or next to the project file).</summary>
        void LoadProjectModel()
        {
            if (string.IsNullOrEmpty(_proj.ModelPath)) return;
            string path = _proj.ModelPath;
            if (!File.Exists(path) && _projPath != null)
            {
                string near = Path.Combine(Path.GetDirectoryName(_projPath), Path.GetFileName(path));
                if (File.Exists(near)) path = near;
            }
            if (File.Exists(path)) LoadModel(path, _proj.ModelScale, true);
            else Status("The project's model " + Path.GetFileName(_proj.ModelPath) + " was not found — load it again in the Alignment tab");
        }

        // ------------------------------------------------------------------ display

        /// <summary>Model → display: the model is in part coordinates.</summary>
        Mat4 ModelToDisplay { get { return DisplayMatrix * _proj.AlignmentMatrix.RigidInverse(); } }

        void RebuildModelLayer()
        {
            var mesh = _proj.NominalModel;
            if (_view == null) return;
            if (mesh == null || !_s.ShowModel) { _view.SetLayer("model", null); _modelLayerKey = null; return; }
            Mat4 X = ModelToDisplay;
            string key = mesh.GetHashCode() + "|" + string.Join(",", X.M.Select(v => v.ToString("R"))) + "|" + (_tool == Tool.ModelPick);
            if (key == _modelLayerKey) return;
            _modelLayerKey = key;
            var pos = new Point3DCollection(mesh.T.Length);
            var nor = new Vector3DCollection(mesh.T.Length);
            var idx = new Int32Collection(mesh.T.Length);
            for (int t = 0; t < mesh.TriangleCount; t++)
            {
                Vec3 n = X.TransformVector(mesh.N[t]);
                for (int k = 0; k < 3; k++)
                {
                    Vec3 p = X.TransformPoint(mesh.V[mesh.T[3 * t + k]]);
                    pos.Add(new Point3D(p.X, p.Y, p.Z));
                    nor.Add(new Vector3D(n.X, n.Y, n.Z));
                    idx.Add(3 * t + k);
                }
            }
            var geo = new MeshGeometry3D { Positions = pos, Normals = nor, TriangleIndices = idx };
            geo.Freeze();
            var mat = Materials.Translucent(Color.FromRgb(170, 190, 215), _tool == Tool.ModelPick ? 0.55 : 0.3);
            var model = new GeometryModel3D(geo, mat) { BackMaterial = mat };
            var g = new Model3DGroup();
            g.Children.Add(model);
            g.Freeze();
            var tags = new Dictionary<Model3D, PickTag>();
            if (_tool == Tool.ModelPick) tags[model] = new PickTag { Model = true };
            _view.SetLayer("model", g, tags);
        }

        /// <summary>Deviation of a probed point from the model (ball radius removed), NaN without a model.</summary>
        double PointDeviation(Feature f, MeasuredPoint q)
        {
            if (_proj.NominalModel == null) return double.NaN;
            return ModelFit.Deviation(_proj.NominalModel, _proj.AlignmentMatrix, q.Position, f.Compensation == CompensationSide.None ? 0 : q.ProbeRadius);
        }

        // ------------------------------------------------------------------ alignment to the model

        void StartPointPairs()
        {
            if (_proj.NominalModel == null) { Status("Load a nominal model first"); return; }
            _modelPts.Clear(); _pairPts.Clear(); _pairProbing = false;
            _s.ShowModel = true;
            SetTool(Tool.ModelPick);
            RebuildAlignmentPanel();
        }

        /// <summary>A click on the model while picking pair points.</summary>
        void ModelPicked(Vec3 hitDisplay)
        {
            Vec3 m = ModelToDisplay.RigidInverse().TransformPoint(hitDisplay);
            _modelPts.Add(m);
            Sounds.Blip();
            UpdatePrompt();
            RebuildToolLayer();
        }

        /// <summary>Enter while picking: start probing the matching points on the part.</summary>
        bool ModelPickFinish()
        {
            if (_tool != Tool.ModelPick) return false;
            if (_modelPts.Count < 3) { Status("Pick at least 3 well-spread points on the model"); return true; }
            _pairProbing = true;
            _pairPts.Clear();
            SetTool(Tool.Select);
            Status("Now touch the same " + _modelPts.Count + " places on the part, in the same order (pedal 1 / Space)");
            UpdatePrompt();
            RebuildToolLayer();
            return true;
        }

        bool ModelPickUndo()
        {
            if (_tool == Tool.ModelPick && _modelPts.Count > 0) { _modelPts.RemoveAt(_modelPts.Count - 1); UpdatePrompt(); RebuildToolLayer(); return true; }
            if (_pairProbing && _pairPts.Count > 0) { _pairPts.RemoveAt(_pairPts.Count - 1); UpdatePrompt(); RebuildToolLayer(); return true; }
            return false;
        }

        void PairCapture(Vec3 tip, Vec3 dir)
        {
            // the ball touches the part where the stylus points
            _pairPts.Add(tip + dir.Normalized() * _s.CurrentTip.BallRadius);
            Sounds.Blip();
            if (_pairPts.Count >= _modelPts.Count) SolvePointPairs();
            UpdatePrompt();
            RebuildToolLayer();
        }

        void SolvePointPairs()
        {
            double rms;
            var m = ModelFit.FromPairs(_pairPts, _modelPts, out rms);
            var r = _proj.AlignmentRecipe;
            r.Orientation = OrientationMode.Model;
            r.ModelMatrix = (double[])m.M.Clone();
            r.ModelNote = string.Format("{0} point pairs, RMS {1}", _pairPts.Count, Ui.LU(rms));
            _pairProbing = false;
            Sounds.Success();
            Status("Aligned to the model from " + _pairPts.Count + " point pairs (RMS " + Ui.LU(rms) + "). Measure features, then use Best fit to refine.");
            _modelPts.Clear(); _pairPts.Clear();
            Changed();
        }

        void CancelPointPairs()
        {
            _pairProbing = false; _modelPts.Clear(); _pairPts.Clear();
            if (_tool == Tool.ModelPick) SetTool(Tool.Select);
            UpdatePrompt(); RebuildToolLayer(); RebuildAlignmentPanel();
        }

        string ModelPrompt()
        {
            if (_tool == Tool.ModelPick)
                return string.Format("Model points — click {0} on the model ({1} so far, 3+ well spread). Enter = done, Backspace = undo", _modelPts.Count < 3 ? "a point" : "more points or press Enter", _modelPts.Count);
            if (_pairProbing)
                return string.Format("Point pairs — touch the part at model point {0} of {1} and press pedal 1 / Space. Backspace = undo", _pairPts.Count + 1, _modelPts.Count);
            return null;
        }

        void BestFitToModel() { BestFitToModel(null); }

        /// <summary>Best fit over the given features (default: the selected ones, or all measured).</summary>
        void BestFitToModel(List<Feature> feats)
        {
            var mesh = _proj.NominalModel;
            if (mesh == null) { Status("Load a nominal model first"); return; }
            if (feats == null) feats = SelectedFeatures().Where(f => f.Origin == FeatureOrigin.Measured).ToList();
            if (feats.Count < 1) feats = _proj.Features.Where(f => f.Origin == FeatureOrigin.Measured).ToList();
            var c = new List<Vec3>(); var rad = new List<double>();
            foreach (var f in feats)
                foreach (var q in f.ActivePoints) { c.Add(q.Position); rad.Add(f.Compensation == CompensationSide.None ? 0 : q.ProbeRadius); }
            if (c.Count < 6) { Status("Best fit needs at least 6 probed points on the part"); return; }
            ModelFit.FitResult fit;
            try { Mouse.OverrideCursor = Cursors.Wait; fit = ModelFit.BestFit(mesh, c, rad, _proj.AlignmentMatrix); }
            finally { Mouse.OverrideCursor = null; }
            if (fit == null) { Sounds.Warn(); Status("Best fit failed — align roughly first (point pairs or 3-2-1)"); return; }
            var r = _proj.AlignmentRecipe;
            r.Orientation = OrientationMode.Model;
            r.ModelMatrix = (double[])fit.Transform.M.Clone();
            r.ModelNote = string.Format("best fit, RMS {0} over {1} of {2} points", Ui.LU(fit.Rms), fit.Used, fit.Total);
            if (fit.Used < fit.Total * 0.8) Sounds.Warn(); else Sounds.Success();
            Status(string.Format("Best fit to the model: RMS {0}, max {1}, {2} of {3} points used ({4} iterations)",
                Ui.LU(fit.Rms), Ui.LU(fit.MaxAbs), fit.Used, fit.Total, fit.Iterations));
            Changed();
        }

        // ------------------------------------------------------------------ UI

        UIElement ModelCard()
        {
            var c = new StackPanel();
            c.Children.Add(Ui.Header("NOMINAL MODEL (STL)"));
            var mesh = _proj.NominalModel;
            var units = new ComboBox { Width = 90 };
            units.Items.Add("mm"); units.Items.Add("inch");
            units.SelectedIndex = Math.Abs(_proj.ModelScale - 25.4) < 1e-9 ? 1 : 0;
            units.SelectionChanged += (s, e) =>
            {
                double sc = units.SelectedIndex == 1 ? 25.4 : 1;
                if (Math.Abs(sc - _proj.ModelScale) < 1e-12) return;
                _proj.ModelScale = sc;
                if (mesh != null && File.Exists(_proj.ModelPath)) LoadModel(_proj.ModelPath, sc, false);
            };
            var row = Ui.Row(Ui.Btn(mesh == null ? "Load STL…" : "Replace…", LoadModelDialog), Ui.Text("  file units ", 12, null, Ui.Muted), units);
            if (mesh != null) row.Children.Add(Ui.Btn("Remove", () => { _proj.NominalModel = null; _proj.ModelPath = ""; _dirty = true; RebuildAll(); }));
            c.Children.Add(row);
            if (mesh == null)
            {
                c.Children.Add(Ui.Text("Export the part from your CAD system as STL (in the part's own coordinates). Then align: pick 3+ points " +
                    "on the model and touch the same places on the part, and refine with a best fit over the measured points.", 11.5, null, Ui.Muted));
                return c;
            }
            Vec3 size = mesh.Max - mesh.Min;
            c.Children.Add(Ui.Text(string.Format("{0} · {1:N0} triangles · {2} × {3} × {4}", Path.GetFileName(_proj.ModelPath), mesh.TriangleCount,
                Ui.L(size.X), Ui.L(size.Y), Ui.LU(size.Z)), 11.5));
            var show = new CheckBox { Content = "Show the model", IsChecked = _s.ShowModel, Margin = new Thickness(0, 4, 0, 2) };
            show.Click += (s, e) => { _s.ShowModel = show.IsChecked == true; RebuildModelLayer(); };
            c.Children.Add(show);
            var acts = new WrapPanel();
            acts.Children.Add(Ui.Btn(_tool == Tool.ModelPick || _pairProbing ? "■ Stop point pairs" : "Point pairs…",
                () => { if (_tool == Tool.ModelPick || _pairProbing) CancelPointPairs(); else StartPointPairs(); },
                "Click 3+ points on the model, then touch the same places on the part"));
            acts.Children.Add(Ui.Btn("Best fit", BestFitToModel, "Refine the alignment over the measured points (selected features, or all)"));
            c.Children.Add(acts);
            if (_proj.AlignmentRecipe.Orientation == OrientationMode.Model)
                c.Children.Add(Ui.Text("Part frame = model: " + _proj.AlignmentRecipe.ModelNote, 11.5, FontWeights.SemiBold));
            var col = new CheckBox { Content = "Colour probed points by deviation from the model", IsChecked = _s.ColorByDeviation, Margin = new Thickness(0, 6, 0, 2) };
            col.Click += (s, e) => { _s.ColorByDeviation = col.IsChecked == true; RebuildScene(); RebuildAlignmentPanel(); };
            c.Children.Add(col);
            if (_s.ColorByDeviation)
            {
                c.Children.Add(Ui.Row(Short("Green within ±"), Ui.LenBox(_s.DeviationRange, v => { _s.DeviationRange = Math.Max(0.001, v); RebuildScene(); }), Ui.UnitLabel()));
                var legend = new WrapPanel { Margin = new Thickness(0, 2, 0, 2) };
                string[] names = { "< −2×", "−2×…−1×", "within", "+1×…+2×", "> +2×" };
                for (int i = 0; i < 5; i++)
                {
                    legend.Children.Add(new Border { Width = 12, Height = 12, Background = Ui.Frozen(DevColors[i]), Margin = new Thickness(0, 0, 3, 0), CornerRadius = new CornerRadius(6) });
                    legend.Children.Add(Ui.Text(names[i] + "   ", 11, null, Ui.Muted));
                }
                c.Children.Add(legend);
            }
            c.Children.Add(Ui.Text("Profile (Measure tab) gives the deviation zone of any feature's points; the readout shows the stylus' distance from the model.", 11, null, Ui.Muted));
            return c;
        }
    }
}
