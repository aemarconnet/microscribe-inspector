using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using MicroScribe.Core;

namespace MicroScribe.App
{
    /// <summary>What a pickable model represents.</summary>
    public sealed class PickTag
    {
        public int FeatureId;
        /// <summary>-1 = the feature's geometry; -2 = its probed-point markers (nearest point is resolved on pick).</summary>
        public int PointIndex = -1;
        /// <summary>Bounding-box anchor handle: values 0/1/2 = min/centre/max per axis.</summary>
        public int[] Anchor;
        /// <summary>Draggable handle id (e.g. "plane-edge"), with an index.</summary>
        public string Handle;
        public int HandleIndex;
        public int DimId;
        public bool Draggable { get { return Handle != null; } }
    }

    public sealed class Label3D
    {
        public Vec3 Position;
        public string Text;
        public Brush Brush;
        public Brush Background;
        public bool Bold;
    }

    /// <summary>
    /// Interactive 3D view (Z-up, CAD style): left-drag orbit, right/middle-drag pan, wheel zoom,
    /// double-click sets the orbit centre, click picks, handles can be dragged, SpaceMouse navigation.
    /// </summary>
    public sealed class SceneView : Grid
    {
        readonly Viewport3D _vp = new Viewport3D();
        readonly PerspectiveCamera _cam = new PerspectiveCamera { FieldOfView = 40, NearPlaneDistance = 1, FarPlaneDistance = 1e6 };
        readonly DirectionalLight _head = new DirectionalLight(Color.FromRgb(200, 200, 200), new Vector3D(0, 0, -1));
        readonly Canvas _labels = new Canvas { IsHitTestVisible = false, ClipToBounds = true };
        readonly Dictionary<string, ModelVisual3D> _layers = new Dictionary<string, ModelVisual3D>();
        readonly Dictionary<string, Dictionary<Model3D, PickTag>> _layerTags = new Dictionary<string, Dictionary<Model3D, PickTag>>();
        readonly Dictionary<string, List<Label3D>> _layerLabels = new Dictionary<string, List<Label3D>>();
        // Opaque layers first, translucent layers last: WPF writes translucent surfaces into the depth
        // buffer, so anything drawn after them and behind them would disappear.
        static readonly string[] LayerOrder = { "grid", "features", "work", "dims", "overlay", "handles", "tool", "live", "surfaces", "workfill", "gridalpha" };

        Vec3 _target = new Vec3(0, 0, 0);
        double _yaw = -0.9, _pitch = 0.55, _dist = 900;
        Point _last, _down;
        bool _rotating, _panning, _moved;
        PickTag _dragTag;

        public event Action<PickTag, Vec3, Point> Picked;
        public event Action<Point> Hover;
        public event Action<Point> DoubleClicked;
        public event Action<PickTag> DragStarted;
        public event Action<PickTag, Vec3, Vec3> Dragging;   // ray origin, ray direction
        public event Action<PickTag> DragEnded;
        public event Action CameraChanged;

        public double SceneScale = 300; // characteristic size of the scene, mm
        /// <summary>When true, left clicks never start an orbit (used while sketching).</summary>
        public bool ClickMode;

        public SceneView()
        {
            Background = new LinearGradientBrush(Color.FromRgb(58, 66, 82), Color.FromRgb(18, 21, 28), 90);
            _vp.Camera = _cam;
            var lights = new Model3DGroup();
            lights.Children.Add(new AmbientLight(Color.FromRgb(70, 70, 76)));
            lights.Children.Add(_head);
            lights.Children.Add(new DirectionalLight(Color.FromRgb(80, 80, 90), new Vector3D(0.3, 0.5, -1)));
            _vp.Children.Add(new ModelVisual3D { Content = lights });
            foreach (var n in LayerOrder)
            {
                var v = new ModelVisual3D();
                _layers[n] = v;
                _vp.Children.Add(v);
                _layerTags[n] = new Dictionary<Model3D, PickTag>();
                _layerLabels[n] = new List<Label3D>();
            }
            Children.Add(_vp);
            Children.Add(_labels);
            ClipToBounds = true;
            Focusable = true;
            UpdateCamera();
            SizeChanged += (s, e) => UpdateLabels();
        }

        // ------------------------------------------------------------------ camera

        public Vec3 Forward
        {
            get { return new Vec3(-Math.Cos(_pitch) * Math.Cos(_yaw), -Math.Cos(_pitch) * Math.Sin(_yaw), -Math.Sin(_pitch)); }
        }

        public Vec3 Right { get { return Forward.Cross(Vec3.UnitZ).Normalized(); } }
        public Vec3 Up { get { return Right.Cross(Forward).Normalized(); } }
        public Vec3 CameraPosition { get { return _target - Forward * _dist; } }
        public double Distance { get { return _dist; } }

        void UpdateCamera()
        {
            Vec3 f = Forward;
            Vec3 pos = _target - f * _dist;
            _cam.Position = new Point3D(pos.X, pos.Y, pos.Z);
            _cam.LookDirection = new Vector3D(f.X * _dist, f.Y * _dist, f.Z * _dist);
            Vec3 up = Up;
            _cam.UpDirection = new Vector3D(up.X, up.Y, up.Z);
            _cam.NearPlaneDistance = Math.Max(0.05, _dist / 2000);
            _cam.FarPlaneDistance = _dist * 50 + SceneScale * 20;
            _head.Direction = new Vector3D(f.X + Right.X * 0.3 - Up.X * 0.4, f.Y + Right.Y * 0.3 - Up.Y * 0.4, f.Z + Right.Z * 0.3 - Up.Z * 0.4);
            UpdateLabels();
            var h = CameraChanged;
            if (h != null) h();
        }

        public void SetView(string name)
        {
            switch (name)
            {
                case "top": _yaw = -Math.PI / 2; _pitch = Math.PI / 2 - 1e-4; break;
                case "bottom": _yaw = -Math.PI / 2; _pitch = -Math.PI / 2 + 1e-4; break;
                case "front": _yaw = -Math.PI / 2; _pitch = 0; break;
                case "back": _yaw = Math.PI / 2; _pitch = 0; break;
                case "right": _yaw = 0; _pitch = 0; break;
                case "left": _yaw = Math.PI; _pitch = 0; break;
                default: _yaw = -Math.PI / 4; _pitch = 0.6155; break; // isometric
            }
            UpdateCamera();
        }

        /// <summary>Frames the given points (the whole set fits in the window).</summary>
        public void Fit(IList<Vec3> pts)
        {
            if (pts == null || pts.Count == 0) return;
            Vec3 c = Vec3.Zero;
            Vec3 min, max;
            MicroScribe.Model.AlignmentSolver.Bounds(pts, out min, out max);
            c = (min + max) * 0.5;
            _target = c;
            // distance so that every point projects inside the view (both directions)
            double tanH = Math.Tan(_cam.FieldOfView * Math.PI / 360);
            double aspect = ActualWidth > 10 && ActualHeight > 10 ? ActualWidth / ActualHeight : 1.6;
            double tanV = tanH / aspect;
            Vec3 f = Forward, r = Right, u = Up;
            double need = 10;
            foreach (var p in pts)
            {
                Vec3 w = p - c;
                double x = Math.Abs(w.Dot(r)), y = Math.Abs(w.Dot(u)), z = w.Dot(f);
                need = Math.Max(need, x / tanH - z);
                need = Math.Max(need, y / tanV - z);
            }
            _dist = need * 1.08 + 5;
            UpdateCamera();
        }

        public void SetOrbitCenter(Vec3 p) { _dist = Math.Max(10, (p - CameraPosition).Length); _target = p; UpdateCamera(); }

        /// <summary>Keeps the camera on the same part geometry when the display frame changes.</summary>
        public void ApplyFrameChange(Mat4 delta)
        {
            _target = delta.TransformPoint(_target);
            UpdateCamera();
        }

        /// <summary>
        /// SpaceMouse navigation in "object mode" (the model follows the cap). Inputs are rates of about ±1:
        /// panRight/panUp move the model on screen, zoomIn brings it closer, tilt/spin rotate it.
        /// </summary>
        public void Navigate(double panRight, double zoomIn, double panUp, double tilt, double spin, double dt)
        {
            double pan = _dist * 0.9 * dt;
            _target = _target - Right * (panRight * pan) - Up * (panUp * pan);
            _dist = Math.Max(1, Math.Min(1e5, _dist * Math.Exp(-zoomIn * 1.6 * dt)));
            _yaw += spin * 1.8 * dt;
            _pitch = Math.Max(-Math.PI / 2 + 1e-4, Math.Min(Math.PI / 2 - 1e-4, _pitch + tilt * 1.8 * dt));
            UpdateCamera();
        }

        // ------------------------------------------------------------------ rays

        /// <summary>Ray through a screen point (origin at the camera).</summary>
        public void Unproject(Point p, out Vec3 origin, out Vec3 dir)
        {
            double s = (ActualWidth / 2) / Math.Tan(_cam.FieldOfView * Math.PI / 360);
            double x = (p.X - ActualWidth / 2) / s, y = -(p.Y - ActualHeight / 2) / s;
            origin = CameraPosition;
            dir = (Forward + Right * x + Up * y).Normalized();
        }

        public static bool RayPlane(Vec3 o, Vec3 d, Vec3 planePt, Vec3 n, out Vec3 hit)
        {
            hit = Vec3.Zero;
            double den = d.Dot(n);
            if (Math.Abs(den) < 1e-12) return false;
            double t = (planePt - o).Dot(n) / den;
            if (t < 0) return false;
            hit = o + d * t;
            return true;
        }

        // ------------------------------------------------------------------ mouse

        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            base.OnMouseDown(e);
            Focus();
            _last = _down = e.GetPosition(this);
            _moved = false;
            if (e.ChangedButton == MouseButton.Left)
            {
                if (e.ClickCount == 2)
                {
                    var hd = DoubleClicked;
                    if (hd != null && ClickMode) { hd(_last); return; }
                    Vec3 hit; PickTag tag;
                    if (HitTest(_last, out hit, out tag)) SetOrbitCenter(hit);
                    return;
                }
                Vec3 h0; PickTag t0;
                if (HitTest(_last, out h0, out t0) && t0.Draggable)
                {
                    _dragTag = t0;
                    var ds = DragStarted; if (ds != null) ds(t0);
                    CaptureMouse();
                    return;
                }
                if (!ClickMode) _rotating = true;
                else { _rotating = false; }
            }
            else _panning = true;
            CaptureMouse();
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            Point p = e.GetPosition(this);
            if (_dragTag != null)
            {
                Vec3 o, d;
                Unproject(p, out o, out d);
                var h = Dragging; if (h != null) h(_dragTag, o, d);
                return;
            }
            if (!_rotating && !_panning)
            {
                var hv = Hover; if (hv != null) hv(p);
                return;
            }
            double dx = p.X - _last.X, dy = p.Y - _last.Y;
            if (Math.Abs(p.X - _down.X) + Math.Abs(p.Y - _down.Y) > 3) _moved = true;
            _last = p;
            if (_rotating && Keyboard.Modifiers != ModifierKeys.Shift)
            {
                _yaw -= dx * 0.008;
                _pitch = Math.Max(-Math.PI / 2 + 1e-4, Math.Min(Math.PI / 2 - 1e-4, _pitch + dy * 0.008));
            }
            else
            {
                double scale = 2 * _dist * Math.Tan(_cam.FieldOfView * Math.PI / 360) / Math.Max(1, ActualWidth);
                _target = _target - Right * (dx * scale) + Up * (dy * scale);
            }
            UpdateCamera();
        }

        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            base.OnMouseUp(e);
            if (_dragTag != null)
            {
                var t = _dragTag; _dragTag = null;
                ReleaseMouseCapture();
                var h = DragEnded; if (h != null) h(t);
                return;
            }
            bool wasRotating = _rotating;
            _rotating = _panning = false;
            ReleaseMouseCapture();
            if (e.ChangedButton == MouseButton.Left && (wasRotating || ClickMode) && !_moved)
            {
                Point p = e.GetPosition(this);
                Vec3 hit; PickTag tag;
                bool ok = HitTest(p, out hit, out tag);
                var h = Picked;
                if (h != null) h(ok ? tag : null, hit, p);
            }
        }

        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            base.OnMouseWheel(e);
            double f = Math.Pow(1.0015, -e.Delta);
            Point p = e.GetPosition(this);
            double scale = 2 * _dist * Math.Tan(_cam.FieldOfView * Math.PI / 360) / Math.Max(1, ActualWidth);
            Vec3 offset = Right * ((p.X - ActualWidth / 2) * scale) - Up * ((p.Y - ActualHeight / 2) * scale);
            _target = _target + offset * (1 - f);
            _dist = Math.Max(1, Math.Min(1e5, _dist * f));
            UpdateCamera();
        }

        public bool HitTest(Point p, out Vec3 hit, out PickTag tag)
        {
            hit = Vec3.Zero; tag = null;
            PickTag found = null; Point3D fp = new Point3D(); double best = double.MaxValue;
            VisualTreeHelper.HitTest(_vp, null, r =>
            {
                var rr = r as RayMeshGeometry3DHitTestResult;
                if (rr != null)
                {
                    PickTag t = null;
                    foreach (var lt in _layerTags.Values) if (lt.TryGetValue(rr.ModelHit, out t)) break;
                    if (t != null)
                    {
                        // handles and points win over surfaces when nearly coincident
                        double bias = t.Draggable || t.Anchor != null ? SceneScale * 0.03 : t.PointIndex != -1 ? SceneScale * 0.01 : 0;
                        double d = rr.DistanceToRayOrigin - bias;
                        if (d < best) { best = d; found = t; fp = rr.PointHit; }
                    }
                }
                return HitTestResultBehavior.Continue;
            }, new PointHitTestParameters(p));
            if (found == null) return false;
            tag = found; hit = new Vec3(fp.X, fp.Y, fp.Z);
            return true;
        }

        // ------------------------------------------------------------------ content

        public static GeometryModel3D Model(MeshBuilder mb, Material m, bool twoSided = false)
        {
            var g = new GeometryModel3D(mb.ToMesh(), m);
            if (twoSided) g.BackMaterial = m;
            return g;
        }

        /// <summary>Replaces a layer's content, pick tags and labels.</summary>
        public void SetLayer(string layer, Model3D content, Dictionary<Model3D, PickTag> tags = null, List<Label3D> labels = null)
        {
            _layers[layer].Content = content;
            _layerTags[layer].Clear();
            if (tags != null) foreach (var kv in tags) _layerTags[layer][kv.Key] = kv.Value;
            _layerLabels[layer].Clear();
            if (labels != null) _layerLabels[layer].AddRange(labels);
            UpdateLabels();
        }

        // ------------------------------------------------------------------ labels

        public bool ShowLabels = true;

        public bool Project(Vec3 p, out Point screen)
        {
            Vec3 w = p - CameraPosition;
            double depth = w.Dot(Forward);
            screen = new Point();
            if (depth <= _cam.NearPlaneDistance) return false;
            double s = (ActualWidth / 2) / Math.Tan(_cam.FieldOfView * Math.PI / 360);
            screen = new Point(ActualWidth / 2 + w.Dot(Right) / depth * s, ActualHeight / 2 - w.Dot(Up) / depth * s);
            return true;
        }

        static readonly Brush LabelBg = Ui.Frozen(Color.FromArgb(150, 15, 18, 24));

        void UpdateLabels()
        {
            _labels.Children.Clear();
            if (ActualWidth < 10) return;
            foreach (var kv in _layerLabels)
            {
                bool always = kv.Key == "dims" || kv.Key == "tool";
                if (!ShowLabels && !always) continue;
                foreach (var l in kv.Value)
                {
                    Point sp;
                    if (!Project(l.Position, out sp)) continue;
                    if (sp.X < -50 || sp.Y < -20 || sp.X > ActualWidth + 50 || sp.Y > ActualHeight + 20) continue;
                    var tb = new TextBlock
                    {
                        Text = l.Text,
                        Foreground = l.Brush ?? Brushes.White,
                        FontSize = l.Bold ? 13 : 11.5,
                        FontWeight = l.Bold ? FontWeights.SemiBold : FontWeights.Normal,
                        Padding = new Thickness(5, 2, 5, 2),
                        Background = l.Background ?? LabelBg
                    };
                    Canvas.SetLeft(tb, sp.X + (always ? -10 : 8));
                    Canvas.SetTop(tb, sp.Y - (always ? 10 : 18));
                    _labels.Children.Add(tb);
                }
            }
        }

        public void RefreshLabels() { UpdateLabels(); }
    }
}
