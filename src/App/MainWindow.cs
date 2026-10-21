using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using MicroScribe.Core;
using MicroScribe.Driver;
using MicroScribe.Geometry;
using MicroScribe.Model;
using MicroScribe.Export;

namespace MicroScribe.App
{
    /// <summary>Main application window (layout, menus, toolbar, feature list).</summary>
    public sealed partial class MainWindow : Window
    {
        readonly AppSettings _s;
        Project _proj = new Project();
        string _projPath;
        bool _dirty;

        // layout pieces
        SceneView _view;
        ComboBox _driverBox, _portBox, _tipBox;
        Button _connectBtn;
        TextBlock _statusText, _readout, _prompt, _deviation, _frameText;
        Ellipse _led1, _led2, _ledConn;
        ListBox _featureList;
        TabControl _tabs;
        StackPanel _measurePalette;
        TextBlock _statusBar;
        Border _promptBar;
        readonly DispatcherTimer _uiTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };

        public MainWindow(AppSettings settings, string openPath)
        {
            _s = settings;
            Title = Installer.AppName;
            Width = _s.WindowWidth; Height = _s.WindowHeight;
            if (_s.WindowMaximized) WindowState = WindowState.Maximized;
            FontFamily = new FontFamily("Segoe UI");
            FontSize = 12.5;
            Background = Ui.PanelBg;
            UseLayoutRounding = true;
            SnapsToDevicePixels = true;
            try
            {
                var ico = System.Drawing.Icon.ExtractAssociatedIcon(Installer.CurrentExe);
                Icon = System.Windows.Interop.Imaging.CreateBitmapSourceFromHIcon(ico.Handle, Int32Rect.Empty, System.Windows.Media.Imaging.BitmapSizeOptions.FromEmptyOptions());
            }
            catch { }

            Ui.Inch = _s.InchDisplay;
            Sounds.Enabled = _s.BeepOnCapture;
            Content = BuildLayout();
            PreviewKeyDown += OnPreviewKey;
            Closing += OnClosing;
            SourceInitialized += (s, e) => InitSpaceMouse();
            Loaded += (s, e) =>
            {
                RebuildAll();
                _view.SetView("iso");
                FitView();
                if (!string.IsNullOrEmpty(openPath) && File.Exists(openPath)) OpenProject(openPath);
                Status(AppSettings.IsPortable ? "Portable mode — settings: " + AppSettings.SettingsPath : "Installed — settings: " + AppSettings.SettingsPath);
            };
            _uiTimer.Tick += (s, e) => UiTick();
            _uiTimer.Start();
        }

        // ------------------------------------------------------------------ layout

        UIElement BuildLayout()
        {
            var dock = new DockPanel();
            var menu = BuildMenu();
            DockPanel.SetDock(menu, Dock.Top);
            dock.Children.Add(menu);
            var tb = BuildToolbar();
            DockPanel.SetDock(tb, Dock.Top);
            dock.Children.Add(tb);

            _statusBar = new TextBlock { Margin = new Thickness(8, 3, 8, 3), Foreground = Ui.Muted, TextTrimming = TextTrimming.CharacterEllipsis };
            var sb = new Border { Child = _statusBar, BorderBrush = Ui.Stroke, BorderThickness = new Thickness(0, 1, 0, 0), Background = Brushes.White };
            DockPanel.SetDock(sb, Dock.Bottom);
            dock.Children.Add(sb);

            var grid = new Grid();
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(270), MinWidth = 200 });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(370), MinWidth = 280 });

            var left = BuildLeftPanel();
            Grid.SetColumn(left, 0); grid.Children.Add(left);
            var s1 = new GridSplitter { Width = 5, HorizontalAlignment = HorizontalAlignment.Stretch, Background = Ui.PanelBg };
            Grid.SetColumn(s1, 1); grid.Children.Add(s1);
            var center = BuildCenter();
            Grid.SetColumn(center, 2); grid.Children.Add(center);
            var s2 = new GridSplitter { Width = 5, HorizontalAlignment = HorizontalAlignment.Stretch, Background = Ui.PanelBg };
            Grid.SetColumn(s2, 3); grid.Children.Add(s2);
            var right = BuildRightPanel();
            Grid.SetColumn(right, 4); grid.Children.Add(right);
            dock.Children.Add(grid);
            return dock;
        }

        Menu BuildMenu()
        {
            var m = new Menu { Background = Brushes.White, Padding = new Thickness(2) };
            Func<string, MenuItem> top = h => { var i = new MenuItem { Header = h }; m.Items.Add(i); return i; };
            Action<MenuItem, string, string, Action> item = (parent, h, key, a) =>
            {
                var i = new MenuItem { Header = h, InputGestureText = key };
                i.Click += (s, e) => a();
                parent.Items.Add(i);
            };

            var file = top("_File");
            item(file, "_New project", "Ctrl+N", NewProject);
            item(file, "_Open...", "Ctrl+O", () => OpenProject(null));
            item(file, "_Save", "Ctrl+S", () => SaveProject(false));
            item(file, "Save _as...", "", () => SaveProject(true));
            var recent = new MenuItem { Header = "Recent" };
            recent.SubmenuOpened += (s, e) =>
            {
                recent.Items.Clear();
                foreach (var r in _s.RecentFiles)
                {
                    string path = r;
                    var ri = new MenuItem { Header = path };
                    ri.Click += (s2, e2) => OpenProject(path);
                    recent.Items.Add(ri);
                }
                if (recent.Items.Count == 0) recent.Items.Add(new MenuItem { Header = "(none)", IsEnabled = false });
            };
            recent.Items.Add(new MenuItem { Header = "(none)" });
            file.Items.Add(recent);
            file.Items.Add(new Separator());
            item(file, "Export _STEP (surfaces + reference geometry)...", "Ctrl+E", ExportStep);
            item(file, "Export points CSV...", "", () => ExportPoints("csv"));
            item(file, "Export point cloud XYZ (ScanTo3D / curve through points)...", "", () => ExportPoints("xyz"));
            item(file, "Inspection report...", "", ExportReport);
            file.Items.Add(new Separator());
            item(file, "E_xit", "Alt+F4", Close);

            var meas = top("_Measure");
            foreach (var k in MeasureKinds)
            {
                var kk = k;
                var mi = new MenuItem { Header = k.HasValue ? (k.Value == FeatureKind.Surface ? "Freeform surface (trace curves)" : k.Value.ToString()) : "Auto-detect", Icon = Ui.Icon(k, 16) };
                mi.Click += (s, e) => StartMeasure(kk);
                meas.Items.Add(mi);
            }
            meas.Items.Add(new Separator());
            item(meas, "Capture point", "Space / Pedal 1", () => CapturePressed());
            item(meas, "Finish feature", "Enter / Pedal 2", FinishMeasure);
            item(meas, "Next traced curve (freeform)", "N", NextSection);
            item(meas, "Undo last point", "Backspace", UndoPoint);
            item(meas, "Cancel", "Esc", CancelMeasure);
            meas.Items.Add(new Separator());
            item(meas, "Measure distance (click entities)", "D", () => SetTool(Tool.Distance));
            item(meas, "Measure angle", "A", () => SetTool(Tool.Angle));
            item(meas, "Measure diameter", "R", () => SetTool(Tool.Diameter));
            item(meas, "Show coordinates", "X", () => SetTool(Tool.Coordinate));

            var view = top("_View");
            foreach (var v in new[] { "Iso", "Top", "Front", "Right", "Left", "Back", "Bottom" })
            {
                string vv = v.ToLowerInvariant();
                item(view, v, "", () => _view.SetView(vv));
            }
            view.Items.Add(new Separator());
            item(view, "Fit (work area, or all features)", "F", FitView);
            item(view, "Fit features", "Shift+F", FitFeatures);
            item(view, "Set up work area (sketch on the table)...", "", () => { _tabs.SelectedIndex = 4; SetTool(Tool.SketchArea); });
            var showArm = new MenuItem { Header = "Show arm", IsCheckable = true, IsChecked = _s.ShowArm };
            showArm.Click += (s, e) => { _s.ShowArm = showArm.IsChecked; };
            view.Items.Add(showArm);
            var labels = new MenuItem { Header = "Show labels", IsCheckable = true, IsChecked = _s.ShowLabels };
            labels.Click += (s, e) => { _s.ShowLabels = labels.IsChecked; _view.ShowLabels = labels.IsChecked; _view.RefreshLabels(); };
            view.Items.Add(labels);
            var inch = new MenuItem { Header = "Inches", IsCheckable = true, IsChecked = _s.InchDisplay };
            inch.Click += (s, e) => SetUnits(inch.IsChecked);
            Ui.UnitsChanged += () => inch.IsChecked = Ui.Inch;
            view.Items.Add(inch);

            var tools = top("_Tools");
            item(tools, "Tip calibration (pivot)...", "", () => { _tabs.SelectedIndex = 3; });
            item(tools, "Settings...", "", ShowSettings);
            tools.Items.Add(new Separator());
            item(tools, Installer.IsInstalledCopy() ? "Reinstall / update shortcuts..." : "Install on this PC...", "", DoInstall);
            item(tools, "Uninstall...", "", DoUninstall);

            var help = top("_Help");
            item(help, "Workflow & shortcuts", "F1", ShowHelp);
            item(help, "Protocol documentation (folder)", "", () =>
            {
                string d = System.IO.Path.Combine(AppSettings.ExeDir, "docs");
                if (!Directory.Exists(d)) d = System.IO.Path.GetFullPath(System.IO.Path.Combine(AppSettings.ExeDir, @"..\docs"));
                if (Directory.Exists(d)) System.Diagnostics.Process.Start("explorer.exe", "\"" + d + "\"");
            });
            item(help, "About", "", () => MessageBox.Show(this, Installer.AppName + " " + Installer.Version + "\n© 2026 Marconnet Technologies — MIT license" +
                "\n\nIndependent, open-source software for serial MicroScribe digitizers, written from public" +
                "\nprotocol documentation and observation of the device. Not affiliated with or endorsed by" +
                "\nthe MicroScribe trademark owner.\n\nSettings: " + AppSettings.SettingsPath, "About", MessageBoxButton.OK, MessageBoxImage.Information));
            return m;
        }

        static readonly FeatureKind?[] MeasureKinds =
        {
            FeatureKind.Point, FeatureKind.Line, FeatureKind.Plane, FeatureKind.Circle, FeatureKind.Cylinder, FeatureKind.Cone,
            FeatureKind.Sphere, FeatureKind.Curve, FeatureKind.Surface, null
        };

        /// <summary>mm / inch display toggle (all values are stored in mm).</summary>
        void SetUnits(bool inch)
        {
            if (Ui.Inch == inch) return;
            _s.InchDisplay = inch;
            Ui.SetInch(inch);
            RebuildAll();
            RebuildArmPanel();
            RebuildWorkspacePanel();
            Status("Units: " + (inch ? "inches" : "millimetres"));
        }

        FrameworkElement UnitToggle()
        {
            var sp = new StackPanel { Orientation = Orientation.Horizontal, Margin = new Thickness(6, 2, 2, 2), VerticalAlignment = VerticalAlignment.Center };
            var mm = new ToggleButton { Content = "mm", Width = 40, Padding = new Thickness(2), Focusable = false, IsChecked = !Ui.Inch, ToolTip = "Show millimetres" };
            var inch = new ToggleButton { Content = "in", Width = 40, Padding = new Thickness(2), Focusable = false, IsChecked = Ui.Inch, ToolTip = "Show inches" };
            mm.Click += (s, e) => SetUnits(false);
            inch.Click += (s, e) => SetUnits(true);
            Ui.UnitsChanged += () => { mm.IsChecked = !Ui.Inch; inch.IsChecked = Ui.Inch; };
            sp.Children.Add(mm); sp.Children.Add(inch);
            return sp;
        }

        FrameworkElement BuildToolbar()
        {
            var bar = new WrapPanel { Margin = new Thickness(6, 4, 6, 4) };
            _driverBox = new ComboBox { Width = 120, Margin = new Thickness(2), ToolTip = "Driver" };
            _driverBox.Items.Add("MicroScribe (serial)");
            _driverBox.Items.Add("Simulator");
            _driverBox.SelectedIndex = _s.Driver == "simulator" ? 1 : 0;
            _driverBox.SelectionChanged += (s, e) => { _s.Driver = _driverBox.SelectedIndex == 1 ? "simulator" : "native"; _portBox.IsEnabled = _driverBox.SelectedIndex != 1; };
            bar.Children.Add(_driverBox);
            _portBox = new ComboBox { Width = 80, Margin = new Thickness(2), IsEditable = true, ToolTip = "Serial port" };
            RefreshPorts();
            _portBox.DropDownOpened += (s, e) => RefreshPorts();
            _portBox.IsEnabled = _driverBox.SelectedIndex != 1;
            bar.Children.Add(_portBox);
            _connectBtn = Ui.Btn("Connect", ToggleConnect, "Connect / disconnect the arm");
            _connectBtn.FontWeight = FontWeights.SemiBold;
            bar.Children.Add(_connectBtn);
            _ledConn = Led(Colors.Gray, "Connection");
            bar.Children.Add(_ledConn);
            _statusText = new TextBlock { Text = "Not connected", VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(6, 0, 12, 0), MaxWidth = 380, TextTrimming = TextTrimming.CharacterEllipsis };
            bar.Children.Add(_statusText);
            bar.Children.Add(new TextBlock { Text = "Pedals", VerticalAlignment = VerticalAlignment.Center, Foreground = Ui.Muted, Margin = new Thickness(6, 0, 2, 0) });
            _led1 = Led(Colors.LightGray, "Pedal / button 1");
            _led2 = Led(Colors.LightGray, "Pedal / button 2");
            bar.Children.Add(_led1); bar.Children.Add(_led2);
            bar.Children.Add(new Separator { Width = 1, Margin = new Thickness(8, 2, 8, 2), Background = Ui.Stroke });
            bar.Children.Add(new TextBlock { Text = "Tip", VerticalAlignment = VerticalAlignment.Center, Foreground = Ui.Muted, Margin = new Thickness(2, 0, 4, 0) });
            _tipBox = new ComboBox { Width = 190, Margin = new Thickness(2) };
            RefreshTips();
            _tipBox.SelectionChanged += (s, e) =>
            {
                var t = _tipBox.SelectedItem as TipProfile;
                if (t == null) return;
                _s.ActiveTip = t.Name;
                ApplyTip();
            };
            bar.Children.Add(_tipBox);
            bar.Children.Add(Ui.Btn("Rehome", Rehome, "Re-zero the encoders: put the arm in its home (rest) position first"));
            bar.Children.Add(UnitToggle());
            bar.Children.Add(new Separator { Width = 1, Margin = new Thickness(8, 2, 8, 2) });
            bar.Children.Add(Ui.Btn("Open", () => OpenProject(null)));
            bar.Children.Add(Ui.Btn("Save", () => SaveProject(false)));
            var exp = Ui.Btn("Export STEP", ExportStep, "Surfaces + axes/centres/curves for SolidWorks (Ctrl+E)");
            exp.Foreground = Ui.Accent; exp.FontWeight = FontWeights.SemiBold;
            bar.Children.Add(exp);
            var border = new Border { Child = bar, Background = Brushes.White, BorderBrush = Ui.Stroke, BorderThickness = new Thickness(0, 0, 0, 1) };
            return border;
        }

        static Ellipse Led(Color c, string tip)
        {
            return new Ellipse { Width = 12, Height = 12, Fill = new SolidColorBrush(c), Stroke = Brushes.DimGray, StrokeThickness = 0.5, Margin = new Thickness(3, 0, 3, 0), VerticalAlignment = VerticalAlignment.Center, ToolTip = tip };
        }

        void RefreshPorts()
        {
            string cur = _portBox.Text;
            _portBox.Items.Clear();
            foreach (var p in SerialPort.GetPortNames().Distinct().OrderBy(p => p.Length).ThenBy(p => p)) _portBox.Items.Add(p);
            _portBox.Text = string.IsNullOrEmpty(cur) ? _s.Port : cur;
        }

        void RefreshTips()
        {
            _tipBox.Items.Clear();
            foreach (var t in _s.Tips) _tipBox.Items.Add(t);
            _tipBox.SelectedItem = _s.CurrentTip;
        }

        FrameworkElement BuildLeftPanel()
        {
            var dock = new DockPanel { Margin = new Thickness(8, 8, 2, 8) };
            var palette = new StackPanel();
            palette.Children.Add(Ui.Header("MEASURE"));
            var wrap = new UniformGrid { Columns = 3 };
            foreach (var k in MeasureKinds)
            {
                var kk = k;
                var b = new Button { Margin = new Thickness(2), Padding = new Thickness(2, 6, 2, 4), Focusable = false, ToolTip = Tooltip(k) };
                var sp = new StackPanel();
                var icon = Ui.Icon(k, 22);
                icon.HorizontalAlignment = HorizontalAlignment.Center;
                sp.Children.Add(icon);
                sp.Children.Add(new TextBlock { Text = k.HasValue ? (k.Value == FeatureKind.Surface ? "Freeform" : k.Value.ToString()) : "Auto", HorizontalAlignment = HorizontalAlignment.Center, FontSize = 11.5, Margin = new Thickness(0, 3, 0, 0) });
                b.Content = sp;
                b.Click += (s, e) => StartMeasure(kk);
                wrap.Children.Add(b);
            }
            palette.Children.Add(wrap);
            _measurePalette = palette;
            DockPanel.SetDock(palette, Dock.Top);
            dock.Children.Add(palette);

            var hdr = new DockPanel { Margin = new Thickness(0, 12, 0, 4) };
            var add = new StackPanel { Orientation = Orientation.Horizontal };
            add.Children.Add(Ui.Btn("✕", DeleteSelected, "Delete selected feature (Del)"));
            DockPanel.SetDock(add, Dock.Right);
            hdr.Children.Add(add);
            hdr.Children.Add(new TextBlock { Text = "FEATURES", FontWeight = FontWeights.SemiBold, Foreground = Ui.Muted, VerticalAlignment = VerticalAlignment.Center });
            DockPanel.SetDock(hdr, Dock.Top);
            dock.Children.Add(hdr);

            _featureList = new ListBox { BorderBrush = Ui.Stroke, HorizontalContentAlignment = HorizontalAlignment.Stretch, SelectionMode = SelectionMode.Extended };
            ScrollViewer.SetHorizontalScrollBarVisibility(_featureList, ScrollBarVisibility.Disabled);
            _featureList.SelectionChanged += (s, e) =>
            {
                if (_rebuildingList) return;
                // measuring tools: clicking a feature in the list picks it
                if (_tool != Tool.Select && _tool != Tool.SketchArea && _tool != Tool.TraceArea && _tool != Tool.ProbeTable && e.AddedItems.Count == 1)
                {
                    var fi = ((FrameworkElement)e.AddedItems[0]).Tag as Feature;
                    if (fi != null) { PickEntity(new EntityRef { FeatureId = fi.Id }); return; }
                }
                var it = _featureList.SelectedItem as FrameworkElement;
                if (e.AddedItems.Count == 1) it = (FrameworkElement)e.AddedItems[0];
                var f = it != null ? it.Tag as Feature : null;
                if (f != _selected) { _selected = f; RebuildScene(); RebuildFeaturePanel(); }
                RebuildMeasurePanel();
            };
            var ctx = new ContextMenu();
            Action<string, Action> ci = (h, a) => { var mi = new MenuItem { Header = h }; mi.Click += (s, e) => a(); ctx.Items.Add(mi); };
            ci("Add more points", () => { if (_selected != null) ResumeMeasure(_selected); });
            ci("Rename...", RenameSelected);
            ci("Toggle visibility", () => { if (_selected != null) { _selected.Visible = !_selected.Visible; Changed(); } });
            ci("Toggle export", () => { if (_selected != null) { _selected.Export = !_selected.Export; Changed(); } });
            ci("Delete", DeleteSelected);
            _featureList.ContextMenu = ctx;
            dock.Children.Add(_featureList);
            return dock;
        }

        static string Tooltip(FeatureKind? k)
        {
            if (!k.HasValue) return "Probe any surface; the best-fitting primitive is chosen automatically";
            switch (k.Value)
            {
                case FeatureKind.Point: return "Single point (averaged if several are captured)";
                case FeatureKind.Line: return "Line / edge — 2+ points";
                case FeatureKind.Plane: return "Plane — 3+ points spread over the face";
                case FeatureKind.Circle: return "Circle (hole/boss edge) — 3+ points at one height";
                case FeatureKind.Cylinder: return "Cylinder — 5+ points: two or more rings at different heights";
                case FeatureKind.Cone: return "Cone — 6+ points at two or more heights";
                case FeatureKind.Sphere: return "Sphere — 4+ points spread over the ball";
                case FeatureKind.Curve: return "Curve scan — hold pedal 1 and trace; points every N mm (spline or linear)";
                case FeatureKind.Surface: return "Freeform surface — trace several curves across the surface (hold pedal 1 for each), then finish";
            }
            return "";
        }

        FrameworkElement BuildCenter()
        {
            var g = new Grid { Margin = new Thickness(2, 8, 2, 8) };
            _view = new SceneView();
            _view.ShowLabels = _s.ShowLabels;
            _view.Picked += OnScenePicked;
            _view.Hover += SketchHover;
            _view.DoubleClicked += p => { if (_tool == Tool.SketchArea) FinishSketch(); };
            _view.DragStarted += OnDragStarted;
            _view.Dragging += OnDragging;
            _view.DragEnded += OnDragEnded;
            var vb = new Border { Child = _view, CornerRadius = new CornerRadius(6), BorderBrush = Ui.Stroke, BorderThickness = new Thickness(1), ClipToBounds = true };
            g.Children.Add(vb);
            g.Children.Add(BuildToolStrip());

            // HUD: live readout (top-left, right of the tool strip)
            var hud = new StackPanel { Margin = new Thickness(60, 12, 0, 0), HorizontalAlignment = HorizontalAlignment.Left, VerticalAlignment = VerticalAlignment.Top, IsHitTestVisible = false };
            _readout = new TextBlock { FontFamily = Ui.Mono, FontSize = 17, Foreground = Brushes.White, Text = "X  —\nY  —\nZ  —" };
            _deviation = new TextBlock { FontFamily = Ui.Mono, FontSize = 13, Foreground = new SolidColorBrush(Color.FromRgb(250, 204, 21)), Margin = new Thickness(0, 4, 0, 0) };
            _frameText = new TextBlock { FontSize = 11, Foreground = new SolidColorBrush(Color.FromRgb(160, 170, 185)), Margin = new Thickness(0, 4, 0, 0), MaxWidth = 420, TextWrapping = TextWrapping.Wrap };
            hud.Children.Add(_readout); hud.Children.Add(_deviation); hud.Children.Add(_frameText);
            g.Children.Add(new Border { Child = hud, IsHitTestVisible = false });

            // view buttons (top-right)
            var vbar = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, VerticalAlignment = VerticalAlignment.Top, Margin = new Thickness(0, 10, 12, 0) };
            foreach (var v in new[] { "Iso", "Top", "Front", "Right" })
            {
                string vv = v.ToLowerInvariant();
                var b = ViewButton(v, () => _view.SetView(vv));
                vbar.Children.Add(b);
            }
            vbar.Children.Add(ViewButton("Fit", FitView));
            g.Children.Add(vbar);

            // prompt (bottom)
            _prompt = new TextBlock { Foreground = Brushes.White, FontSize = 14, TextWrapping = TextWrapping.Wrap };
            _promptBar = new Border
            {
                Child = _prompt, Background = new SolidColorBrush(Color.FromArgb(200, 30, 64, 175)), CornerRadius = new CornerRadius(6),
                Padding = new Thickness(14, 8, 14, 8), Margin = new Thickness(14), VerticalAlignment = VerticalAlignment.Bottom,
                HorizontalAlignment = HorizontalAlignment.Center, Visibility = Visibility.Collapsed
            };
            var promptPanel = new StackPanel { VerticalAlignment = VerticalAlignment.Bottom, HorizontalAlignment = HorizontalAlignment.Center };
            promptPanel.Children.Add(_promptBar);
            var pbtns = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Center, Margin = new Thickness(0, 0, 0, 14) };
            pbtns.Children.Add(Ui.Btn("Capture (Space)", () => CapturePressed()));
            pbtns.Children.Add(Ui.Btn("Undo (⌫)", UndoPoint));
            pbtns.Children.Add(Ui.Btn("Finish (Enter)", FinishMeasure));
            pbtns.Children.Add(Ui.Btn("Cancel (Esc)", CancelMeasure));
            pbtns.Visibility = Visibility.Collapsed;
            promptPanel.Children.Add(pbtns);
            _promptButtons = pbtns;
            g.Children.Add(promptPanel);
            return g;
        }
        StackPanel _promptButtons;

        static Button ViewButton(string text, Action a)
        {
            var b = new Button
            {
                Content = text, Margin = new Thickness(2), Padding = new Thickness(8, 2, 8, 2), Focusable = false,
                Background = new SolidColorBrush(Color.FromArgb(160, 40, 46, 58)), Foreground = Brushes.White,
                BorderBrush = new SolidColorBrush(Color.FromArgb(120, 255, 255, 255))
            };
            b.Click += (s, e) => a();
            return b;
        }

        FrameworkElement BuildRightPanel()
        {
            _tabs = new TabControl { Margin = new Thickness(2, 8, 8, 8), Background = Ui.PanelBg };
            _tabs.Items.Add(new TabItem { Header = "Feature", Content = Scroll(BuildFeaturePanel()) });
            _tabs.Items.Add(new TabItem { Header = "Measure", Content = Scroll(BuildRelationsPanel()) });
            _tabs.Items.Add(new TabItem { Header = "Alignment", Content = Scroll(BuildAlignmentPanel()) });
            _tabs.Items.Add(new TabItem { Header = "Arm & tip", Content = Scroll(BuildArmPanel()) });
            _tabs.Items.Add(new TabItem { Header = "Workspace", Content = Scroll(BuildWorkspacePanel()) });
            _tabs.SelectionChanged += (s, e) =>
            {
                if (e.Source != _tabs) return;
                RebuildOverlay();
                BuildWorkArea(DisplayMatrix);
                if (_tabs.SelectedIndex == 4) RebuildWorkspacePanel();
            };
            return _tabs;
        }

        static ScrollViewer Scroll(UIElement e)
        {
            return new ScrollViewer { Content = e, VerticalScrollBarVisibility = ScrollBarVisibility.Auto, Padding = new Thickness(8) };
        }

        // ------------------------------------------------------------------ feature list

        void RebuildFeatureList()
        {
            var sel = _selected;
            bool inch = _s.InchDisplay;
            _rebuildingList = true;
            _featureList.Items.Clear();
            foreach (var f in _proj.Features)
            {
                var g = new Grid { Tag = f, Margin = new Thickness(0, 2, 0, 2) };
                g.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
                g.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
                var icon = Ui.Icon(f.Kind, 18);
                icon.Margin = new Thickness(2, 2, 8, 2);
                icon.VerticalAlignment = VerticalAlignment.Top;
                if (!f.Visible) icon.Opacity = 0.35;
                g.Children.Add(icon);
                var sp = new StackPanel();
                var name = new TextBlock { Text = f.Name + (f == _active ? "  ● measuring" : "") + (f.Export ? "" : "  (no export)"), FontWeight = FontWeights.SemiBold, TextTrimming = TextTrimming.CharacterEllipsis };
                if (f == _active) name.Foreground = Ui.Accent;
                sp.Children.Add(name);
                sp.Children.Add(new TextBlock { Text = Summary(f, inch), Foreground = f.Geometry == null ? Brushes.IndianRed : Ui.Muted, FontSize = 11.5, TextTrimming = TextTrimming.CharacterEllipsis });
                Grid.SetColumn(sp, 1);
                g.Children.Add(sp);
                _featureList.Items.Add(g);
                if (f == sel) _featureList.SelectedItem = g;
            }
            _rebuildingList = false;
        }
        bool _rebuildingList;

        /// <summary>Features currently selected in the list (Ctrl/Shift multi-select), in selection order.</summary>
        List<Feature> SelectedFeatures()
        {
            return _featureList.SelectedItems.Cast<FrameworkElement>().Select(x => x.Tag as Feature).Where(x => x != null).ToList();
        }

        string Summary(Feature f, bool inch)
        {
            int n = f.ActivePoints.Count();
            string pts = f.Origin == FeatureOrigin.Constructed ? "constructed" : n + " pt" + (n == 1 ? "" : "s");
            var g = f.Geometry;
            if (g == null) return pts + (f.FitError != null ? " · " + f.FitError : "");
            string u = Ui.Unit(inch);
            string val = "";
            if (g is CirclePrim) val = "Ø" + Ui.Mm(2 * ((CirclePrim)g).Radius, inch);
            else if (g is CylinderPrim) val = "Ø" + Ui.Mm(2 * ((CylinderPrim)g).Radius, inch);
            else if (g is SpherePrim) val = "Ø" + Ui.Mm(2 * ((SpherePrim)g).Radius, inch);
            else if (g is ConePrim) val = (2 * ((ConePrim)g).HalfAngle * 180 / Math.PI).ToString("0.00", Ui.CI) + "°";
            else if (g is LinePrim) val = "L " + Ui.Mm(((LinePrim)g).TMax - ((LinePrim)g).TMin, inch);
            else if (g is CurvePrim) val = (((CurvePrim)g).Degree <= 1 ? "linear" : "spline") + " L " + Ui.L(((CurvePrim)g).Length);
            else if (g is SurfacePrim) val = ((SurfacePrim)g).SectionCount + " curves";
            else if (g is PlanePrim && f.CustomBounds) val = Ui.L(f.BW) + "×" + Ui.L(f.BH);
            string formLabel = g is CurvePrim || g is SurfacePrim ? " · dev " : " · form ";
            string form = g.Stats != null && g.Stats.N > Fitter.MinPoints(f.Kind) ? formLabel + Ui.Mm(g.Stats.Form, inch) : "";
            bool warn = f.Origin == FeatureOrigin.Measured && FitQuality.Warnings(f.Kind, f.RawFit, f.ActivePoints.Select(q => q.Position).ToList())
                .Any(w => !w.StartsWith("Minimum"));
            return (warn ? "⚠ " : "") + pts + (val != "" ? " · " + val : "") + form + (val != "" || form != "" ? " " + u : "");
        }

        // ------------------------------------------------------------------ misc

        void Status(string s) { _statusBar.Text = s; }

        void Changed()
        {
            _dirty = true;
            Mat4 before = DisplayMatrix;
            FeatureSolver.SolveAll(_proj);
            Mat4 after = DisplayMatrix;
            if (_view != null)
            {
                Mat4 delta = after * before.RigidInverse();
                bool same = true;
                for (int i = 0; i < 16; i++) if (Math.Abs(delta.M[i] - (i % 5 == 0 ? 1 : 0)) > 1e-9) same = false;
                if (!same) _view.ApplyFrameChange(delta);
            }
            RebuildAll();
        }

        void RebuildAll()
        {
            RebuildFeatureList();
            RebuildScene();
            RebuildFeaturePanel();
            RebuildMeasurePanel();
            RebuildAlignmentPanel();
            UpdateTitle();
        }

        void UpdateTitle()
        {
            Title = (_projPath != null ? System.IO.Path.GetFileName(_projPath) : _proj.Title) + (_dirty ? " *" : "") + " — " + Installer.AppName;
        }

        void OnPreviewKey(object sender, KeyEventArgs e)
        {
            if (Keyboard.FocusedElement is TextBox) return;
            bool ctrl = (Keyboard.Modifiers & ModifierKeys.Control) != 0;
            bool shift = (Keyboard.Modifiers & ModifierKeys.Shift) != 0;
            switch (e.Key)
            {
                case Key.Space: CapturePressed(); e.Handled = true; break;
                case Key.Enter: if (!ToolFinish() && !CheckFinish() && !ModelPickFinish()) FinishMeasure(); e.Handled = true; break;
                case Key.Back: if (!ToolUndo() && !CheckUndo() && !ModelPickUndo()) UndoPoint(); e.Handled = true; break;
                case Key.Escape:
                    if (_tool == Tool.ModelPick || _pairProbing) CancelPointPairs();
                    else if (_tool != Tool.Select) SetTool(Tool.Select);
                    else if (_check != CheckMode.None) ToggleCheck(_check);
                    else CancelMeasure();
                    e.Handled = true; break;
                case Key.Delete:
                    if (_selectedDim != 0 && _tabs.SelectedIndex == 1) DeleteDimension(_selectedDim); else DeleteSelected();
                    e.Handled = true; break;
                case Key.F: if (!ctrl) { if (shift) FitFeatures(); else FitView(); e.Handled = true; } break;
                case Key.F1: ShowHelp(); e.Handled = true; break;
                case Key.S: if (ctrl) { SaveProject(false); e.Handled = true; } break;
                case Key.O: if (ctrl) { OpenProject(null); e.Handled = true; } break;
                case Key.N:
                    if (ctrl) NewProject(); else NextSection();
                    e.Handled = true; break;
                case Key.E: if (ctrl) { ExportStep(); e.Handled = true; } break;
                case Key.D: if (!ctrl) { SetTool(Tool.Distance); e.Handled = true; } break;
                case Key.A: if (!ctrl) { SetTool(Tool.Angle); e.Handled = true; } break;
                case Key.R: if (!ctrl) { SetTool(Tool.Diameter); e.Handled = true; } break;
                case Key.X: if (!ctrl) { SetTool(Tool.Coordinate); e.Handled = true; } break;
            }
        }

        void OnClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (_dirty && _proj.Features.Count > 0)
            {
                var r = MessageBox.Show(this, "Save changes to the project before closing?", Installer.AppName, MessageBoxButton.YesNoCancel, MessageBoxImage.Question);
                if (r == MessageBoxResult.Cancel) { e.Cancel = true; return; }
                if (r == MessageBoxResult.Yes && !SaveProject(false)) { e.Cancel = true; return; }
            }
            DisconnectArm();
            _s.WindowMaximized = WindowState == WindowState.Maximized;
            if (WindowState == WindowState.Normal) { _s.WindowWidth = Width; _s.WindowHeight = Height; }
            _s.Port = _portBox.Text;
            _s.Save();
        }

        void ShowHelp()
        {
            MessageBox.Show(this,
                "WORKFLOW\n" +
                "1. Connect (native driver, COM port). Power the arm on in its home (rest) pose.\n" +
                "2. Pick a feature type on the left (or Auto). Probe points:\n" +
                "     Pedal 1 / Space = capture point     Pedal 2 / Enter = finish feature\n" +
                "     Backspace = undo last point          Esc = cancel\n" +
                "   Curve: hold pedal 1 and trace; a point is added every few mm (spline or linear).\n" +
                "   Freeform: trace several curves across the surface (hold pedal 1 for each,\n" +
                "   N = next curve from the keyboard), then finish — a B-spline surface is skinned.\n" +
                "3. Results update live. Exclude bad points in the Feature tab. A happy chime =\n" +
                "   good point; a low 'hmm' = the point doesn't fit the others (Backspace to undo).\n" +
                "   Planes: drag the yellow handles to resize, or extend them to another plane.\n" +
                "   Measure: D distance · A angle · R diameter · X coordinates — click entities\n" +
                "   (features or single probed points) in the 3D view or the feature list.\n" +
                "4. Alignment tab: square the part up (3-2-1 from features or principal axes),\n" +
                "   then put the origin at a corner, the centre, or a feature — or click a\n" +
                "   bounding-box handle in the 3D view.\n" +
                "5. File > Export STEP. In SolidWorks: Open the .step, enable\n" +
                "   'Import free point/curve entities' to get axes, centres and curves.\n\n" +
                "VIEW\n  Left-drag orbit · Right/middle-drag (or Shift+left) pan · Wheel zoom\n  Double-click = orbit around that point · F = fit work area (Shift+F = features)\n" +
                "  Workspace tab: sketch the work area on the table + max height; Fit frames it.\n" +
                "  3Dconnexion SpaceMouse: push/pull/twist the cap; right button = Fit, left = Iso.\n" +
                "  mm / in toggle in the toolbar.\n\n" +
                "BALL TIPS\n  Select a tip with its ball radius; surfaces are compensated\n  automatically (inside/outside detected from the stylus direction).",
                "Help", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }
}
