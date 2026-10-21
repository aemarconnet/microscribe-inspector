using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using MicroScribe.Geometry;

namespace MicroScribe.App
{
    /// <summary>Small helpers to build WPF UI in code (no XAML compiler needed).</summary>
    public static class Ui
    {
        public static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        public static readonly Brush Accent = Frozen(Color.FromRgb(37, 99, 235));
        public static readonly Brush PanelBg = Frozen(Color.FromRgb(246, 247, 249));
        public static readonly Brush Stroke = Frozen(Color.FromRgb(214, 218, 225));
        public static readonly Brush Muted = Frozen(Color.FromRgb(100, 108, 120));
        public static readonly FontFamily Mono = new FontFamily("Consolas");

        public static Brush Frozen(Color c) { var b = new SolidColorBrush(c); b.Freeze(); return b; }

        public static Color KindColor(FeatureKind k)
        {
            switch (k)
            {
                case FeatureKind.Point: return Color.FromRgb(239, 68, 68);
                case FeatureKind.Line: return Color.FromRgb(34, 197, 94);
                case FeatureKind.Plane: return Color.FromRgb(59, 130, 246);
                case FeatureKind.Circle: return Color.FromRgb(245, 158, 11);
                case FeatureKind.Sphere: return Color.FromRgb(20, 184, 166);
                case FeatureKind.Cylinder: return Color.FromRgb(249, 115, 22);
                case FeatureKind.Cone: return Color.FromRgb(168, 85, 247);
                case FeatureKind.Curve: return Color.FromRgb(236, 72, 153);
                case FeatureKind.Surface: return Color.FromRgb(132, 204, 22);
                case FeatureKind.Cloud: return Color.FromRgb(20, 184, 166);
            }
            return Colors.Gray;
        }

        static string IconPath(FeatureKind? k)
        {
            if (!k.HasValue) return "M10,1 L12,8 L19,10 L12,12 L10,19 L8,12 L1,10 L8,8 Z"; // auto
            switch (k.Value)
            {
                case FeatureKind.Point: return "M10,6 A4,4 0 1 1 9.99,6 Z M2,10 L6,10 M14,10 L18,10 M10,2 L10,6 M10,14 L10,18";
                case FeatureKind.Line: return "M3,16 L17,4 M3,16 A1.5,1.5 0 1 1 2.99,16 M17,4 A1.5,1.5 0 1 1 16.99,4";
                case FeatureKind.Plane: return "M2,15 L7,5 L18,5 L13,15 Z";
                case FeatureKind.Circle: return "M10,3 A7,7 0 1 1 9.99,3 Z";
                case FeatureKind.Sphere: return "M10,2 A8,8 0 1 1 9.99,2 Z M2,10 A8,3 0 0 0 18,10";
                case FeatureKind.Cylinder: return "M4,5 A6,2 0 1 0 16,5 A6,2 0 1 0 4,5 M4,5 L4,15 A6,2 0 0 0 16,15 L16,5";
                case FeatureKind.Cone: return "M10,2 L3,15 A7,2.5 0 0 0 17,15 Z";
                case FeatureKind.Curve: return "M2,15 C6,1 10,19 18,5";
                case FeatureKind.Surface: return "M2,8 C6,3 10,11 18,5 L18,14 C10,20 6,12 2,17 Z M2,12.5 C6,7.5 10,15.5 18,9.5 M7,6 L7,15 M12.5,7.5 L12.5,16.5";
                case FeatureKind.Cloud: return "M3,12 L3.5,12 M6,8 L6.5,8 M8,14 L8.5,14 M10,5 L10.5,5 M12,11 L12.5,11 M14,16 L14.5,16 M15,7 L15.5,7 M17,12 L17.5,12 M5,16 L5.5,16 M11,17 L11.5,17";
            }
            return "M2,2 L18,18";
        }

        public static FrameworkElement Icon(FeatureKind? k, double size = 18, Brush stroke = null)
        {
            var p = new Path
            {
                Data = System.Windows.Media.Geometry.Parse(IconPath(k)),
                Stroke = stroke ?? (k.HasValue ? Frozen(KindColor(k.Value)) : Accent),
                StrokeThickness = 1.6,
                StrokeLineJoin = PenLineJoin.Round,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Stretch = Stretch.Uniform,
                Width = size, Height = size
            };
            if (k == FeatureKind.Plane) p.Fill = Frozen(Color.FromArgb(50, 59, 130, 246));
            if (k == FeatureKind.Surface) p.Fill = Frozen(Color.FromArgb(50, 132, 204, 22));
            return p;
        }

        public static Button Btn(string text, Action click, string tip = null, FrameworkElement icon = null)
        {
            var b = new Button { Padding = new Thickness(8, 3, 8, 3), Margin = new Thickness(2), ToolTip = tip, Focusable = false };
            if (icon != null)
            {
                var sp = new StackPanel { Orientation = Orientation.Horizontal };
                icon.Margin = new Thickness(0, 0, 5, 0);
                sp.Children.Add(icon);
                sp.Children.Add(new TextBlock { Text = text, VerticalAlignment = VerticalAlignment.Center });
                b.Content = sp;
            }
            else b.Content = text;
            b.Click += (s, e) => click();
            return b;
        }

        public static TextBlock Text(string t, double size = 12, FontWeight? weight = null, Brush fg = null)
        {
            return new TextBlock { Text = t, FontSize = size, FontWeight = weight ?? FontWeights.Normal, Foreground = fg ?? Brushes.Black, TextWrapping = TextWrapping.Wrap };
        }

        public static TextBlock Header(string t)
        {
            return new TextBlock { Text = t, FontSize = 12, FontWeight = FontWeights.SemiBold, Foreground = Muted, Margin = new Thickness(0, 10, 0, 4) };
        }

        /// <summary>Horizontal row that wraps onto the next line when the panel is narrow.</summary>
        public static WrapPanel Row(params UIElement[] items)
        {
            var sp = new WrapPanel { Orientation = Orientation.Horizontal, Margin = new Thickness(0, 2, 0, 2) };
            foreach (var i in items) sp.Children.Add(i);
            return sp;
        }

        public static TextBox NumBox(double value, Action<double> changed, double width = 70)
        {
            var tb = new TextBox { Text = value.ToString("0.###", CI), Width = width, Margin = new Thickness(2), VerticalContentAlignment = VerticalAlignment.Center };
            Action commit = () =>
            {
                double v;
                if (double.TryParse(tb.Text.Replace(',', '.'), NumberStyles.Float, CI, out v)) changed(v);
            };
            tb.LostFocus += (s, e) => commit();
            tb.KeyDown += (s, e) => { if (e.Key == System.Windows.Input.Key.Enter) commit(); };
            return tb;
        }

        public static Border Card(UIElement child, Thickness? margin = null)
        {
            return new Border
            {
                Background = Brushes.White, BorderBrush = Stroke, BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(6), Padding = new Thickness(10), Margin = margin ?? new Thickness(0, 0, 0, 8), Child = child
            };
        }

        // ------------------------------------------------------------------ display units
        // Everything is stored in millimetres; only formatting/parsing goes through these.

        /// <summary>True = inches, false = millimetres. Change via MainWindow.SetUnits.</summary>
        public static bool Inch;
        public static event Action UnitsChanged;
        public static void SetInch(bool inch) { Inch = inch; var h = UnitsChanged; if (h != null) h(); }

        public static string U { get { return Inch ? "in" : "mm"; } }
        public static double ToDisp(double mm) { return Inch ? mm / 25.4 : mm; }
        public static double FromDisp(double v) { return Inch ? v * 25.4 : v; }
        /// <summary>Length in display units: mm with 3 decimals, inch with 4.</summary>
        public static string L(double mm) { return Inch ? (mm / 25.4).ToString("0.0000", CI) : mm.ToString("0.000", CI); }
        public static string LU(double mm) { return L(mm) + " " + U; }

        public static string Mm(double v, bool inch) { return L(v); }
        public static string Unit(bool inch) { return U; }

        /// <summary>Numeric box for a length: shows display units, reports millimetres.</summary>
        public static TextBox LenBox(double mm, Action<double> setMm, double width = 70)
        {
            var tb = NumBox(ToDisp(mm), v => setMm(FromDisp(v)), width);
            tb.Text = Inch ? (mm / 25.4).ToString("0.####", CI) : mm.ToString("0.###", CI);
            tb.ToolTip = "Value in " + (Inch ? "inches" : "millimetres");
            return tb;
        }

        public static TextBlock UnitLabel() { return new TextBlock { Text = U, Foreground = Muted, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(3, 0, 0, 0) }; }
    }
}
