using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Serialization;
using MicroScribe.Core;

namespace MicroScribe.App
{
    /// <summary>A stylus/tip definition: offset in the stylus frame plus ball radius.</summary>
    public sealed class TipProfile
    {
        public string Name = "Standard point tip";
        public double OffsetX, OffsetY, OffsetZ;   // mm, stylus frame
        public double BallRadius;                  // mm (0 = point tip)
        /// <summary>Last qualification on a reference sphere (DateTime.MinValue = never).</summary>
        public DateTime QualifiedOn = DateTime.MinValue;
        public double QualifiedReference, QualifiedRms, QualifiedFormError;
        public int QualifiedPoints;

        [XmlIgnore] public Vec3 Offset { get { return new Vec3(OffsetX, OffsetY, OffsetZ); } set { OffsetX = value.X; OffsetY = value.Y; OffsetZ = value.Z; } }
        [XmlIgnore] public bool IsQualified { get { return QualifiedOn > DateTime.MinValue; } }
        public override string ToString() { return Name; }
    }

    /// <summary>One saved arm check (see ArmCheck): kind "single-point" or "length", values in mm.</summary>
    public sealed class ArmCheckRecord
    {
        public DateTime Date;
        public string Kind = "single-point";
        public string Arm = "";
        public string Tip = "";
        public int N;
        public double MaxDeviation, Range, TwoSigma, MeanError, Reference, Limit;
        public bool Pass;
    }

    public enum PedalAction { Capture, Finish, Undo, None }

    /// <summary>
    /// The usable measuring volume: a polygon sketched on the table plus a maximum height.
    /// Stored in machine coordinates (the arm is fixed to the table), so it survives across projects.
    /// "Fit" frames this volume.
    /// </summary>
    public sealed class WorkArea
    {
        public bool Enabled;
        /// <summary>Table plane (machine frame). Default: the arm's base plane z = 0.</summary>
        public double TOX, TOY, TOZ, TNX, TNY, TNZ = 1;
        public List<double[]> Outline = new List<double[]>();
        public double Height = 300;

        [XmlIgnore] public Vec3 TableOrigin { get { return new Vec3(TOX, TOY, TOZ); } set { TOX = value.X; TOY = value.Y; TOZ = value.Z; } }
        [XmlIgnore] public Vec3 TableNormal { get { var n = new Vec3(TNX, TNY, TNZ); return n.Length > 1e-9 ? n.Normalized() : Vec3.UnitZ; } set { TNX = value.X; TNY = value.Y; TNZ = value.Z; } }
        [XmlIgnore] public List<Vec3> OutlinePoints { get { return Outline.Select(a => new Vec3(a[0], a[1], a[2])).ToList(); } }
        [XmlIgnore] public bool IsDefined { get { return Outline.Count >= 3 && Height > 0; } }

        /// <summary>All 2N prism corners (machine frame).</summary>
        public List<Vec3> Corners()
        {
            var r = new List<Vec3>();
            Vec3 n = TableNormal;
            foreach (var p in OutlinePoints) { r.Add(p); r.Add(p + n * Height); }
            return r;
        }
    }

    public sealed class SpaceMouseSettings
    {
        public bool Enabled = true;
        public double Sensitivity = 1.0;
        public double Deadzone = 12;
        /// <summary>Per-motion direction reversal, relative to the default mapping.</summary>
        public bool ReversePanX, ReversePanY, ReverseZoom, ReverseTilt, ReverseSpin;
    }

    /// <summary>User settings, stored next to the exe (portable) or in %APPDATA% (installed).</summary>
    public sealed class AppSettings
    {
        public string Port = "COM4";
        public int Baud = 0;                 // 0 = auto
        public string Driver = "native";     // native | simulator
        public bool InchDisplay;
        public List<TipProfile> Tips = new List<TipProfile>();
        public string ActiveTip = "Standard point tip";
        public PedalAction Pedal1 = PedalAction.Capture;
        public PedalAction Pedal2 = PedalAction.Finish;
        /// <summary>Average this many milliseconds of samples after a capture press (arm held still).</summary>
        public int CaptureAverageMs = 60;
        /// <summary>Reject a capture when the stylus moved more than this during the averaging window (mm, 0 = off).</summary>
        public double CaptureMotionLimit = 0.25;
        /// <summary>Hands-free capture: stylus held still this long captures a point (ms, 0 = off).</summary>
        public int DwellMs = 0;
        public double DwellRadius = 0.2;      // mm the tip may wander while "still"
        public double DwellRearm = 2.0;       // mm the tip must move away before the next hands-free capture
        /// <summary>Arm checks: pass/fail limit (mm) and the saved history (newest last).</summary>
        public double ArmCheckLimit = 0.30;
        public double ArmCheckReference = 100;
        public int ArmCheckIntervalDays = 30;
        public List<ArmCheckRecord> ArmChecks = new List<ArmCheckRecord>();
        public double QualifyReference = 25.4;
        /// <summary>Nominal model display and deviation colours (green within ±DeviationRange).</summary>
        public bool ShowModel = true;
        public bool ColorByDeviation;
        public double DeviationRange = 0.25;
        public double CurveSpacing = 1.0;    // mm between points while scanning a curve
        public double DefaultExportMargin = 2.0;
        public bool ShowArm = true;
        public bool ShowLabels = true;
        public bool BeepOnCapture = true;
        public List<string> RecentFiles = new List<string>();
        public string LastExportFolder = "";
        public string Author = Environment.UserName;
        public double WindowWidth = 1500, WindowHeight = 920;
        public bool WindowMaximized;
        public WorkArea WorkArea = new WorkArea();
        public SpaceMouseSettings SpaceMouse = new SpaceMouseSettings();
        /// <summary>Points that deviate more than this from the fit (and > 3σ) trigger the warning sound.</summary>
        public double GoodPointTolerance = 0.15;
        public double SketchSnap = 5;

        [XmlIgnore] public static string SettingsPath { get; private set; }
        [XmlIgnore] public static bool IsPortable { get; private set; }

        public TipProfile CurrentTip
        {
            get
            {
                var t = Tips.FirstOrDefault(x => x.Name == ActiveTip);
                if (t == null) { if (Tips.Count == 0) Tips.Add(new TipProfile()); t = Tips[0]; ActiveTip = t.Name; }
                return t;
            }
        }

        /// <summary>Latest saved check of each kind (null when none).</summary>
        public ArmCheckRecord LatestCheck(string kind)
        {
            return ArmChecks.Where(c => c.Kind == kind).OrderBy(c => c.Date).LastOrDefault();
        }

        /// <summary>
        /// The arm's demonstrated measuring error (mm) from the latest checks: the larger of the
        /// single-point maximum deviation and the length-test maximum error. NaN when never checked.
        /// </summary>
        [XmlIgnore]
        public double MeasuringError
        {
            get
            {
                var a = LatestCheck("single-point"); var b = LatestCheck("length");
                if (a == null && b == null) return double.NaN;
                return Math.Max(a != null ? a.MaxDeviation : 0, b != null ? b.MaxDeviation : 0);
            }
        }

        public static string ExeDir { get { return AppDomain.CurrentDomain.BaseDirectory.TrimEnd('\\'); } }

        static bool IsWritable(string dir)
        {
            try
            {
                string probe = Path.Combine(dir, ".write-test-" + Guid.NewGuid().ToString("N"));
                File.WriteAllText(probe, "x");
                File.Delete(probe);
                return true;
            }
            catch { return false; }
        }

        public static AppSettings Load()
        {
            string local = Path.Combine(ExeDir, "MicroScribeInspector.settings.xml");
            string appData = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "MicroScribe Inspector", "settings.xml");
            bool installed = Installer.IsInstalledCopy();
            if (File.Exists(Path.Combine(ExeDir, "portable.flag")) || (!installed && (File.Exists(local) || IsWritable(ExeDir))))
            {
                IsPortable = true; SettingsPath = local;
            }
            else { IsPortable = false; SettingsPath = appData; }

            AppSettings s = null;
            try
            {
                if (File.Exists(SettingsPath))
                    using (var f = File.OpenRead(SettingsPath)) s = (AppSettings)new XmlSerializer(typeof(AppSettings)).Deserialize(f);
            }
            catch { s = null; }
            if (s == null) s = new AppSettings();
            if (s.Tips.Count == 0)
            {
                s.Tips.Add(new TipProfile { Name = "Standard point tip" });
                s.Tips.Add(new TipProfile { Name = "Ball tip Ø3 mm (calibrate!)", BallRadius = 1.5 });
            }
            return s;
        }

        public void Save()
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(SettingsPath));
                using (var f = File.Create(SettingsPath)) new XmlSerializer(typeof(AppSettings)).Serialize(f, this);
            }
            catch { /* settings are best-effort */ }
        }

        public void AddRecent(string path)
        {
            RecentFiles.RemoveAll(p => string.Equals(p, path, StringComparison.OrdinalIgnoreCase));
            RecentFiles.Insert(0, path);
            while (RecentFiles.Count > 10) RecentFiles.RemoveAt(RecentFiles.Count - 1);
        }
    }
}
