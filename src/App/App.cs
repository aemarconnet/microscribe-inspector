using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Windows;
using System.Windows.Threading;

[assembly: AssemblyTitle("MicroScribe Inspector")]
[assembly: AssemblyDescription("MicroScribe digitizer driver, surface fitting and STEP export")]
[assembly: AssemblyProduct("MicroScribe Inspector")]
[assembly: AssemblyCompany("Marconnet Technologies")]
[assembly: AssemblyCopyright("Copyright © 2026 Marconnet Technologies")]
[assembly: AssemblyVersion("1.1.1.0")]
[assembly: AssemblyFileVersion("1.1.1.0")]

namespace MicroScribe.App
{
    public static class Program
    {
        [STAThread]
        public static int Main(string[] args)
        {
            Thread.CurrentThread.CurrentCulture = System.Globalization.CultureInfo.InvariantCulture;
            string a0 = args.Length > 0 ? args[0].ToLowerInvariant() : "";
            if (a0 == "/install" || a0 == "--install")
            {
                string log = Installer.Install(args.Any(x => x.ToLowerInvariant().Contains("desktop")), true);
                if (!args.Any(x => x.ToLowerInvariant() == "/quiet")) MessageBox.Show(log, Installer.AppName + " installed", MessageBoxButton.OK, MessageBoxImage.Information);
                return 0;
            }
            if (a0 == "/uninstall" || a0 == "--uninstall")
            {
                if (args.Any(x => x.ToLowerInvariant() == "/quiet") ||
                    MessageBox.Show("Uninstall " + Installer.AppName + "?", Installer.AppName, MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
                {
                    string log = Installer.Uninstall();
                    if (!args.Any(x => x.ToLowerInvariant() == "/quiet")) MessageBox.Show(log, Installer.AppName, MessageBoxButton.OK, MessageBoxImage.Information);
                }
                return 0;
            }

            var app = new Application { ShutdownMode = ShutdownMode.OnMainWindowClose };
            app.DispatcherUnhandledException += OnUiException;
            AppDomain.CurrentDomain.UnhandledException += (s, e) => LogCrash(e.ExceptionObject as Exception);
            var settings = AppSettings.Load();
            int st = Array.FindIndex(args, x => x.ToLowerInvariant() == "/selftest");
            string open = st < 0 ? args.FirstOrDefault(x => !x.StartsWith("/") && File.Exists(x)) : null;
            var w = new MainWindow(settings, open);
            if (st >= 0)
            {
                string dir = st + 1 < args.Length ? args[st + 1] : Path.Combine(Path.GetTempPath(), "msinspector-selftest");
                w.WindowStartupLocation = WindowStartupLocation.Manual;
                w.Left = -30000; w.Top = 0; w.Width = 1600; w.Height = 950;
                w.WindowState = WindowState.Normal;
                w.ShowInTaskbar = false; w.ShowActivated = false;
                w.Loaded += (s, e) => w.RunSelfTest(dir);
            }
            return app.Run(w);
        }

        static void OnUiException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            LogCrash(e.Exception);
            MessageBox.Show("Unexpected error: " + e.Exception.Message + "\n\nDetails were written to " + CrashLog, Installer.AppName, MessageBoxButton.OK, MessageBoxImage.Error);
            e.Handled = true;
        }

        static string CrashLog
        {
            get { return Path.Combine(Path.GetTempPath(), "MicroScribeInspector-error.log"); }
        }

        static void LogCrash(Exception ex)
        {
            try { File.AppendAllText(CrashLog, DateTime.Now + "\r\n" + ex + "\r\n\r\n"); } catch { }
        }
    }
}
