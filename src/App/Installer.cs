using System;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32;

namespace MicroScribe.App
{
    /// <summary>
    /// Optional per-user installation (no admin rights): copies the exe to
    /// %LOCALAPPDATA%\Programs\MicroScribe Inspector, adds a Start-menu shortcut, registers the
    /// .msproj file type and an "Apps &amp; features" uninstall entry. The exe also runs portably.
    /// </summary>
    public static class Installer
    {
        public const string AppName = "MicroScribe Inspector";
        const string AppKey = "MicroScribeInspector";
        const string ProgId = "MicroScribeInspector.Project";
        const string UninstallKey = @"Software\Microsoft\Windows\CurrentVersion\Uninstall\" + AppKey;

        public static string InstallDir
        {
            get { return Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Programs", AppName); }
        }

        public static string InstalledExe { get { return Path.Combine(InstallDir, "MicroScribeInspector.exe"); } }

        static string StartMenuLink
        {
            get { return Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Programs), AppName + ".lnk"); }
        }

        static string DesktopLink
        {
            get { return Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory), AppName + ".lnk"); }
        }

        public static string CurrentExe { get { return Assembly.GetExecutingAssembly().Location; } }

        public static bool IsInstalledCopy()
        {
            return string.Equals(Path.GetFullPath(CurrentExe), Path.GetFullPath(InstalledExe), StringComparison.OrdinalIgnoreCase);
        }

        public static bool IsInstalled()
        {
            using (var k = Registry.CurrentUser.OpenSubKey(UninstallKey)) return k != null && File.Exists(InstalledExe);
        }

        public static string Version
        {
            get { return Assembly.GetExecutingAssembly().GetName().Version.ToString(3); }
        }

        /// <summary>Installs (or updates) the current exe. Returns a log of what was done.</summary>
        public static string Install(bool desktopShortcut, bool associateFiles)
        {
            var log = new StringBuilder();
            Directory.CreateDirectory(InstallDir);
            if (!IsInstalledCopy())
            {
                File.Copy(CurrentExe, InstalledExe, true);
                log.AppendLine("Copied program to " + InstallDir);
                // bring documentation along when present
                string docs = Path.Combine(Path.GetDirectoryName(CurrentExe), "docs");
                if (Directory.Exists(docs))
                {
                    string dst = Path.Combine(InstallDir, "docs");
                    Directory.CreateDirectory(dst);
                    foreach (var f in Directory.GetFiles(docs)) File.Copy(f, Path.Combine(dst, Path.GetFileName(f)), true);
                }
            }
            CreateShortcut(StartMenuLink, InstalledExe, "Measure surfaces with a MicroScribe and export them to CAD");
            log.AppendLine("Start menu shortcut: " + StartMenuLink);
            if (desktopShortcut)
            {
                CreateShortcut(DesktopLink, InstalledExe, AppName);
                log.AppendLine("Desktop shortcut: " + DesktopLink);
            }
            if (associateFiles)
            {
                using (var k = Registry.CurrentUser.CreateSubKey(@"Software\Classes\.msproj")) k.SetValue("", ProgId);
                using (var k = Registry.CurrentUser.CreateSubKey(@"Software\Classes\" + ProgId))
                {
                    k.SetValue("", "MicroScribe Inspector project");
                    using (var i = k.CreateSubKey("DefaultIcon")) i.SetValue("", "\"" + InstalledExe + "\",0");
                    using (var c = k.CreateSubKey(@"shell\open\command")) c.SetValue("", "\"" + InstalledExe + "\" \"%1\"");
                }
                SHChangeNotify(0x08000000, 0, IntPtr.Zero, IntPtr.Zero); // SHCNE_ASSOCCHANGED
                log.AppendLine("Registered .msproj files");
            }
            using (var k = Registry.CurrentUser.CreateSubKey(UninstallKey))
            {
                k.SetValue("DisplayName", AppName);
                k.SetValue("DisplayVersion", Version);
                k.SetValue("Publisher", "Marconnet Technologies");
                k.SetValue("InstallLocation", InstallDir);
                k.SetValue("DisplayIcon", InstalledExe + ",0");
                k.SetValue("UninstallString", "\"" + InstalledExe + "\" /uninstall");
                k.SetValue("NoModify", 1, RegistryValueKind.DWord);
                k.SetValue("NoRepair", 1, RegistryValueKind.DWord);
                k.SetValue("EstimatedSize", (int)(new FileInfo(InstalledExe).Length / 1024), RegistryValueKind.DWord);
                k.SetValue("InstallDate", DateTime.Now.ToString("yyyyMMdd"));
            }
            log.AppendLine("Added to Apps & features (uninstall from there or with /uninstall)");
            return log.ToString();
        }

        /// <summary>Removes shortcuts, registry entries and (after exit) the install folder.</summary>
        public static string Uninstall()
        {
            var log = new StringBuilder();
            foreach (var l in new[] { StartMenuLink, DesktopLink })
                if (File.Exists(l)) { File.Delete(l); log.AppendLine("Removed " + l); }
            Registry.CurrentUser.DeleteSubKeyTree(@"Software\Classes\" + ProgId, false);
            using (var k = Registry.CurrentUser.OpenSubKey(@"Software\Classes\.msproj", true))
                if (k != null && (k.GetValue("") as string) == ProgId) { k.Close(); Registry.CurrentUser.DeleteSubKeyTree(@"Software\Classes\.msproj", false); }
            Registry.CurrentUser.DeleteSubKeyTree(UninstallKey, false);
            SHChangeNotify(0x08000000, 0, IntPtr.Zero, IntPtr.Zero);
            log.AppendLine("Removed registry entries");
            if (Directory.Exists(InstallDir))
            {
                // The running exe cannot delete itself: let cmd remove the folder after we exit.
                var psi = new ProcessStartInfo("cmd.exe", "/c timeout /t 2 /nobreak >nul & rmdir /s /q \"" + InstallDir + "\"")
                {
                    CreateNoWindow = true, UseShellExecute = false, WorkingDirectory = Path.GetTempPath()
                };
                Process.Start(psi);
                log.AppendLine("Program folder will be removed: " + InstallDir);
            }
            log.AppendLine("Settings in %APPDATA%\\" + AppName + " were kept.");
            return log.ToString();
        }

        // ------------------------------------------------------------------ shell link COM interop

        [DllImport("shell32.dll")] static extern void SHChangeNotify(int wEventId, int uFlags, IntPtr dwItem1, IntPtr dwItem2);

        [ComImport, Guid("00021401-0000-0000-C000-000000000046")] class ShellLink { }

        [ComImport, InterfaceType(ComInterfaceType.InterfaceIsIUnknown), Guid("000214F9-0000-0000-C000-000000000046")]
        interface IShellLinkW
        {
            void GetPath([Out, MarshalAs(UnmanagedType.LPWStr)] StringBuilder pszFile, int cch, IntPtr pfd, int fFlags);
            void GetIDList(out IntPtr ppidl);
            void SetIDList(IntPtr pidl);
            void GetDescription([Out, MarshalAs(UnmanagedType.LPWStr)] StringBuilder pszName, int cch);
            void SetDescription([MarshalAs(UnmanagedType.LPWStr)] string pszName);
            void GetWorkingDirectory([Out, MarshalAs(UnmanagedType.LPWStr)] StringBuilder pszDir, int cch);
            void SetWorkingDirectory([MarshalAs(UnmanagedType.LPWStr)] string pszDir);
            void GetArguments([Out, MarshalAs(UnmanagedType.LPWStr)] StringBuilder pszArgs, int cch);
            void SetArguments([MarshalAs(UnmanagedType.LPWStr)] string pszArgs);
            void GetHotkey(out short pwHotkey);
            void SetHotkey(short wHotkey);
            void GetShowCmd(out int piShowCmd);
            void SetShowCmd(int iShowCmd);
            void GetIconLocation([Out, MarshalAs(UnmanagedType.LPWStr)] StringBuilder pszIconPath, int cch, out int piIcon);
            void SetIconLocation([MarshalAs(UnmanagedType.LPWStr)] string pszIconPath, int iIcon);
            void SetRelativePath([MarshalAs(UnmanagedType.LPWStr)] string pszPathRel, int dwReserved);
            void Resolve(IntPtr hwnd, int fFlags);
            void SetPath([MarshalAs(UnmanagedType.LPWStr)] string pszFile);
        }

        [ComImport, InterfaceType(ComInterfaceType.InterfaceIsIUnknown), Guid("0000010b-0000-0000-C000-000000000046")]
        interface IPersistFile
        {
            void GetClassID(out Guid pClassID);
            [PreserveSig] int IsDirty();
            void Load([MarshalAs(UnmanagedType.LPWStr)] string pszFileName, int dwMode);
            void Save([MarshalAs(UnmanagedType.LPWStr)] string pszFileName, [MarshalAs(UnmanagedType.Bool)] bool fRemember);
            void SaveCompleted([MarshalAs(UnmanagedType.LPWStr)] string pszFileName);
            void GetCurFile([MarshalAs(UnmanagedType.LPWStr)] out string ppszFileName);
        }

        static void CreateShortcut(string linkPath, string target, string description)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(linkPath));
            var link = (IShellLinkW)new ShellLink();
            link.SetPath(target);
            link.SetWorkingDirectory(Path.GetDirectoryName(target));
            link.SetDescription(description);
            link.SetIconLocation(target, 0);
            ((IPersistFile)link).Save(linkPath, true);
            Marshal.FinalReleaseComObject(link);
        }
    }
}
