using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Windows.Interop;

namespace MicroScribe.App
{
    /// <summary>
    /// 3Dconnexion SpaceMouse / SpaceNavigator input via Windows Raw Input (HID usage page 1, usage 8
    /// "multi-axis controller"). Works alongside 3DxWare; no SDK required.
    /// Report 1 = translation (+ rotation on newer devices, 12 bytes), report 2 = rotation, report 3 = buttons.
    /// </summary>
    public sealed class SpaceMouseInput
    {
        [StructLayout(LayoutKind.Sequential)]
        struct RAWINPUTDEVICE { public ushort UsagePage; public ushort Usage; public uint Flags; public IntPtr Target; }

        [StructLayout(LayoutKind.Sequential)]
        struct RAWINPUTHEADER { public uint Type; public uint Size; public IntPtr Device; public IntPtr WParam; }

        [StructLayout(LayoutKind.Sequential)]
        struct RID_DEVICE_INFO_HID { public uint Size; public uint Type; public uint VendorId; public uint ProductId; public uint Version; public ushort UsagePage; public ushort Usage; }

        [DllImport("user32.dll", SetLastError = true)] static extern bool RegisterRawInputDevices(RAWINPUTDEVICE[] devices, uint count, uint size);
        [DllImport("user32.dll")] static extern uint GetRawInputData(IntPtr hRawInput, uint command, IntPtr data, ref uint size, uint headerSize);
        [DllImport("user32.dll")] static extern uint GetRawInputDeviceInfo(IntPtr hDevice, uint command, IntPtr data, ref uint size);

        const int WM_INPUT = 0x00FF;
        const uint RID_INPUT = 0x10000003, RIM_TYPEHID = 2, RIDI_DEVICEINFO = 0x2000000b;

        /// <summary>Latest raw values: tx, ty, tz, rx, ry, rz (about ±350 at full deflection).</summary>
        public readonly double[] Axes = new double[6];
        public int Buttons { get; private set; }
        public bool Registered { get; private set; }
        public string LastDevice = "";
        public long Reports;
        long _lastReport;

        public event Action<int, bool> ButtonChanged;

        public bool Register(HwndSource src)
        {
            var dev = new[] { new RAWINPUTDEVICE { UsagePage = 0x01, Usage = 0x08, Flags = 0, Target = src.Handle } };
            Registered = RegisterRawInputDevices(dev, 1, (uint)Marshal.SizeOf(typeof(RAWINPUTDEVICE)));
            if (Registered) src.AddHook(Hook);
            return Registered;
        }

        /// <summary>True while the cap is deflected (recent non-zero report).</summary>
        public bool Active
        {
            get
            {
                double age = (Stopwatch.GetTimestamp() - _lastReport) * 1000.0 / Stopwatch.Frequency;
                if (age > 250) return false;
                foreach (double a in Axes) if (Math.Abs(a) > 0) return true;
                return false;
            }
        }

        IntPtr Hook(IntPtr hwnd, int msg, IntPtr wParam, IntPtr lParam, ref bool handled)
        {
            if (msg != WM_INPUT) return IntPtr.Zero;
            try { Parse(lParam); } catch { }
            return IntPtr.Zero;
        }

        void Parse(IntPtr lParam)
        {
            uint hdr = (uint)Marshal.SizeOf(typeof(RAWINPUTHEADER));
            uint size = 0;
            GetRawInputData(lParam, RID_INPUT, IntPtr.Zero, ref size, hdr);
            if (size == 0) return;
            IntPtr buf = Marshal.AllocHGlobal((int)size);
            try
            {
                if (GetRawInputData(lParam, RID_INPUT, buf, ref size, hdr) != size) return;
                var h = (RAWINPUTHEADER)Marshal.PtrToStructure(buf, typeof(RAWINPUTHEADER));
                if (h.Type != RIM_TYPEHID) return;
                int sizeHid = Marshal.ReadInt32(buf, (int)hdr);
                int count = Marshal.ReadInt32(buf, (int)hdr + 4);
                int off = (int)hdr + 8;
                if (LastDevice == "") LastDevice = DeviceName(h.Device);
                for (int r = 0; r < count; r++, off += sizeHid)
                {
                    var data = new byte[sizeHid];
                    Marshal.Copy(IntPtr.Add(buf, off), data, 0, sizeHid);
                    Report(data);
                }
            }
            finally { Marshal.FreeHGlobal(buf); }
        }

        static short S16(byte[] d, int i) { return (short)(d[i] | (d[i + 1] << 8)); }

        void Report(byte[] d)
        {
            if (d.Length < 2) return;
            switch (d[0])
            {
                case 1:
                    if (d.Length >= 7) { Axes[0] = S16(d, 1); Axes[1] = S16(d, 3); Axes[2] = S16(d, 5); }
                    if (d.Length >= 13) { Axes[3] = S16(d, 7); Axes[4] = S16(d, 9); Axes[5] = S16(d, 11); }
                    break;
                case 2:
                    if (d.Length >= 7) { Axes[3] = S16(d, 1); Axes[4] = S16(d, 3); Axes[5] = S16(d, 5); }
                    break;
                case 3:
                    {
                        int b = 0;
                        for (int i = 1; i < Math.Min(d.Length, 5); i++) b |= d[i] << (8 * (i - 1));
                        int changed = b ^ Buttons;
                        Buttons = b;
                        var hb = ButtonChanged;
                        for (int i = 0; i < 32 && changed != 0; i++)
                            if ((changed & (1 << i)) != 0 && hb != null) hb(i, (b & (1 << i)) != 0);
                        break;
                    }
                default: return;
            }
            Reports++;
            _lastReport = Stopwatch.GetTimestamp();
        }

        static string DeviceName(IntPtr dev)
        {
            try
            {
                uint size = 32; // sizeof(RID_DEVICE_INFO)
                IntPtr buf = Marshal.AllocHGlobal(64);
                try
                {
                    Marshal.WriteInt32(buf, (int)size);
                    if ((int)GetRawInputDeviceInfo(dev, RIDI_DEVICEINFO, buf, ref size) <= 0) return "multi-axis controller";
                    uint vid = (uint)Marshal.ReadInt32(buf, 8), pid = (uint)Marshal.ReadInt32(buf, 12);
                    return string.Format("VID {0:X4} PID {1:X4}", vid, pid);
                }
                finally { Marshal.FreeHGlobal(buf); }
            }
            catch { return "multi-axis controller"; }
        }
    }
}
