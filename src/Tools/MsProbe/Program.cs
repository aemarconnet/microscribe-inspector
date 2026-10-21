using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using MicroScribe.Core;
using MicroScribe.Driver;

namespace MicroScribe.Tools
{
    /// <summary>
    /// Hardware probe / validation tool. Only read-only HCI commands are used.
    ///   msprobe info   [COMx] [baud]     identify arm, dump calibration, 20 samples
    ///   msprobe scan   [COMx]            try every baud rate
    ///   msprobe stream [COMx] [seconds]  live tip position + pedal events
    /// </summary>
    static class Program
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        static TextWriter _log;

        static int Main(string[] args)
        {
            Thread.CurrentThread.CurrentCulture = CI;
            string cmd = args.Length > 0 ? args[0].ToLowerInvariant() : "info";
            string port = args.Length > 1 ? args[1] : "COM4";
            string logDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "probe-logs");
            Directory.CreateDirectory(logDir);
            string logPath = Path.Combine(logDir, string.Format("{0}-{1:yyyyMMdd-HHmmss}.txt", cmd, DateTime.Now));
            using (_log = new StreamWriter(logPath))
            {
                try
                {
                    switch (cmd)
                    {
                        case "info": return Info(port, args.Length > 2 ? int.Parse(args[2]) : 0);
                        case "scan": return Scan(port);
                        case "stream": return Stream(port, args.Length > 2 ? double.Parse(args[2], CI) : 15);
                        default: W("unknown command " + cmd); return 2;
                    }
                }
                catch (Exception ex)
                {
                    W("ERROR: " + ex.GetType().Name + ": " + ex.Message);
                    return 1;
                }
                finally
                {
                    W("log: " + logPath);
                }
            }
        }

        static void W(string s) { Console.WriteLine(s); if (_log != null) { _log.WriteLine(s); _log.Flush(); } }

        static string F(Vec3 v) { return string.Format(CI, "{0,9:0.000} {1,9:0.000} {2,9:0.000}", v.X, v.Y, v.Z); }

        static int Info(string port, int baud)
        {
            var arm = new MicroScribeArm(port, baud);
            arm.Log = s => W("  [arm] " + s);
            arm.Connect();
            try
            {
                var info = arm.Info;
                W("");
                W(info.ToString());
                W("Active baud  : " + arm.ActiveBaud);
                W("Encoders     : " + info.NumEncoders + "   buttons supported mask: 0x" + info.ButtonsSupported.ToString("X2"));
                W("GET_MAXES    : " + HciSerial.Hex(info.Maxes, info.Maxes.Length));
                W("GET_PARAMS   : " + HciSerial.Hex(info.ParamBlock, info.ParamBlock.Length));
                if (info.ExtParamBlock != null) W("GET_EXT_PRM  : " + HciSerial.Hex(info.ExtParamBlock, info.ExtParamBlock.Length));
                if (info.HomeRef != null) W("GET_HOME_REF : " + HciSerial.Hex(info.HomeRef, info.HomeRef.Length));
                W("");
                W("Decoded DH model (mm / deg):");
                W(arm.Model.Describe());
                W("Samples (tip mm, buttons, encoders):");
                for (int i = 0; i < 20; i++)
                {
                    Thread.Sleep(100);
                    var s = arm.Latest;
                    if (s == null) continue;
                    W(string.Format(CI, "  {0}   btn={1}   enc=[{2}]", F(s.Tip), s.Buttons, string.Join(",", s.Encoders)));
                }
                W(string.Format(CI, "Sample rate: {0:0.0} Hz", arm.SampleRate));
            }
            finally { arm.Disconnect(); }
            return 0;
        }

        static int Scan(string port)
        {
            foreach (int baud in new[] { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 })
            {
                var hci = new HciSerial();
                try
                {
                    hci.Open(port, baud);
                    var sw = Stopwatch.StartNew();
                    string id = hci.SignOn(1500);
                    W(string.Format("{0,7} baud: OK  ({1} ms)  id='{2}'", baud, sw.ElapsedMilliseconds, id));
                    // measure the packet rate briefly
                    sw.Restart(); int n = 0;
                    while (sw.ElapsedMilliseconds < 1000) { hci.Query(HciSerial.StdCmd(5, false)); n++; }
                    W(string.Format("               {0} packets/s", n));
                }
                catch (Exception ex) { W(string.Format("{0,7} baud: {1}", baud, ex.Message)); }
                finally { hci.EndSession(); hci.Close(); }
                Thread.Sleep(200);
            }
            return 0;
        }

        static int Stream(string port, double seconds)
        {
            var arm = new MicroScribeArm(port);
            arm.Log = s => W("  [arm] " + s);
            arm.ButtonChanged += (b, down, s) => W(string.Format(CI, ">>> button {0} {1} at {2}", b, down ? "DOWN" : "up", F(s.Tip)));
            arm.Connect();
            W(arm.Description);
            var sw = Stopwatch.StartNew();
            try
            {
                while (sw.Elapsed.TotalSeconds < seconds)
                {
                    Thread.Sleep(250);
                    var s = arm.Latest;
                    if (s != null)
                        W(string.Format(CI, "{0,6:0.0}s  tip {1}  dir {2}  btn={3}  {4:0}Hz", sw.Elapsed.TotalSeconds, F(s.Tip), F(s.Direction), s.Buttons, arm.SampleRate));
                }
            }
            finally { arm.Disconnect(); }
            return 0;
        }
    }
}
