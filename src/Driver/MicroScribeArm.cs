using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using MicroScribe.Core;

namespace MicroScribe.Driver
{
    /// <summary>One pose of the stylus.</summary>
    public sealed class ArmSample
    {
        /// <summary>Probe tip centre, base frame, mm (includes the active tip offset).</summary>
        public Vec3 Tip;
        /// <summary>Unit vector along the stylus, pointing from the wrist toward the tip.</summary>
        public Vec3 Direction;
        /// <summary>Stylus frame (before tip offset).</summary>
        public Mat4 StylusFrame;
        public int Buttons;
        public int[] Encoders;
        public double[] JointAngles;
        public Vec3[] JointOrigins;
        public long Timestamp;
    }

    public enum ArmState { Disconnected, Connecting, Connected, Error }

    /// <summary>Common interface for the real arm and the simulator.</summary>
    public interface IArmDevice : IDisposable
    {
        ArmState State { get; }
        string Description { get; }
        ArmInfo Info { get; }
        /// <summary>Tip offset in the stylus frame (mm).</summary>
        Vec3 TipOffset { get; set; }
        ArmSample Latest { get; }
        double SampleRate { get; }

        event Action<ArmSample> SampleReceived;
        /// <summary>(buttonIndex, pressed, sample at the moment of the change)</summary>
        event Action<int, bool, ArmSample> ButtonChanged;
        event Action<ArmState, string> StateChanged;

        void Connect();
        void Disconnect();
        /// <summary>Re-zero encoders: arm must be resting in its home pose.</summary>
        void Rehome();
    }

    /// <summary>MicroScribe arm over RS-232 using our own HCI implementation (no vendor software needed).</summary>
    public sealed class MicroScribeArm : IArmDevice
    {
        readonly string _portName;
        readonly int _preferredBaud;
        HciSerial _hci;
        ArmModel _model;
        Thread _thread;
        volatile bool _stop;
        readonly Queue<Action<HciSerial>> _commands = new Queue<Action<HciSerial>>();
        readonly object _lock = new object();
        ArmSample _latest;
        Vec3 _tipOffset;
        int _lastButtons;
        double _rate;

        public event Action<ArmSample> SampleReceived;
        public event Action<int, bool, ArmSample> ButtonChanged;
        public event Action<ArmState, string> StateChanged;

        public Action<string> Log;

        public MicroScribeArm(string portName, int preferredBaud = 0)
        {
            _portName = portName;
            _preferredBaud = preferredBaud;
        }

        public ArmState State { get; private set; }
        public ArmInfo Info { get; private set; }
        public ArmModel Model { get { return _model; } }
        public int ActiveBaud { get; private set; }
        public double SampleRate { get { return _rate; } }

        public string Description
        {
            get
            {
                if (Info == null) return "MicroScribe on " + _portName;
                return string.Format("{0} {1} (S/N {2}) on {3} @ {4}", Info.ProductName, Info.ModelName, Info.SerialNumber, _portName, ActiveBaud);
            }
        }

        public Vec3 TipOffset
        {
            get { lock (_lock) return _tipOffset; }
            set { lock (_lock) _tipOffset = value; }
        }

        public ArmSample Latest { get { lock (_lock) return _latest; } }

        void SetState(ArmState s, string msg)
        {
            State = s;
            var h = StateChanged;
            if (h != null) h(s, msg);
        }

        void L(string s) { var h = Log; if (h != null) h(s); }

        /// <summary>Blocking connect (call from a worker thread in UI code).</summary>
        public void Connect()
        {
            Disconnect();
            SetState(ArmState.Connecting, "Connecting to " + _portName + "...");
            var bauds = new List<int>();
            if (_preferredBaud > 0) bauds.Add(_preferredBaud);
            foreach (int b in new[] { 9600, 115200, 38400, 19200 }) if (!bauds.Contains(b)) bauds.Add(b);

            Exception last = null;
            foreach (int baud in bauds)
            {
                var hci = new HciSerial();
                try
                {
                    hci.Open(_portName, baud);
                    string pid = hci.SignOn(1500);
                    L("Signed on at " + baud + " baud: " + pid);
                    Info = ReadInfo(hci, pid);
                    _model = ArmModel.FromInfo(Info);
                    _hci = hci;
                    ActiveBaud = baud;
                    break;
                }
                catch (Exception ex)
                {
                    last = ex;
                    L("  " + baud + " baud: " + ex.Message);
                    hci.EndSession();
                    hci.Close();
                }
            }
            if (_hci == null)
            {
                SetState(ArmState.Error, last != null ? last.Message : "Could not connect");
                throw last ?? new HciException("Could not connect");
            }
            _stop = false;
            _thread = new Thread(PollLoop) { IsBackground = true, Name = "MicroScribe poll", Priority = ThreadPriority.AboveNormal };
            _thread.Start();
            SetState(ArmState.Connected, Description);
        }

        /// <summary>Reads identification, encoder resolutions and calibration (read-only commands only).</summary>
        public static ArmInfo ReadInfo(HciSerial hci, string productId)
        {
            var info = new ArmInfo();
            info.ProductId = productId;
            info.ProductName = hci.GetString(HciSerial.GET_PROD_NAME);
            try { info.ProductId = hci.GetString(HciSerial.GET_PROD_ID); } catch (HciException) { }
            info.ModelName = hci.GetString(HciSerial.GET_MODEL_NAME);
            info.SerialNumber = hci.GetString(HciSerial.GET_SERNUM);
            info.Comment = hci.GetString(HciSerial.GET_COMMENT);
            info.ParamFormat = hci.GetString(HciSerial.GET_PRM_FORMAT);
            info.Version = hci.GetString(HciSerial.GET_VERSION);
            info.Maxes = hci.GetFixed(HciSerial.GET_MAXES, 24);
            info.ButtonsSupported = info.Maxes[0];
            // A 6th encoder (stylus roll) is present when its max is non-zero.
            // Observed: the 5-axis MicroScribe 3DX (model DX) reports 0 here.
            int max6 = (info.Maxes[22] << 8) | info.Maxes[23];
            info.NumEncoders = max6 != 0 ? 6 : 5;
            info.ParamBlock = hci.GetBlock(HciSerial.GET_PARAMS);
            if (info.Comment.Contains("Beta"))
            {
                try { info.ExtParamBlock = hci.GetBlock(HciSerial.GET_EXT_PARAMS); } catch (HciException) { }
            }
            try { info.HomeRef = hci.GetFixed(HciSerial.GET_HOME_REF, 12); } catch (HciException) { }
            return info;
        }

        public void Disconnect()
        {
            _stop = true;
            if (_thread != null)
            {
                if (!_thread.Join(1500)) { try { _thread.Abort(); } catch { } }
                _thread = null;
            }
            if (_hci != null)
            {
                _hci.EndSession();
                _hci.Close();
                _hci = null;
            }
            if (State != ArmState.Disconnected) SetState(ArmState.Disconnected, "Disconnected");
        }

        public void Dispose() { Disconnect(); }

        public void Rehome()
        {
            lock (_commands) _commands.Enqueue(h => h.HomePos());
        }

        /// <summary>Computes a sample from raw encoders (used by the poll loop and by tests).</summary>
        public ArmSample Compute(int[] encoders, int buttons)
        {
            var s = new ArmSample();
            s.Encoders = encoders;
            s.Buttons = buttons;
            s.JointAngles = _model.JointAngles(encoders);
            s.StylusFrame = _model.StylusFrame(s.JointAngles);
            s.JointOrigins = _model.JointOrigins(s.JointAngles);
            Vec3 off = TipOffset;
            s.Tip = s.StylusFrame.TransformPoint(off);
            s.Direction = s.StylusFrame.Column(2).Normalized();
            s.Timestamp = Stopwatch.GetTimestamp();
            return s;
        }

        void PollLoop()
        {
            byte cmd = HciSerial.StdCmd(Info.NumEncoders, false);
            int failures = 0;
            var rateWatch = Stopwatch.StartNew();
            int count = 0;
            while (!_stop)
            {
                try
                {
                    Action<HciSerial> c = null;
                    lock (_commands) if (_commands.Count > 0) c = _commands.Dequeue();
                    if (c != null) { c(_hci); continue; }

                    HciPacket p = _hci.Query(cmd, 300);
                    failures = 0;
                    ArmSample s = Compute(p.Encoders, p.Buttons);
                    lock (_lock) _latest = s;

                    count++;
                    if (rateWatch.ElapsedMilliseconds >= 1000)
                    {
                        _rate = count * 1000.0 / rateWatch.ElapsedMilliseconds;
                        count = 0; rateWatch.Restart();
                    }

                    var hs = SampleReceived;
                    if (hs != null) hs(s);

                    int changed = s.Buttons ^ _lastButtons;
                    _lastButtons = s.Buttons;
                    if (changed != 0)
                    {
                        var hb = ButtonChanged;
                        for (int b = 0; b < 7; b++)
                            if ((changed & (1 << b)) != 0 && hb != null) hb(b, (s.Buttons & (1 << b)) != 0, s);
                    }
                }
                catch (Exception ex)
                {
                    if (_stop) break;
                    failures++;
                    L("Poll error: " + ex.Message);
                    if (failures >= 5)
                    {
                        // Try to recover the session (e.g. after a cable glitch).
                        try
                        {
                            _hci.Drain(50);
                            _hci.SignOn(1500);
                            failures = 0;
                            L("Session recovered");
                        }
                        catch (Exception ex2)
                        {
                            SetState(ArmState.Error, "Lost connection: " + ex2.Message);
                            Thread.Sleep(500);
                        }
                    }
                }
            }
        }
    }
}
