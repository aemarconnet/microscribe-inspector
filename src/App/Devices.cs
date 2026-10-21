using System;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using MicroScribe.Core;
using MicroScribe.Driver;

namespace MicroScribe.App
{
    /// <summary>
    /// Simulated arm with a virtual test part (block with bore, boss, ball and countersink) placed in
    /// front of the base. The tip wanders over the chosen target surface; pedals are pressed from the UI
    /// or keyboard. Lets you try every workflow without hardware.
    /// </summary>
    public sealed class SimulatedArm : IArmDevice
    {
        public static readonly string[] Targets =
            { "Free air", "Block top", "Block front", "Block left side", "Bore Ø20", "Boss Ø25", "Ball Ø25.4", "Countersink 90°", "Top edge (curve)" };

        // Virtual part, machine frame (mm). Block corner at Origin, rotated 20° about Z.
        static readonly Vec3 Origin = new Vec3(250, -60, -80);
        static readonly double Rot = 20 * Math.PI / 180;
        static readonly Vec3 PX = new Vec3(Math.Cos(Rot), Math.Sin(Rot), 0), PY = new Vec3(-Math.Sin(Rot), Math.Cos(Rot), 0), PZ = Vec3.UnitZ;
        const double L = 120, W = 80, H = 30;

        public volatile int Target = 1;
        public double Noise = 0.04;
        /// <summary>Simulated probe ball radius: the reported tip centre sits this far off the surface.</summary>
        public double BallRadius;

        readonly object _lock = new object();
        readonly Random _rng = new Random();
        Thread _thread;
        volatile bool _stop;
        ArmSample _latest;
        Vec3 _tipOffset;
        int _buttons;
        double _u = 0.3, _v = 0.4, _du = 0.0011, _dv = 0.0007, _time;

        public event Action<ArmSample> SampleReceived;
        public event Action<int, bool, ArmSample> ButtonChanged;
        public event Action<ArmState, string> StateChanged;

        public ArmState State { get; private set; }
        public string Description { get { return "Simulated MicroScribe (virtual test block)"; } }
        public ArmInfo Info { get; private set; }
        public Vec3 TipOffset { get { lock (_lock) return _tipOffset; } set { lock (_lock) _tipOffset = value; } }
        public ArmSample Latest { get { lock (_lock) return _latest; } }
        public double SampleRate { get { return 50; } }

        static Vec3 P(double x, double y, double z) { return Origin + PX * x + PY * y + PZ * z; }

        public void Connect()
        {
            Info = new ArmInfo { ProductName = "MicroScribe3D", ModelName = "Simulator", SerialNumber = "SIM-0001", ProductId = "MSCR", Comment = "simulated", ParamFormat = "Format DH0.5", Version = "HCI 2.0", NumEncoders = 5 };
            _stop = false;
            _thread = new Thread(Loop) { IsBackground = true, Name = "Simulator" };
            _thread.Start();
            State = ArmState.Connected;
            var h = StateChanged; if (h != null) h(State, Description);
        }

        public void Disconnect()
        {
            _stop = true;
            if (_thread != null) { _thread.Join(500); _thread = null; }
            State = ArmState.Disconnected;
            var h = StateChanged; if (h != null) h(State, "Disconnected");
        }

        public void Dispose() { Disconnect(); }
        public void Rehome() { }

        public void SetButton(int index, bool down)
        {
            ArmSample s = Latest;
            lock (_lock)
            {
                if (down) _buttons |= 1 << index; else _buttons &= ~(1 << index);
            }
            var h = ButtonChanged;
            if (h != null && s != null) h(index, down, s);
        }

        double G() { double u1 = 1 - _rng.NextDouble(), u2 = _rng.NextDouble(); return Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2); }

        /// <summary>Exact point on a virtual surface (used by the demo project / self-test).</summary>
        public static void SamplePoint(int target, double u, double v, out Vec3 p, out Vec3 n)
        {
            new SimulatedArm().Surface(target, u, v, out p, out n);
        }

        /// <summary>Surface point and outward normal for the current target at (u, v) ∈ [0,1]².</summary>
        void Surface(int target, double u, double v, out Vec3 p, out Vec3 n)
        {
            switch (target)
            {
                case 1: p = P(10 + u * 60, 8 + v * 64, H); n = PZ; break;                // top (left part, away from the boss)
                case 2: p = P(5 + u * 110, 0, 4 + v * 22); n = -PY; break;               // front face
                case 3: p = P(0, 5 + u * 70, 4 + v * 22); n = -PX; break;                // left face
                case 4:
                    {
                        double t = u * 2 * Math.PI, rr = 10;
                        Vec3 c = P(95, 25, 0), rad = PX * Math.Cos(t) + PY * Math.Sin(t);
                        p = c + rad * rr + PZ * (5 + v * 22); n = -rad; break;           // bore (normal into the hole)
                    }
                case 5:
                    {
                        double t = u * 2 * Math.PI, rr = 12.5;
                        Vec3 c = P(95, 60, H), rad = PX * Math.Cos(t) + PY * Math.Sin(t);
                        p = c + rad * rr + PZ * (2 + v * 16); n = rad; break;             // boss on the top
                    }
                case 6:
                    {
                        double t = u * 2 * Math.PI, ph = v * 1.4; // cap up to 80° from the pole
                        Vec3 c = P(-40, 40, 60), d = PZ * Math.Cos(ph) + (PX * Math.Cos(t) + PY * Math.Sin(t)) * Math.Sin(ph);
                        p = c + d * 12.7; n = d; break;
                    }
                case 7:
                    {
                        double t = u * 2 * Math.PI, depth = 1 + v * 6; // 90° countersink, Ø20 at the top face
                        Vec3 c = P(30, 60, H), rad = PX * Math.Cos(t) + PY * Math.Sin(t);
                        double r = 10 - depth;
                        p = c + rad * r - PZ * depth; n = (-rad + PZ).Normalized(); break;
                    }
                case 8: p = P(5 + u * 110, 0, H); n = (-PY + PZ).Normalized(); break;    // top-front edge
                default:
                    p = P(60 + 70 * Math.Sin(_time * 0.4), 40 + 50 * Math.Sin(_time * 0.27), 80 + 25 * Math.Sin(_time * 0.6));
                    n = PZ; break;
            }
        }

        void Loop()
        {
            var sw = Stopwatch.StartNew();
            while (!_stop)
            {
                Thread.Sleep(20);
                _time = sw.Elapsed.TotalSeconds;
                int target = Target;
                bool held;
                lock (_lock) held = _buttons != 0;
                // slow random walk over the surface (slower while a pedal is held, like a user)
                double k = held ? 0.4 : 1.0;
                _u += _du * k * (target == 8 ? 3 : 1); _v += _dv * k;
                if (target == 8) _v = 0.5;
                if (_u > 1 || _u < 0) { _du = -_du; _u = Math.Max(0, Math.Min(1, _u)); }
                if (_v > 1 || _v < 0) { _dv = -_dv; _v = Math.Max(0, Math.Min(1, _v)); }
                if (_rng.NextDouble() < 0.02) { _du = (_rng.NextDouble() - 0.5) * 0.004; _dv = (_rng.NextDouble() - 0.5) * 0.004; }
                Vec3 p, n;
                Surface(target, _u, _v, out p, out n);
                Vec3 off = TipOffset;
                // stylus points into the material (against the normal), slightly tilted
                Vec3 dir = (-n + PX * 0.25 * Math.Sin(_time) + PY * 0.2 * Math.Cos(_time * 0.7)).Normalized();
                Vec3 tip = p + n * BallRadius + new Vec3(G(), G(), G()) * Noise;
                // a believable arm skeleton for display
                var s = new ArmSample
                {
                    Tip = tip,
                    Direction = dir,
                    Buttons = _buttons,
                    Timestamp = Stopwatch.GetTimestamp(),
                    Encoders = new int[5],
                    JointAngles = new double[6]
                };
                Vec3 wrist = tip - dir * 90;
                Vec3 shoulder = new Vec3(0, 0, 210);
                Vec3 mid = (shoulder + wrist) * 0.5;
                Vec3 side = (wrist - shoulder).Cross(Vec3.UnitZ);
                Vec3 elbow = mid + Vec3.UnitZ * 60 + (side.Length > 1e-6 ? side.Normalized() * 10 : Vec3.Zero);
                s.JointOrigins = new[] { Vec3.Zero, new Vec3(0, 0, 120), shoulder, elbow, wrist, wrist + dir * 40, tip };
                s.StylusFrame = Mat4.FromFrame(tip - dir * off.Z, dir.AnyPerpendicular(), dir.Cross(dir.AnyPerpendicular()), dir);
                lock (_lock) _latest = s;
                var h = SampleReceived; if (h != null) h(s);
            }
        }
    }
}
