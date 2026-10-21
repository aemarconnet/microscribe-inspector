using System;
using System.IO;
using System.Media;

namespace MicroScribe.App
{
    /// <summary>Synthesized feedback sounds (no audio files needed).</summary>
    public static class Sounds
    {
        const int Rate = 44100;
        static SoundPlayer _good, _success, _blip, _warn;
        public static bool Enabled = true;

        static Sounds()
        {
            try
            {
                // bright rising "ding-ding!" (major sixth) — a good point
                _good = Player(Mix(Bell(1318.5, 0.00, 0.22, 0.30), Bell(2217.5, 0.07, 0.35, 0.32)));
                // C6 E6 G6 C7 arpeggio — feature finished with a clean fit
                _success = Player(Mix(Bell(1046.5, 0.00, 0.30, 0.22), Bell(1318.5, 0.07, 0.30, 0.22), Bell(1568.0, 0.14, 0.32, 0.24), Bell(2093.0, 0.21, 0.45, 0.28)));
                // soft single blip — point captured, not enough points to judge yet
                _blip = Player(Bell(1760.0, 0.0, 0.10, 0.20));
                // gentle descending "hmm" — point does not fit the others
                _warn = Player(Mix(Soft(523.3, 0.00, 0.16, 0.25), Soft(392.0, 0.13, 0.26, 0.25)));
            }
            catch { Enabled = false; }
        }

        public static void Good() { Play(_good); }
        public static void Success() { Play(_success); }
        public static void Blip() { Play(_blip); }
        public static void Warn() { Play(_warn); }

        static void Play(SoundPlayer p)
        {
            if (!Enabled || p == null) return;
            try { p.Play(); } catch { }
        }

        /// <summary>Bell-like tone: fundamental plus a few inharmonic partials with exponential decay.</summary>
        static double[] Bell(double f, double start, double decay, double amp)
        {
            int n = (int)((start + decay * 5) * Rate);
            var s = new double[n];
            int s0 = (int)(start * Rate);
            for (int i = s0; i < n; i++)
            {
                double t = (i - s0) / (double)Rate;
                double env = Math.Min(1, t / 0.004) * Math.Exp(-t / decay);
                double v = Math.Sin(2 * Math.PI * f * t) + 0.35 * Math.Sin(2 * Math.PI * f * 2.01 * t) * Math.Exp(-t / (decay * 0.5))
                           + 0.12 * Math.Sin(2 * Math.PI * f * 3.02 * t) * Math.Exp(-t / (decay * 0.3));
                s[i] = amp * env * v / 1.47;
            }
            return s;
        }

        /// <summary>Soft rounded tone (for the warning).</summary>
        static double[] Soft(double f, double start, double len, double amp)
        {
            int n = (int)((start + len + 0.05) * Rate);
            var s = new double[n];
            int s0 = (int)(start * Rate);
            for (int i = s0; i < n; i++)
            {
                double t = (i - s0) / (double)Rate;
                double env = Math.Min(1, t / 0.015) * Math.Max(0, 1 - t / (len + 0.05));
                s[i] = amp * env * (Math.Sin(2 * Math.PI * f * t) + 0.2 * Math.Sin(2 * Math.PI * f * 2 * t));
            }
            return s;
        }

        static double[] Mix(params double[][] parts)
        {
            int n = 0;
            foreach (var p in parts) n = Math.Max(n, p.Length);
            var r = new double[n];
            foreach (var p in parts) for (int i = 0; i < p.Length; i++) r[i] += p[i];
            return r;
        }

        static SoundPlayer Player(double[] samples)
        {
            var ms = new MemoryStream();
            var w = new BinaryWriter(ms);
            int dataLen = samples.Length * 2;
            w.Write(new[] { 'R', 'I', 'F', 'F' }); w.Write(36 + dataLen);
            w.Write(new[] { 'W', 'A', 'V', 'E' }); w.Write(new[] { 'f', 'm', 't', ' ' });
            w.Write(16); w.Write((short)1); w.Write((short)1); w.Write(Rate); w.Write(Rate * 2); w.Write((short)2); w.Write((short)16);
            w.Write(new[] { 'd', 'a', 't', 'a' }); w.Write(dataLen);
            foreach (double v in samples) w.Write((short)Math.Max(-32767, Math.Min(32767, v * 32767)));
            w.Flush();
            ms.Position = 0;
            var p = new SoundPlayer(ms);
            p.Load();
            return p;
        }

        /// <summary>Writes the sounds as .wav files (for checking them by ear).</summary>
        public static void Dump(string dir)
        {
            Directory.CreateDirectory(dir);
            foreach (var kv in new[] { new { n = "good", p = _good }, new { n = "success", p = _success }, new { n = "blip", p = _blip }, new { n = "warn", p = _warn } })
            {
                var s = kv.p.Stream;
                s.Position = 0;
                using (var f = File.Create(Path.Combine(dir, kv.n + ".wav"))) s.CopyTo(f);
                s.Position = 0;
            }
        }
    }
}
