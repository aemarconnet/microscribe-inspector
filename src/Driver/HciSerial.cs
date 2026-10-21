using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace MicroScribe.Driver
{
    public class HciException : Exception
    {
        public HciException(string message) : base(message) { }
    }

    /// <summary>One decoded standard data packet.</summary>
    public sealed class HciPacket
    {
        public int Buttons;
        public int Timer = -1;
        public int[] Encoders = new int[0];
        public long TimestampTicks;
    }

    /// <summary>
    /// Low-level MicroScribe serial ("HCI") protocol (see docs/PROTOCOL.md).
    /// Not thread-safe: use from one thread at a time.
    /// </summary>
    public sealed class HciSerial : IDisposable
    {
        // Command bytes (docs/PROTOCOL.md §3)
        public const byte GET_PARAMS = 0xC0, GET_HOME_REF = 0xC1, HOME_POS = 0xC2, END_SESSION = 0xC5,
            GET_MAXES = 0xC6, GET_PROD_NAME = 0xC8, GET_PROD_ID = 0xC9, GET_MODEL_NAME = 0xCA,
            GET_SERNUM = 0xCB, GET_COMMENT = 0xCC, GET_PRM_FORMAT = 0xCD, GET_VERSION = 0xCE,
            GET_EXT_PARAMS = 0xD3;
        public const byte TIMER_BIT = 0x20;

        SerialPort _port;
        readonly Queue<byte> _rx = new Queue<byte>();
        readonly byte[] _buf = new byte[512];

        /// <summary>Optional trace sink: receives "TX ..."/"RX ..." hex lines.</summary>
        public Action<string> Trace;

        public string PortName { get; private set; }
        public int BaudRate { get; private set; }
        public bool IsOpen { get { return _port != null && _port.IsOpen; } }

        public void Open(string portName, int baud, bool dtr = true, bool rts = true)
        {
            Close();
            _port = new SerialPort(portName, baud, Parity.None, 8, StopBits.One);
            _port.Handshake = Handshake.None;
            _port.DtrEnable = dtr;
            _port.RtsEnable = rts;
            _port.ReadBufferSize = 4096;
            _port.WriteTimeout = 500;
            _port.ReadTimeout = 50;
            _port.Open();
            PortName = portName;
            BaudRate = baud;
            _rx.Clear();
        }

        public void Close()
        {
            if (_port != null)
            {
                try { if (_port.IsOpen) _port.Close(); } catch { }
                _port.Dispose();
                _port = null;
            }
        }

        public void Dispose() { Close(); }

        // ------------------------------------------------------------------ raw I/O

        public void Write(params byte[] data)
        {
            if (Trace != null) Trace("TX " + Hex(data, data.Length));
            _port.Write(data, 0, data.Length);
        }

        public void Write(string ascii) { Write(Encoding.ASCII.GetBytes(ascii)); }

        /// <summary>Pulls whatever is available (waiting up to waitMs for the first byte).</summary>
        int Pump(int waitMs)
        {
            int n = 0;
            if (_port.BytesToRead == 0 && waitMs > 0)
            {
                _port.ReadTimeout = Math.Max(1, waitMs);
                try { n = _port.Read(_buf, 0, _buf.Length); }
                catch (TimeoutException) { n = 0; }
            }
            else if (_port.BytesToRead > 0)
            {
                n = _port.Read(_buf, 0, Math.Min(_buf.Length, _port.BytesToRead));
            }
            if (n > 0)
            {
                if (Trace != null) Trace("RX " + Hex(_buf, n));
                for (int i = 0; i < n; i++) _rx.Enqueue(_buf[i]);
            }
            return n;
        }

        /// <summary>Reads one byte, or -1 on timeout.</summary>
        public int ReadByte(int timeoutMs)
        {
            if (_rx.Count > 0) return _rx.Dequeue();
            var sw = Stopwatch.StartNew();
            while (true)
            {
                int left = timeoutMs - (int)sw.ElapsedMilliseconds;
                if (left <= 0) return -1;
                Pump(left);
                if (_rx.Count > 0) return _rx.Dequeue();
            }
        }

        public void Purge()
        {
            _rx.Clear();
            try { _port.DiscardInBuffer(); } catch { }
        }

        /// <summary>Discards input until the line has been quiet for quietMs.</summary>
        public void Drain(int quietMs)
        {
            var sw = Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < 2000)
            {
                if (Pump(quietMs) == 0) break;
            }
            _rx.Clear();
        }

        // ------------------------------------------------------------------ session

        /// <summary>
        /// Sign-on: repeats END_SESSION + "IMMC" until the arm echoes "IMMC" (it autobauds on it),
        /// then sends "BEGIN" and returns the product-ID string.
        /// </summary>
        public string SignOn(int timeoutMs = 2000)
        {
            Purge();
            var sw = Stopwatch.StartNew();
            var seen = new StringBuilder();
            bool ok = false;
            while (sw.ElapsedMilliseconds < timeoutMs && !ok)
            {
                Write(END_SESSION);
                Write("IMMC");
                Thread.Sleep(15);
                Pump(20);
                while (_rx.Count > 0)
                {
                    int b = _rx.Dequeue();
                    seen.Append((char)b);
                }
                if (seen.ToString().Contains("IMMC")) ok = true;
                if (seen.Length > 256) seen.Remove(0, seen.Length - 16);
            }
            if (!ok) throw new HciException("No response to IMMC sign-on on " + PortName + " @ " + BaudRate + " baud (arm off, wrong port, or cable?)");
            Drain(60);
            Write("BEGIN");
            string id = ReadCString(1000);
            if (id == null) throw new HciException("Arm answered IMMC but not BEGIN");
            return id;
        }

        public void EndSession()
        {
            if (!IsOpen) return;
            try { Write(END_SESSION); Thread.Sleep(20); } catch { }
        }

        // ------------------------------------------------------------------ config commands

        bool WaitEcho(byte cmd, int timeoutMs)
        {
            var sw = Stopwatch.StartNew();
            while (true)
            {
                int left = timeoutMs - (int)sw.ElapsedMilliseconds;
                if (left <= 0) return false;
                int b = ReadByte(left);
                if (b < 0) return false;
                if (b == cmd) return true;
            }
        }

        string ReadCString(int timeoutMs)
        {
            var sb = new StringBuilder();
            var sw = Stopwatch.StartNew();
            while (true)
            {
                int left = timeoutMs - (int)sw.ElapsedMilliseconds;
                if (left <= 0) return sb.Length > 0 ? sb.ToString() : null;
                int b = ReadByte(left);
                if (b < 0) return sb.Length > 0 ? sb.ToString() : null;
                if (b == 0) return sb.ToString();
                if (sb.Length < 200) sb.Append((char)b);
            }
        }

        public string GetString(byte cmd, int timeoutMs = 1000)
        {
            Write(cmd);
            if (!WaitEcho(cmd, timeoutMs)) throw new HciException(string.Format("No echo for string command 0x{0:X2}", cmd));
            string s = ReadCString(timeoutMs);
            if (s == null) throw new HciException(string.Format("Timeout reading string for 0x{0:X2}", cmd));
            return s.TrimEnd(' ', '\0');
        }

        /// <summary>Length-prefixed block (GET_PARAMS / GET_EXT_PARAMS).</summary>
        public byte[] GetBlock(byte cmd, int timeoutMs = 2000)
        {
            Write(cmd);
            if (!WaitEcho(cmd, timeoutMs)) throw new HciException(string.Format("No echo for block command 0x{0:X2}", cmd));
            int n = ReadByte(timeoutMs);
            if (n < 0) throw new HciException("Timeout reading block length");
            return ReadExactly(n, timeoutMs);
        }

        public byte[] GetFixed(byte cmd, int length, int timeoutMs = 1000)
        {
            Write(cmd);
            if (!WaitEcho(cmd, timeoutMs)) throw new HciException(string.Format("No echo for command 0x{0:X2}", cmd));
            return ReadExactly(length, timeoutMs);
        }

        byte[] ReadExactly(int n, int timeoutMs)
        {
            var r = new byte[n];
            var sw = Stopwatch.StartNew();
            for (int i = 0; i < n; i++)
            {
                int left = timeoutMs - (int)sw.ElapsedMilliseconds;
                int b = left > 0 ? ReadByte(left) : -1;
                if (b < 0) throw new HciException(string.Format("Timeout: got {0} of {1} bytes", i, n));
                r[i] = (byte)b;
            }
            return r;
        }

        /// <summary>Re-zeros all encoders at the current pose. Only call with the arm in its home pose.</summary>
        public void HomePos()
        {
            Write(HOME_POS);
            if (!WaitEcho(HOME_POS, 1000)) throw new HciException("No echo for HOME_POS");
        }

        // ------------------------------------------------------------------ data packets

        public static byte StdCmd(int encoders, bool timer)
        {
            int e = encoders >= 7 ? 0x02 : encoders == 6 ? 0x03 : encoders > 0 ? 0x01 : 0x00;
            return (byte)(e | (timer ? TIMER_BIT : 0));
        }

        public static int PayloadSize(int header)
        {
            int n = 1;
            if ((header & TIMER_BIT) != 0) n += 2;
            switch (header & 0x0C) { case 0x04: n += 3; break; case 0x08: n += 5; break; case 0x0C: n += 9; break; }
            switch (header & 0x03) { case 0x01: n += 10; break; case 0x02: n += 14; break; case 0x03: n += 12; break; }
            return n;
        }

        public static int EncoderCount(int header)
        {
            switch (header & 0x03) { case 0x01: return 5; case 0x02: return 7; case 0x03: return 6; }
            return 0;
        }

        /// <summary>Sends a standard query and reads the reply.</summary>
        public HciPacket Query(byte cmd, int timeoutMs = 250)
        {
            Write(cmd);
            return ReadPacket(cmd, timeoutMs);
        }

        public HciPacket ReadPacket(byte cmd, int timeoutMs)
        {
            int header = cmd | 0x80;
            var sw = Stopwatch.StartNew();
            while (true)
            {
                int left = timeoutMs - (int)sw.ElapsedMilliseconds;
                int b = left > 0 ? ReadByte(left) : -1;
                if (b < 0) throw new HciException("Timeout waiting for data packet");
                if (b == header) break;
            }
            int len = PayloadSize(header);
            byte[] d = ReadExactly(len, Math.Max(50, timeoutMs - (int)sw.ElapsedMilliseconds));
            for (int i = 0; i < len; i++)
                if ((d[i] & 0x80) != 0) { Drain(20); throw new HciException("Corrupt packet (bit 7 set in payload)"); }

            var p = new HciPacket();
            p.TimestampTicks = Stopwatch.GetTimestamp();
            int pos = 0;
            p.Buttons = d[pos++];
            if ((header & TIMER_BIT) != 0) { p.Timer = (d[pos] << 7) | d[pos + 1]; pos += 2; }
            switch (header & 0x0C) { case 0x04: pos += 3; break; case 0x08: pos += 5; break; case 0x0C: pos += 9; break; }
            int ne = EncoderCount(header);
            p.Encoders = new int[ne];
            for (int i = 0; i < ne; i++) { p.Encoders[i] = (d[pos] << 7) | d[pos + 1]; pos += 2; }
            return p;
        }

        public static string Hex(byte[] b, int n)
        {
            var sb = new StringBuilder(n * 3);
            for (int i = 0; i < n; i++) { if (i > 0) sb.Append(' '); sb.Append(b[i].ToString("X2")); }
            return sb.ToString();
        }
    }
}
