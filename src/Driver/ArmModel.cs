using System;
using System.Globalization;
using System.Text;
using MicroScribe.Core;

namespace MicroScribe.Driver
{
    /// <summary>Identification strings and raw calibration read from the arm.</summary>
    public sealed class ArmInfo
    {
        public string ProductId = "", ProductName = "", ModelName = "", SerialNumber = "",
            Comment = "", ParamFormat = "", Version = "";
        public byte[] ParamBlock;     // GET_PARAMS payload (36 bytes)
        public byte[] ExtParamBlock;  // GET_EXT_PARAMS payload (may be null)
        public byte[] Maxes;          // GET_MAXES payload (24 bytes)
        public byte[] HomeRef;        // GET_HOME_REF payload (12 bytes, diagnostic)
        public int ButtonsSupported;
        public int NumEncoders = 5;

        public override string ToString()
        {
            var sb = new StringBuilder();
            sb.AppendLine("Product ID   : " + ProductId);
            sb.AppendLine("Product name : " + ProductName);
            sb.AppendLine("Model        : " + ModelName);
            sb.AppendLine("Serial       : " + SerialNumber);
            sb.AppendLine("Comment      : " + Comment);
            sb.AppendLine("Param format : " + ParamFormat);
            sb.AppendLine("Firmware     : " + Version);
            return sb.ToString();
        }
    }

    /// <summary>
    /// Denavit-Hartenberg model of the arm, decoded from the arm's own parameter block
    /// (see docs/PROTOCOL.md §3 and §5 for the public sources of the format).
    /// All lengths in millimetres, angles in radians.
    /// </summary>
    public sealed class ArmModel
    {
        public const double MmPerInch = 25.4;
        public const int Joints = 6;

        public readonly double[] Alpha = new double[Joints];
        public readonly double[] A = new double[Joints];
        public readonly double[] D = new double[Joints];
        public double Beta;
        /// <summary>Counts per revolution for each encoder (0 = joint not present).</summary>
        public readonly int[] Cpr = new int[Joints];
        public int NumEncoders = 5;
        /// <summary>Factory stylus length D[5] as read from the arm (mm).</summary>
        public double FactoryStylusLength;

        double[] _ca = new double[Joints], _sa = new double[Joints];
        double _cb = 1, _sb = 0;

        public static ArmModel FromInfo(ArmInfo info)
        {
            var m = new ArmModel();
            m.Decode(info.ParamBlock, info.ExtParamBlock, info.Comment, info.Maxes, info.NumEncoders);
            return m;
        }

        static int I16(byte[] b, int i) { return (short)((b[i] << 8) | b[i + 1]); }
        static int U16(byte[] b, int i) { return (b[i] << 8) | b[i + 1]; }

        public void Decode(byte[] pb, byte[] ext, string comment, byte[] maxes, int numEncoders)
        {
            if (pb == null || pb.Length != 36)
                throw new HciException("Unsupported parameter block length " + (pb == null ? 0 : pb.Length) + " (expected 36)");
            for (int i = 0; i < Joints; i++)
            {
                Alpha[i] = I16(pb, 2 * i) / 32768.0 * Math.PI;
                A[i] = I16(pb, 12 + 2 * i) / 1000.0 * MmPerInch;
                D[i] = I16(pb, 24 + 2 * i) / 1000.0 * MmPerInch;
            }
            FactoryStylusLength = D[5];
            Beta = 0;
            if (comment != null && comment.Contains("Beta") && ext != null && ext.Length >= 2)
                Beta = I16(ext, 0) / 32768.0 * Math.PI;

            NumEncoders = numEncoders;
            for (int i = 0; i < Joints; i++) Cpr[i] = 0;
            if (maxes != null && maxes.Length >= 24)
            {
                // counts per revolution = max + 1 (the max is the highest count, e.g. 0x3FFF -> 16384)
                for (int i = 0; i < Math.Min(numEncoders, 6); i++) Cpr[i] = U16(maxes, 12 + 2 * i) + 1;
            }
            Precompute();
        }

        public void Precompute()
        {
            for (int i = 0; i < Joints; i++) { _ca[i] = Math.Cos(Alpha[i]); _sa[i] = Math.Sin(Alpha[i]); }
            _cb = Math.Cos(Beta); _sb = Math.Sin(Beta);
        }

        /// <summary>Joint angles (rad) from raw encoder counts.</summary>
        public double[] JointAngles(int[] enc)
        {
            var q = new double[Joints];
            for (int i = 0; i < Joints; i++)
            {
                if (enc == null || i >= enc.Length || Cpr[i] <= 0) { q[i] = 0; continue; }
                int c = enc[i] % Cpr[i];
                if (c < 0) c += Cpr[i];
                q[i] = c * 2.0 * Math.PI / Cpr[i];
            }
            return q;
        }

        /// <summary>Link transform M[i] (modified DH, BETA on joint 2).</summary>
        public Mat4 LinkTransform(int i, double theta)
        {
            double c = Math.Cos(theta), s = Math.Sin(theta);
            double ca = _ca[i], sa = _sa[i], a = A[i], d = D[i];
            var m = Mat4.Identity();
            if (i != 2)
            {
                m[0, 0] = c; m[0, 1] = -s; m[0, 2] = 0; m[0, 3] = a;
                m[1, 0] = s * ca; m[1, 1] = c * ca; m[1, 2] = -sa; m[1, 3] = -sa * d;
                m[2, 0] = s * sa; m[2, 1] = c * sa; m[2, 2] = ca; m[2, 3] = ca * d;
            }
            else
            {
                double cb = _cb, sb = _sb;
                m[0, 0] = c * cb; m[0, 1] = -s * cb; m[0, 2] = sb; m[0, 3] = sb * d + a;
                m[1, 0] = s * ca + sa * sb * c; m[1, 1] = c * ca - sa * sb * s; m[1, 2] = -sa * cb; m[1, 3] = -sa * cb * d;
                m[2, 0] = s * sa - ca * sb * c; m[2, 1] = c * sa + s * sb * ca; m[2, 2] = ca * cb; m[2, 3] = cb * ca * d;
            }
            return m;
        }

        /// <summary>Stylus frame (after joint 5, before any tip offset) in the base frame, mm.</summary>
        public Mat4 StylusFrame(double[] q)
        {
            Mat4 t = LinkTransform(0, q[0]);
            for (int i = 1; i < Joints; i++) t = t * LinkTransform(i, q[i]);
            return t;
        }

        /// <summary>Intermediate joint origins (for drawing the arm skeleton).</summary>
        public Vec3[] JointOrigins(double[] q)
        {
            var pts = new Vec3[Joints + 1];
            pts[0] = Vec3.Zero;
            Mat4 t = Mat4.Identity();
            for (int i = 0; i < Joints; i++)
            {
                t = t * LinkTransform(i, q[i]);
                pts[i + 1] = t.Translation3;
            }
            return pts;
        }

        public string Describe()
        {
            var sb = new StringBuilder();
            var ci = CultureInfo.InvariantCulture;
            sb.AppendLine(" i   alpha(deg)     A(mm)      D(mm)    cpr");
            for (int i = 0; i < Joints; i++)
                sb.AppendLine(string.Format(ci, " {0}  {1,10:0.0000} {2,10:0.0000} {3,10:0.0000} {4,6}",
                    i, Alpha[i] * 180 / Math.PI, A[i], D[i], Cpr[i]));
            sb.AppendLine(string.Format(ci, " beta = {0:0.00000} deg", Beta * 180 / Math.PI));
            return sb.ToString();
        }
    }
}
