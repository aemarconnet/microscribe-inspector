using System;
using System.Globalization;

namespace MicroScribe.Core
{
    /// <summary>Double-precision 3D vector.</summary>
    public struct Vec3
    {
        public double X, Y, Z;

        public Vec3(double x, double y, double z) { X = x; Y = y; Z = z; }

        public static readonly Vec3 Zero = new Vec3(0, 0, 0);
        public static readonly Vec3 UnitX = new Vec3(1, 0, 0);
        public static readonly Vec3 UnitY = new Vec3(0, 1, 0);
        public static readonly Vec3 UnitZ = new Vec3(0, 0, 1);

        public static Vec3 operator +(Vec3 a, Vec3 b) { return new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
        public static Vec3 operator -(Vec3 a, Vec3 b) { return new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
        public static Vec3 operator -(Vec3 a) { return new Vec3(-a.X, -a.Y, -a.Z); }
        public static Vec3 operator *(Vec3 a, double s) { return new Vec3(a.X * s, a.Y * s, a.Z * s); }
        public static Vec3 operator *(double s, Vec3 a) { return new Vec3(a.X * s, a.Y * s, a.Z * s); }
        public static Vec3 operator /(Vec3 a, double s) { return new Vec3(a.X / s, a.Y / s, a.Z / s); }

        public double Dot(Vec3 b) { return X * b.X + Y * b.Y + Z * b.Z; }
        public Vec3 Cross(Vec3 b) { return new Vec3(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X); }
        public double Length { get { return Math.Sqrt(X * X + Y * Y + Z * Z); } }
        public double LengthSquared { get { return X * X + Y * Y + Z * Z; } }

        public Vec3 Normalized()
        {
            double l = Length;
            return l > 1e-300 ? new Vec3(X / l, Y / l, Z / l) : new Vec3(0, 0, 1);
        }

        public double this[int i]
        {
            get { return i == 0 ? X : (i == 1 ? Y : Z); }
            set { if (i == 0) X = value; else if (i == 1) Y = value; else Z = value; }
        }

        /// <summary>Any unit vector perpendicular to this one.</summary>
        public Vec3 AnyPerpendicular()
        {
            Vec3 n = Normalized();
            Vec3 a = Math.Abs(n.X) < 0.9 ? UnitX : UnitY;
            return n.Cross(a).Normalized();
        }

        public static double Distance(Vec3 a, Vec3 b) { return (a - b).Length; }

        public override string ToString()
        {
            return string.Format(CultureInfo.InvariantCulture, "({0:0.0000}, {1:0.0000}, {2:0.0000})", X, Y, Z);
        }
    }

    /// <summary>Row-major 4x4 homogeneous transform, M[r,c].</summary>
    public sealed class Mat4
    {
        public readonly double[] M = new double[16];

        public double this[int r, int c]
        {
            get { return M[r * 4 + c]; }
            set { M[r * 4 + c] = value; }
        }

        public static Mat4 Identity()
        {
            var m = new Mat4();
            m.M[0] = m.M[5] = m.M[10] = m.M[15] = 1;
            return m;
        }

        public static Mat4 Translation(double x, double y, double z)
        {
            var m = Identity();
            m.M[3] = x; m.M[7] = y; m.M[11] = z;
            return m;
        }

        public static Mat4 operator *(Mat4 a, Mat4 b)
        {
            var r = new Mat4();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 4; k++) s += a.M[i * 4 + k] * b.M[k * 4 + j];
                    r.M[i * 4 + j] = s;
                }
            return r;
        }

        public Vec3 TransformPoint(Vec3 p)
        {
            return new Vec3(
                M[0] * p.X + M[1] * p.Y + M[2] * p.Z + M[3],
                M[4] * p.X + M[5] * p.Y + M[6] * p.Z + M[7],
                M[8] * p.X + M[9] * p.Y + M[10] * p.Z + M[11]);
        }

        public Vec3 TransformVector(Vec3 v)
        {
            return new Vec3(
                M[0] * v.X + M[1] * v.Y + M[2] * v.Z,
                M[4] * v.X + M[5] * v.Y + M[6] * v.Z,
                M[8] * v.X + M[9] * v.Y + M[10] * v.Z);
        }

        public Vec3 Translation3 { get { return new Vec3(M[3], M[7], M[11]); } }
        public Vec3 Column(int c) { return new Vec3(M[c], M[4 + c], M[8 + c]); }

        /// <summary>Inverse of a rigid transform (rotation + translation).</summary>
        public Mat4 RigidInverse()
        {
            var r = Identity();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    r.M[i * 4 + j] = M[j * 4 + i];
            Vec3 t = Translation3;
            r.M[3] = -(r.M[0] * t.X + r.M[1] * t.Y + r.M[2] * t.Z);
            r.M[7] = -(r.M[4] * t.X + r.M[5] * t.Y + r.M[6] * t.Z);
            r.M[11] = -(r.M[8] * t.X + r.M[9] * t.Y + r.M[10] * t.Z);
            return r;
        }

        /// <summary>Builds a rigid frame from origin + orthonormal axes.</summary>
        public static Mat4 FromFrame(Vec3 origin, Vec3 xAxis, Vec3 yAxis, Vec3 zAxis)
        {
            var m = Identity();
            m.M[0] = xAxis.X; m.M[1] = yAxis.X; m.M[2] = zAxis.X; m.M[3] = origin.X;
            m.M[4] = xAxis.Y; m.M[5] = yAxis.Y; m.M[6] = zAxis.Y; m.M[7] = origin.Y;
            m.M[8] = xAxis.Z; m.M[9] = yAxis.Z; m.M[10] = zAxis.Z; m.M[11] = origin.Z;
            return m;
        }

        public Mat4 Clone()
        {
            var r = new Mat4();
            Array.Copy(M, r.M, 16);
            return r;
        }
    }
}
