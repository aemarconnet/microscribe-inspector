using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using MicroScribe.Core;

namespace MicroScribe.App
{
    /// <summary>Accumulates triangles into a single MeshGeometry3D (one draw call per material).</summary>
    public sealed class MeshBuilder
    {
        readonly Point3DCollection _pos = new Point3DCollection();
        readonly Vector3DCollection _nrm = new Vector3DCollection();
        readonly Int32Collection _idx = new Int32Collection();

        public int VertexCount { get { return _pos.Count; } }

        static Point3D P(Vec3 v) { return new Point3D(v.X, v.Y, v.Z); }
        static Vector3D V(Vec3 v) { return new Vector3D(v.X, v.Y, v.Z); }

        int Vert(Vec3 p, Vec3 n) { _pos.Add(P(p)); _nrm.Add(V(n)); return _pos.Count - 1; }

        void Tri(int a, int b, int c) { _idx.Add(a); _idx.Add(b); _idx.Add(c); }

        public MeshGeometry3D ToMesh()
        {
            var m = new MeshGeometry3D { Positions = _pos, Normals = _nrm, TriangleIndices = _idx };
            m.Freeze();
            return m;
        }

        public void Sphere(Vec3 c, double r, int slices = 14, int stacks = 9)
        {
            int start = _pos.Count;
            for (int i = 0; i <= stacks; i++)
            {
                double phi = Math.PI * i / stacks;
                for (int j = 0; j <= slices; j++)
                {
                    double th = 2 * Math.PI * j / slices;
                    var n = new Vec3(Math.Sin(phi) * Math.Cos(th), Math.Sin(phi) * Math.Sin(th), Math.Cos(phi));
                    Vert(c + n * r, n);
                }
            }
            for (int i = 0; i < stacks; i++)
                for (int j = 0; j < slices; j++)
                {
                    int a = start + i * (slices + 1) + j, b = a + slices + 1;
                    Tri(a, b, a + 1); Tri(a + 1, b, b + 1);
                }
        }

        public void Tube(Vec3 a, Vec3 b, double r, int sides = 8)
        {
            Vec3 d = b - a;
            if (d.Length < 1e-9) return;
            Vec3 z = d.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
            int start = _pos.Count;
            for (int j = 0; j <= sides; j++)
            {
                double t = 2 * Math.PI * j / sides;
                Vec3 n = x * Math.Cos(t) + y * Math.Sin(t);
                Vert(a + n * r, n); Vert(b + n * r, n);
            }
            for (int j = 0; j < sides; j++)
            {
                int i0 = start + 2 * j;
                Tri(i0, i0 + 2, i0 + 1); Tri(i0 + 1, i0 + 2, i0 + 3);
            }
        }

        public void Polyline(IList<Vec3> pts, double r, bool closed = false)
        {
            for (int i = 0; i + 1 < pts.Count; i++) Tube(pts[i], pts[i + 1], r, 6);
            if (closed && pts.Count > 2) Tube(pts[pts.Count - 1], pts[0], r, 6);
        }

        public void Cone(Vec3 baseCenter, Vec3 tip, double r, int sides = 16)
        {
            Vec3 d = tip - baseCenter;
            Vec3 z = d.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
            int c0 = Vert(baseCenter, -z);
            for (int j = 0; j < sides; j++)
            {
                double t0 = 2 * Math.PI * j / sides, t1 = 2 * Math.PI * (j + 1) / sides;
                Vec3 r0 = x * Math.Cos(t0) + y * Math.Sin(t0), r1 = x * Math.Cos(t1) + y * Math.Sin(t1);
                double k = r / d.Length;
                Vec3 n0 = (r0 + z * k).Normalized(), n1 = (r1 + z * k).Normalized();
                int a = Vert(baseCenter + r0 * r, n0), b = Vert(baseCenter + r1 * r, n1), t = Vert(tip, (n0 + n1).Normalized());
                Tri(a, b, t);
                int ba = Vert(baseCenter + r0 * r, -z), bb = Vert(baseCenter + r1 * r, -z);
                Tri(c0, bb, ba);
            }
        }

        public void Arrow(Vec3 from, Vec3 to, double shaftR)
        {
            Vec3 d = to - from;
            double len = d.Length;
            if (len < 1e-9) return;
            Vec3 z = d / len;
            double head = Math.Min(len * 0.25, shaftR * 8);
            Tube(from, to - z * head, shaftR, 10);
            Cone(to - z * head, to, shaftR * 2.6, 16);
        }

        /// <summary>Flat polygon (convex), both sides.</summary>
        public void Polygon(IList<Vec3> poly, Vec3 normal)
        {
            if (poly.Count < 3) return;
            int s = _pos.Count;
            foreach (var p in poly) Vert(p, normal);
            for (int i = 1; i + 1 < poly.Count; i++) Tri(s, s + i, s + i + 1);
        }

        /// <summary>Surface of revolution patch with radius r(h).</summary>
        public void Revolved(Vec3 o, Vec3 axis, Vec3 xRef, Func<double, double> radius, double slope,
                             double t0, double t1, double h0, double h1, int segU = 48, int segV = 6)
        {
            Vec3 z = axis.Normalized(), x = (xRef - z * xRef.Dot(z)).Normalized(), y = z.Cross(x);
            int nu = Math.Max(2, (int)Math.Ceiling(segU * (t1 - t0) / (2 * Math.PI)));
            int s = _pos.Count;
            for (int i = 0; i <= segV; i++)
            {
                double h = h0 + (h1 - h0) * i / segV;
                double r = radius(h);
                for (int j = 0; j <= nu; j++)
                {
                    double t = t0 + (t1 - t0) * j / nu;
                    Vec3 rad = x * Math.Cos(t) + y * Math.Sin(t);
                    Vec3 n = (rad - z * slope).Normalized();
                    Vert(o + z * h + rad * r, n);
                }
            }
            for (int i = 0; i < segV; i++)
                for (int j = 0; j < nu; j++)
                {
                    int a = s + i * (nu + 1) + j, b = a + nu + 1;
                    Tri(a, a + 1, b); Tri(a + 1, b + 1, b);
                }
        }

        public void SphereCap(Vec3 c, double r, Vec3 pole, double cap, int seg = 40)
        {
            Vec3 z = pole.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
            int rings = Math.Max(4, (int)(seg * cap / Math.PI));
            int s = _pos.Count;
            for (int i = 0; i <= rings; i++)
            {
                double phi = cap * i / rings;
                for (int j = 0; j <= seg; j++)
                {
                    double th = 2 * Math.PI * j / seg;
                    Vec3 n = z * Math.Cos(phi) + (x * Math.Cos(th) + y * Math.Sin(th)) * Math.Sin(phi);
                    Vert(c + n * r, n);
                }
            }
            for (int i = 0; i < rings; i++)
                for (int j = 0; j < seg; j++)
                {
                    int a = s + i * (seg + 1) + j, b = a + seg + 1;
                    Tri(a, b, a + 1); Tri(a + 1, b, b + 1);
                }
        }

        public void Ring(Vec3 c, Vec3 normal, double radius, double tube, int seg = 64)
        {
            Vec3 z = normal.Normalized(), x = z.AnyPerpendicular(), y = z.Cross(x);
            var pts = new List<Vec3>();
            for (int i = 0; i < seg; i++) { double t = 2 * Math.PI * i / seg; pts.Add(c + (x * Math.Cos(t) + y * Math.Sin(t)) * radius); }
            Polyline(pts, tube, true);
        }

        public void BoxEdges(Vec3 min, Vec3 max, Mat4 toWorld, double r)
        {
            var c = new Vec3[8];
            for (int i = 0; i < 8; i++)
                c[i] = toWorld.TransformPoint(new Vec3((i & 1) != 0 ? max.X : min.X, (i & 2) != 0 ? max.Y : min.Y, (i & 4) != 0 ? max.Z : min.Z));
            int[,] e = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } };
            for (int i = 0; i < 12; i++) Tube(c[e[i, 0]], c[e[i, 1]], r, 6);
        }
    }

    public static class Materials
    {
        public static Material Solid(Color c, double specular = 0.35)
        {
            var g = new MaterialGroup();
            g.Children.Add(new DiffuseMaterial(new SolidColorBrush(c)));
            if (specular > 0)
                g.Children.Add(new SpecularMaterial(new SolidColorBrush(Color.FromArgb((byte)(255 * specular), 255, 255, 255)), 40));
            g.Freeze();
            return g;
        }

        public static Material Translucent(Color c, double alpha)
        {
            var b = new SolidColorBrush(Color.FromArgb((byte)(255 * alpha), c.R, c.G, c.B));
            var g = new MaterialGroup();
            g.Children.Add(new DiffuseMaterial(b));
            g.Children.Add(new SpecularMaterial(new SolidColorBrush(Color.FromArgb(60, 255, 255, 255)), 30));
            g.Freeze();
            return g;
        }

        public static Material Emissive(Color c)
        {
            var g = new MaterialGroup();
            g.Children.Add(new DiffuseMaterial(new SolidColorBrush(Color.FromRgb((byte)(c.R / 3), (byte)(c.G / 3), (byte)(c.B / 3)))));
            g.Children.Add(new EmissiveMaterial(new SolidColorBrush(c)));
            g.Freeze();
            return g;
        }
    }
}
