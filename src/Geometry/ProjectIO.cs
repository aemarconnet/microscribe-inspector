using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MicroScribe.Geometry;
using System.Xml;
using System.Xml.Serialization;

namespace MicroScribe.Model
{
    /// <summary>Project files (*.msproj) are plain XML so they stay readable and diff-able.</summary>
    public static class ProjectIO
    {
        public const string Extension = ".msproj";
        static readonly XmlSerializer Ser = new XmlSerializer(typeof(Project));

        public static void Save(Project p, string path)
        {
            string tmp = path + ".tmp";
            var settings = new XmlWriterSettings { Indent = true, Encoding = new UTF8Encoding(false) };
            using (var w = XmlWriter.Create(tmp, settings)) Ser.Serialize(w, p);
            if (File.Exists(path)) File.Replace(tmp, path, null);
            else File.Move(tmp, path);
        }

        public static Project Load(string path)
        {
            Project p;
            using (var r = XmlReader.Create(path)) p = (Project)Ser.Deserialize(r);
            if (p.AlignmentRecipe == null) p.AlignmentRecipe = new AlignmentRecipe();
            FeatureSolver.SolveAll(p);
            return p;
        }

        /// <summary>Deep copy through the file format (runtime-only data such as the model mesh is shared).</summary>
        public static Project Clone(Project p)
        {
            var sb = new StringBuilder();
            using (var w = XmlWriter.Create(sb)) Ser.Serialize(w, p);
            Project c;
            using (var r = XmlReader.Create(new StringReader(sb.ToString()))) c = (Project)Ser.Deserialize(r);
            if (c.AlignmentRecipe == null) c.AlignmentRecipe = new AlignmentRecipe();
            c.NominalModel = p.NominalModel;
            return c;
        }
    }

    /// <summary>
    /// Inspection plans: a project measured once ("learned") is replayed on the next parts. The run is a
    /// copy with the same features (ids, names, types, options, notes as operator prompts), constructions,
    /// alignment recipe and measurements with their tolerances — but no points. The plan's own points
    /// show the operator where to probe and how many points each step takes.
    /// </summary>
    public static class InspectionPlan
    {
        /// <summary>The features the operator measures, in order (constructions follow automatically).</summary>
        public static List<Feature> Steps(Project plan)
        {
            return plan.Features.Where(f => f.Origin == FeatureOrigin.Measured).ToList();
        }

        /// <summary>Points to take in a step: as many as the plan used (traced curves: until the operator finishes).</summary>
        public static int PlannedPoints(Feature f)
        {
            if (f.Kind == FeatureKind.Curve || f.Kind == FeatureKind.Surface) return 0;
            return Math.Max(Fitter.MinPoints(f.Kind), f.ActivePoints.Count());
        }

        /// <summary>A fresh run of the plan for the next part.</summary>
        public static Project NewRun(Project plan, string partSerial)
        {
            var run = ProjectIO.Clone(plan);
            foreach (var f in run.Features) f.Points.Clear();
            run.PartSerial = partSerial ?? "";
            FeatureSolver.SolveAll(run);
            return run;
        }

        /// <summary>Next serial number: the trailing number is incremented, keeping its width (S009 → S010).</summary>
        public static string NextSerial(string s)
        {
            if (string.IsNullOrEmpty(s)) return "1";
            int i = s.Length;
            while (i > 0 && char.IsDigit(s[i - 1])) i--;
            if (i == s.Length) return s + "-2";
            string digits = s.Substring(i);
            long n;
            if (!long.TryParse(digits, out n)) return s + "-2";
            return s.Substring(0, i) + (n + 1).ToString().PadLeft(digits.Length, '0');
        }
    }
}
