using System.IO;
using System.Text;
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
    }
}
