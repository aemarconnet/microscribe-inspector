using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace MicroScribe.Export
{
    /// <summary>One row of the CSV results log (lengths converted to mm, angles in degrees; NaN = empty).</summary>
    public sealed class LogEntry
    {
        public DateTime Date;
        public string Project = "", PartNumber = "", Serial = "", Operator = "", Name = "", Characteristic = "", Unit = "", Status = "";
        public double Nominal = double.NaN, TolPlus = double.NaN, TolMinus = double.NaN, Actual = double.NaN, Deviation = double.NaN;

        /// <summary>Key that groups the same characteristic across parts.</summary>
        public string Key { get { return Name + " · " + Characteristic; } }

        /// <summary>Lower / upper specification limits (NaN when open).</summary>
        public double Lsl { get { return !double.IsNaN(Nominal) && !double.IsNaN(TolMinus) ? Nominal + TolMinus : double.NaN; } }
        public double Usl
        {
            get
            {
                if (!double.IsNaN(Nominal) && !double.IsNaN(TolPlus)) return Nominal + TolPlus;
                return double.IsNaN(Nominal) ? TolPlus : double.NaN;   // geometric tolerance: zone width ≤ tolerance
            }
        }
    }

    /// <summary>Reads the CSV written by <see cref="Results.AppendCsv"/>.</summary>
    public static class ResultsLog
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        public static List<string> SplitCsv(string line)
        {
            var r = new List<string>();
            var cur = new StringBuilder();
            bool q = false;
            foreach (char c in line)
            {
                if (c == '"') q = !q;
                else if (c == ',' && !q) { r.Add(cur.ToString()); cur.Clear(); }
                else cur.Append(c);
            }
            r.Add(cur.ToString());
            return r;
        }

        public static List<LogEntry> Read(string path)
        {
            var list = new List<LogEntry>();
            var lines = File.ReadAllLines(path);
            if (lines.Length == 0) return list;
            var head = SplitCsv(lines[0]).Select(h => h.Trim()).ToList();
            Func<List<string>, string, string> col = (row, name) => { int i = head.IndexOf(name); return i >= 0 && i < row.Count ? row[i].Trim() : ""; };
            for (int k = 1; k < lines.Length; k++)
            {
                if (string.IsNullOrWhiteSpace(lines[k]) || lines[k].StartsWith("date,")) continue;
                var row = SplitCsv(lines[k]);
                var e = new LogEntry
                {
                    Project = col(row, "project"), PartNumber = col(row, "part_number"), Serial = col(row, "serial"), Operator = col(row, "operator"),
                    Name = col(row, "name"), Characteristic = col(row, "characteristic"), Unit = col(row, "unit"), Status = col(row, "status")
                };
                DateTime.TryParse(col(row, "date"), CI, DateTimeStyles.None, out e.Date);
                double scale = e.Unit == "in" ? 25.4 : 1;
                Func<string, double> num = s => { double v; return double.TryParse(s, NumberStyles.Float, CI, out v) ? v * scale : double.NaN; };
                e.Nominal = num(col(row, "nominal")); e.TolPlus = num(col(row, "tol_plus")); e.TolMinus = num(col(row, "tol_minus"));
                e.Actual = num(col(row, "actual")); e.Deviation = num(col(row, "deviation"));
                if (e.Unit == "in") e.Unit = "mm";
                list.Add(e);
            }
            return list;
        }
    }

    /// <summary>Process statistics of one characteristic over many parts.</summary>
    public sealed class SpcStats
    {
        public int N;
        public double Mean = double.NaN, Sigma = double.NaN, Min = double.NaN, Max = double.NaN, Cp = double.NaN, Cpk = double.NaN;
        public double Lsl = double.NaN, Usl = double.NaN;
        public int OutOfTolerance;
    }

    public static class Spc
    {
        /// <summary>Mean, sample standard deviation and capability (Cp needs both limits; Cpk one or both).</summary>
        public static SpcStats Compute(IList<double> values, double lsl, double usl)
        {
            var v = values.Where(x => !double.IsNaN(x)).ToList();
            var s = new SpcStats { N = v.Count, Lsl = lsl, Usl = usl };
            if (v.Count == 0) return s;
            s.Mean = v.Average(); s.Min = v.Min(); s.Max = v.Max();
            s.OutOfTolerance = v.Count(x => (!double.IsNaN(lsl) && x < lsl - 1e-9) || (!double.IsNaN(usl) && x > usl + 1e-9));
            if (v.Count < 2) return s;
            s.Sigma = Math.Sqrt(v.Sum(x => (x - s.Mean) * (x - s.Mean)) / (v.Count - 1));
            if (s.Sigma <= 1e-15) return s;
            if (!double.IsNaN(lsl) && !double.IsNaN(usl)) s.Cp = (usl - lsl) / (6 * s.Sigma);
            double up = double.IsNaN(usl) ? double.MaxValue : (usl - s.Mean) / (3 * s.Sigma);
            double lo = double.IsNaN(lsl) ? double.MaxValue : (s.Mean - lsl) / (3 * s.Sigma);
            double k = Math.Min(up, lo);
            s.Cpk = k == double.MaxValue ? double.NaN : k;
            return s;
        }
    }
}
