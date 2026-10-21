using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using MicroScribe.Core;
using MicroScribe.Geometry;
using MicroScribe.Model;

namespace MicroScribe.Export
{
    /// <summary>Header information for reports and the results log.</summary>
    public sealed class ReportInfo
    {
        public string PartNumber = "", Serial = "", Operator = "", Company = "", Capability = "", Software = "";
        /// <summary>Optional logo as a data: URI (PNG/JPEG).</summary>
        public string LogoDataUri;
        public DateTime Date = DateTime.Now;
    }

    /// <summary>One reported characteristic (lengths in mm, angles in degrees).</summary>
    public sealed class ResultRow
    {
        public string Name, Characteristic;
        public bool HasNominal, IsAngle, IsZone;
        public double Nominal, TolPlus, TolMinus, Actual, Deviation;
        public TolStatus Status;
    }

    public static class Results
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        /// <summary>All measurements as rows; a position gives one row per axis (part frame).</summary>
        public static List<ResultRow> Rows(Project p)
        {
            var rows = new List<ResultRow>();
            Mat4 part = p.AlignmentMatrix;
            foreach (var d in p.Dimensions)
            {
                var r = DimensionSolver.Evaluate(p, d);
                string name = d.Name ?? d.Kind.ToString();
                if (!r.Ok) { rows.Add(new ResultRow { Name = name, Characteristic = r.Error ?? "not measurable", Actual = double.NaN, Deviation = double.NaN }); continue; }
                var t = Tolerance.Evaluate(d, r, part);
                if (d.Kind == DimKind.Coordinate)
                {
                    Vec3 q = part.TransformPoint(r.A);
                    double[] act = { q.X, q.Y, q.Z }, nom = { d.NX, d.NY, d.NZ };
                    for (int i = 0; i < 3; i++)
                    {
                        var row = new ResultRow { Name = name, Characteristic = "XYZ"[i] + " of " + DimensionSolver.RefName(p, d.Refs[0]), Actual = act[i], HasNominal = d.Toleranced, Nominal = nom[i],
                            TolPlus = d.TolPlus, TolMinus = d.TolMinus, Deviation = d.Toleranced ? act[i] - nom[i] : double.NaN };
                        row.Status = !d.Toleranced ? TolStatus.None : row.Deviation <= d.TolPlus + 1e-9 && row.Deviation >= d.TolMinus - 1e-9 ? TolStatus.Pass : TolStatus.Fail;
                        rows.Add(row);
                    }
                    continue;
                }
                bool zone = Gdt.IsGdt(d.Kind);
                rows.Add(new ResultRow
                {
                    Name = name, Characteristic = r.Title, Actual = r.Value, IsAngle = r.IsAngle, IsZone = zone,
                    HasNominal = d.Toleranced && !zone, Nominal = d.Nominal, TolPlus = d.Toleranced ? d.TolPlus : double.NaN, TolMinus = zone ? 0 : d.TolMinus,
                    Deviation = d.Toleranced && !zone ? t.Deviation : double.NaN, Status = t.Status
                });
            }
            return rows;
        }

        static string Csv(string s) { return "\"" + (s ?? "").Replace("\"", "'") + "\""; }

        /// <summary>
        /// Appends this part's results to a CSV log (one row per characteristic), writing the header when
        /// the file is new. The log collects many parts for trends and capability studies.
        /// </summary>
        public static int AppendCsv(string path, Project p, ReportInfo info, UnitFmt u)
        {
            var sb = new StringBuilder();
            if (!File.Exists(path) || new FileInfo(path).Length == 0)
                sb.AppendLine("date,project,part_number,serial,operator,name,characteristic,unit,nominal,tol_plus,tol_minus,actual,deviation,status");
            var rows = Rows(p);
            Func<double, bool, string> V = (x, ang) => double.IsNaN(x) ? "" : ang ? x.ToString("0.0000", CI) : u.L(x);
            foreach (var r in rows)
                sb.AppendLine(string.Join(",", new[]
                {
                    info.Date.ToString("yyyy-MM-dd HH:mm:ss", CI), Csv(p.Title), Csv(info.PartNumber), Csv(info.Serial), Csv(info.Operator), Csv(r.Name), Csv(r.Characteristic),
                    r.IsAngle ? "deg" : u.U, r.HasNominal ? V(r.Nominal, r.IsAngle) : "", V(r.TolPlus, r.IsAngle), r.HasNominal || r.IsZone ? V(r.TolMinus, r.IsAngle) : "",
                    V(r.Actual, r.IsAngle), V(r.Deviation, r.IsAngle), r.Status == TolStatus.None ? "" : r.Status.ToString().ToUpperInvariant()
                }));
            File.AppendAllText(path, sb.ToString());
            return rows.Count;
        }
    }

    /// <summary>Self-contained HTML inspection report (open in a browser; print to PDF from there).</summary>
    public static class HtmlReport
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        static string H(string s) { return WebUtility.HtmlEncode(s ?? ""); }

        public static string Build(Project p, UnitFmt u, ReportInfo info, IList<KeyValuePair<string, byte[]>> images = null)
        {
            var rows = Results.Rows(p);
            var tol = rows.Where(r => r.Status != TolStatus.None).ToList();
            int fail = tol.Count(r => r.Status == TolStatus.Fail);
            var sb = new StringBuilder();
            sb.Append("<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
            sb.Append("<title>Inspection report — ").Append(H(p.Title)).Append("</title><style>");
            sb.Append(@"
:root{--ink:#1f2937;--muted:#6b7280;--line:#e5e7eb;--pass:#15803d;--fail:#b91c1c;--band:#f3f4f6}
*{box-sizing:border-box}body{font-family:Segoe UI,Arial,sans-serif;color:var(--ink);margin:0;padding:24px;background:#fff;font-size:13px}
header{display:flex;align-items:center;gap:16px;border-bottom:2px solid var(--ink);padding-bottom:10px;margin-bottom:14px}
header img{max-height:56px;max-width:200px}header h1{font-size:22px;margin:0}header .sub{color:var(--muted)}
table{border-collapse:collapse;width:100%;margin:6px 0 18px}th,td{border-bottom:1px solid var(--line);padding:5px 8px;text-align:left;vertical-align:top}
th{background:var(--band);font-weight:600}td.n{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap}
.meta td:first-child{color:var(--muted);width:160px}.banner{padding:10px 14px;border-radius:6px;font-weight:600;font-size:15px;margin:10px 0 16px}
.banner.pass{background:#dcfce7;color:var(--pass)}.banner.fail{background:#fee2e2;color:var(--fail)}.banner.none{background:var(--band);color:var(--muted)}
.st{font-weight:700}.st.PASS{color:var(--pass)}.st.FAIL{color:var(--fail)}tr.FAIL td{background:#fef2f2}
h2{font-size:15px;margin:18px 0 4px}.shots img{max-width:100%;border:1px solid var(--line);border-radius:4px;margin:4px 0}
.note{color:var(--muted);font-size:12px}footer{margin-top:24px;color:var(--muted);font-size:11px;border-top:1px solid var(--line);padding-top:6px}
@media print{body{padding:0}h2{break-after:avoid}tr{break-inside:avoid}.shots img{max-height:110mm}}");
            sb.Append("</style></head><body><header>");
            if (!string.IsNullOrEmpty(info.LogoDataUri)) sb.Append("<img alt=\"\" src=\"").Append(info.LogoDataUri).Append("\">");
            sb.Append("<div><h1>Inspection report</h1><div class=\"sub\">").Append(H(p.Title));
            if (!string.IsNullOrEmpty(info.Company)) sb.Append(" · ").Append(H(info.Company));
            sb.Append("</div></div></header>");

            sb.Append("<table class=\"meta\">");
            Action<string, string> meta = (k, v) => { if (!string.IsNullOrEmpty(v)) sb.Append("<tr><td>").Append(H(k)).Append("</td><td>").Append(H(v)).Append("</td></tr>"); };
            meta("Part number", info.PartNumber);
            meta("Serial number", info.Serial);
            meta("Operator", info.Operator);
            meta("Date", info.Date.ToString("yyyy-MM-dd HH:mm", CI));
            meta("Coordinate system", p.AlignmentDescription);
            meta("Units", (u.Inch ? "inch" : "mm") + ", degrees");
            meta("Measuring capability", info.Capability);
            if (!string.IsNullOrEmpty(p.ModelPath)) meta("Nominal model", Path.GetFileName(p.ModelPath));
            sb.Append("</table>");

            if (tol.Count == 0) sb.Append("<div class=\"banner none\">No tolerances set — values only</div>");
            else if (fail == 0) sb.Append("<div class=\"banner pass\">PASS — all ").Append(tol.Count).Append(" toleranced characteristics in tolerance</div>");
            else sb.Append("<div class=\"banner fail\">FAIL — ").Append(fail).Append(" of ").Append(tol.Count).Append(" toleranced characteristics out of tolerance</div>");

            if (images != null && images.Count > 0)
            {
                sb.Append("<h2>Views</h2><div class=\"shots\">");
                foreach (var im in images)
                    sb.Append("<figure style=\"margin:0 0 8px\"><img alt=\"").Append(H(im.Key)).Append("\" src=\"data:image/png;base64,").Append(Convert.ToBase64String(im.Value))
                      .Append("\"><figcaption class=\"note\">").Append(H(im.Key)).Append("</figcaption></figure>");
                sb.Append("</div>");
            }

            Func<double, bool, string> V = (x, ang) => double.IsNaN(x) ? "" : ang ? x.ToString("0.000", CI) + "°" : u.L(x);
            Func<double, bool, string> S = (x, ang) => double.IsNaN(x) ? "" : (x >= 0 ? "+" : "−") + V(Math.Abs(x), ang);
            if (rows.Count > 0)
            {
                sb.Append("<h2>Results</h2><table><tr><th>Name</th><th>Characteristic</th><th class=\"n\">Nominal</th><th class=\"n\">Tolerance</th><th class=\"n\">Actual</th><th class=\"n\">Deviation</th><th>Status</th></tr>");
                foreach (var r in rows)
                {
                    string band = r.IsZone ? (double.IsNaN(r.TolPlus) ? "" : V(r.TolPlus, false))
                                : r.HasNominal ? (Math.Abs(r.TolPlus + r.TolMinus) < 1e-12 ? "±" + V(r.TolPlus, r.IsAngle) : S(r.TolPlus, r.IsAngle) + " / −" + V(-r.TolMinus, r.IsAngle)) : "";
                    string st = r.Status == TolStatus.None ? "" : r.Status.ToString().ToUpperInvariant();
                    sb.Append("<tr class=\"").Append(st).Append("\"><td>").Append(H(r.Name)).Append("</td><td>").Append(H(r.Characteristic)).Append("</td><td class=\"n\">")
                      .Append(r.HasNominal ? V(r.Nominal, r.IsAngle) : "").Append("</td><td class=\"n\">").Append(H(band)).Append("</td><td class=\"n\">").Append(V(r.Actual, r.IsAngle))
                      .Append("</td><td class=\"n\">").Append(S(r.Deviation, r.IsAngle)).Append("</td><td class=\"st ").Append(st).Append("\">").Append(st).Append("</td></tr>");
                }
                sb.Append("</table><p class=\"note\">Geometric tolerances (form, orientation, position, profile) show the zone width as the actual value. Lengths in ")
                  .Append(u.Inch ? "inches" : "millimetres").Append(".</p>");
            }

            sb.Append("<h2>Features</h2><table><tr><th>Feature</th><th>Type</th><th class=\"n\">Points</th><th>Result</th></tr>");
            Mat4 m = p.AlignmentMatrix;
            foreach (var f in p.Features)
            {
                var lines = CadExport.Describe(GeoUtil.Transform(f.Geometry, m), f, u).ToList();
                sb.Append("<tr><td>").Append(H(f.Name)).Append("</td><td>").Append(H(f.Kind == FeatureKind.Surface ? "Freeform" : f.Kind.ToString()))
                  .Append(f.Origin == FeatureOrigin.Constructed ? " (constructed)" : "").Append("</td><td class=\"n\">").Append(f.ActivePoints.Count())
                  .Append("</td><td>").Append(string.Join("<br>", lines.Select(H))).Append("</td></tr>");
            }
            sb.Append("</table>");
            if (!string.IsNullOrEmpty(p.Notes)) sb.Append("<h2>Notes</h2><p>").Append(H(p.Notes).Replace("\n", "<br>")).Append("</p>");
            sb.Append("<footer>").Append(H(string.IsNullOrEmpty(info.Software) ? "MicroScribe Inspector" : info.Software)).Append(" · ")
              .Append(info.Date.ToString("yyyy-MM-dd HH:mm", CI)).Append("</footer></body></html>");
            return sb.ToString();
        }
    }
}
