using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;

namespace DiagTelnetWin;

public sealed record DeviceStatus(string? Gnss, int? FixIn, string? Has, string? Solution)
{
    public string[] MissingKeys => new[]
    {
        Gnss is null ? "GNSS" : null,
        FixIn is null ? "FIX_IN" : null,
        Has is null ? "HAS" : null,
        Solution is null ? "SOL" : null
    }.OfType<string>().ToArray();
}

public static class StatusParser
{
    private static readonly Regex Pairs = new(
        @"\b(GNSS|FIX_IN|FIX|HAS|SOL|SOLUTION)\s*=\s*([^\s;=]+)",
        RegexOptions.IgnoreCase | RegexOptions.CultureInvariant,
        TimeSpan.FromMilliseconds(100));
    private static readonly Regex Header = new(
        @"^=+[ \t]*(?:ESTADO[^\r\n]*|STATUS)[ \t]*=+[ \t]*\r?$",
        RegexOptions.Multiline | RegexOptions.CultureInvariant,
        TimeSpan.FromMilliseconds(100));

    internal static bool IsStatusHeader(string line) => Header.IsMatch(Normalize(line));

    public static DeviceStatus? Parse(string text)
    {
        if (text.Length > 65536)
            return null;

        string normalized = Normalize(text);
        Match header = Header.Match(normalized);
        if (header.Success)
        {
            int start = normalized.IndexOf('\n', header.Index + header.Length);
            if (start < 0)
                return null;
            int end = normalized.IndexOf("===", start);
            // No aceptar un bloque status truncado como una respuesta completa.
            if (end < 0)
                return null;
            normalized = normalized[(start + 1)..end];
        }
        else if (normalized.Contains("[DIAG]", StringComparison.Ordinal))
        {
            return null;
        }

        var values = new Dictionary<string, string>();
        foreach (string line in normalized.Split('\n'))
        {
            foreach (Match match in Pairs.Matches(line))
                values[Canonical(match.Groups[1].Value)] = match.Groups[2].Value;

            int colon = line.IndexOf(':');
            if (colon < 0)
                continue;
            string key = Canonical(line[..colon].Trim());
            if (key is "GNSS" or "FIX_IN" or "HAS" or "SOL")
                values[key] = line[(colon + 1)..].Trim();
        }

        string? gnss = GetKnown(values, "GNSS", "OK", "FAIL", "KO", "UP", "DOWN");
        string? has = GetKnown(values, "HAS", "ON", "OFF", "SI", "NO", "ACTIVO", "INACTIVO", "TRUE", "FALSE", "1", "0");
        string? solution = GetKnown(values, "SOL", "NOFIX", "GPS", "DGPS", "RTK", "RTK_FLOAT", "RTK_FIX", "HAS", "UNKNOWN");
        int? fix = values.TryGetValue("FIX_IN", out string? number)
            && int.TryParse(number, NumberStyles.None, CultureInfo.InvariantCulture, out int parsed)
            && parsed is >= 0 and <= 8 ? parsed : null;
        return values.Count == 0 && !header.Success ? null : new DeviceStatus(gnss, fix, has, solution);
    }

    private static string Canonical(string key) => key switch
    {
        "ESTADO GNSS" => "GNSS",
        "FIX" or "GGA ENTRADA FIX" => "FIX_IN",
        "HAS ACTIVO" => "HAS",
        "SOLUTION" or "SOLUCION" or "TIPO DE SOLUCION" => "SOL",
        _ => key
    };

    private static string? GetKnown(Dictionary<string, string> values, string key, params string[] known) =>
        values.TryGetValue(key, out string? value) && known.Contains(value) ? value : null;

    private static string Normalize(string text)
    {
        var result = new StringBuilder(text.Length);
        foreach (char c in text.Normalize(NormalizationForm.FormD))
        {
            if (CharUnicodeInfo.GetUnicodeCategory(c) != UnicodeCategory.NonSpacingMark)
                result.Append(char.ToUpperInvariant(c));
        }
        return result.ToString();
    }
}
