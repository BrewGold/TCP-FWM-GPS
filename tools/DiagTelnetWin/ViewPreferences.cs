using System.Text.Json;

namespace DiagTelnetWin;

public static class ViewPreferences
{
    public static string DefaultPath => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
        "DiagTelnetWin", "vista.json");

    public static bool LoadCompact(string path)
    {
        try
        {
            if (!File.Exists(path) || new FileInfo(path).Length > 1024)
                return true;
            using JsonDocument document = JsonDocument.Parse(File.ReadAllText(path));
            return document.RootElement.ValueKind != JsonValueKind.Object ||
                !document.RootElement.TryGetProperty("Compacto", out JsonElement value) ||
                value.ValueKind != JsonValueKind.False;
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException)
        {
            return true;
        }
    }

    public static bool SaveCompact(string path, bool compact)
    {
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
            File.WriteAllText(path, JsonSerializer.Serialize(new { Compacto = compact }));
            return true;
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            return false;
        }
    }
}
