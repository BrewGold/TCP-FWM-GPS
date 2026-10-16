namespace DiagTelnetWin;

public enum SignalLevel { Green, Yellow, Red }

public sealed record StatusVisual(string Text, SignalLevel Level);

public static class StatusPresentation
{
    public static StatusVisual Gnss(string? value) => value switch
    {
        "OK" or "UP" => new("Correcto", SignalLevel.Green),
        "FAIL" or "KO" or "DOWN" => new("Fallo", SignalLevel.Red),
        _ => new("No disponible", SignalLevel.Yellow)
    };

    public static StatusVisual Fix(int? value) => value switch
    {
        0 => new("0 · Sin fijación", SignalLevel.Red),
        4 => new("4 · RTK fijo", SignalLevel.Green),
        1 => new("1 · GPS", SignalLevel.Yellow),
        2 => new("2 · DGPS", SignalLevel.Yellow),
        3 => new("3 · PPS", SignalLevel.Yellow),
        5 => new("5 · RTK flotante", SignalLevel.Yellow),
        6 => new("6 · Estimado", SignalLevel.Yellow),
        7 => new("7 · Manual", SignalLevel.Yellow),
        8 => new("8 · Simulado", SignalLevel.Yellow),
        _ => new("No disponible", SignalLevel.Yellow)
    };

    public static StatusVisual Has(string? value) => value switch
    {
        "ON" or "SI" or "ACTIVO" or "TRUE" or "1" => new("Activo", SignalLevel.Green),
        "OFF" or "NO" or "INACTIVO" or "FALSE" or "0" => new("Inactivo", SignalLevel.Yellow),
        _ => new("No disponible", SignalLevel.Yellow)
    };

    public static StatusVisual Solution(string? value) => value switch
    {
        "NOFIX" => new("Sin solución", SignalLevel.Red),
        "UNKNOWN" => new("Desconocida", SignalLevel.Yellow),
        "RTK_FLOAT" => new("RTK flotante", SignalLevel.Yellow),
        "RTK_FIX" => new("RTK fija", SignalLevel.Green),
        "HAS" => new("HAS", SignalLevel.Green),
        "GPS" or "DGPS" or "RTK" => new(value, SignalLevel.Yellow),
        _ => new("No disponible", SignalLevel.Yellow)
    };

    public static StatusVisual Freshness(StatusVisual visual, bool live) =>
        live ? visual : visual with { Level = SignalLevel.Yellow };
}
