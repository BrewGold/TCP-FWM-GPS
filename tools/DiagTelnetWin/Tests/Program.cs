using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using DiagTelnetWin;

static void Check(bool condition, string description)
{
    if (!condition)
        throw new InvalidOperationException("Prueba fallida: " + description);
    Console.WriteLine("Correcto: " + description);
}

const string rev4 = """
    ========== STATUS ==========
    GNSS: OK
    GGA entrada fix: 5
    HAS: ACTIVO
    Solucion: HAS
    Salida COM2:
    Fix salida: 4
    ============================
    Servidor diagnostico TCP 15920: LISTEN
    """;
DeviceStatus? parsed = StatusParser.Parse(rev4.Replace("\n", "\r\n"));
Check(parsed is { Gnss: "OK", FixIn: 5, Has: "ACTIVO", Solution: "HAS" }, "formato Rev.4; no confundir fix de entrada y salida");
Check(parsed!.MissingKeys.Length == 0, "claves completas");
const string iri = """
    ========== STATUS ==========
    Producto: TCP-IRI-GPS
    GNSS: OK
    GGA entrada fix: 5
    HAS: ACTIVO
    Solucion: HAS
    LAT: 40.12345678
    LON: -3.45678901
    ALT: 650.00 m
    Velocidad: 12.345 m/s
    IMU: OFF (sin magnetometro)
    Estado: MOVING (GNSS directo, sin promedio ni offset)
    Fix salida: 5
    ============================
    """;
Check(StatusParser.Parse(iri) is { Gnss: "OK", FixIn: 5, Has: "ACTIVO", Solution: "HAS" },
    "TCP-IRI-GPS con identidad y velocidad mantiene compatibilidad status");
Check(StatusParser.Parse(iri.Replace("12.345", "nan"))?.MissingKeys.Length == 0,
    "velocidad no disponible en IRI no invalida las claves GNSS");
Check(StatusParser.Parse("""
    === ESTADO ACTUAL ===
    Estado GNSS: OK
    HAS activo: SI
    ===================
    """)?.MissingKeys.SequenceEqual(new[] { "FIX_IN", "SOL" }) == true, "formato anterior con campos ausentes");
Check(StatusParser.Parse("gnss=ok FIX_IN=4 has=off solution=rtk") is { FixIn: 4, Solution: "RTK" }, "pares, alias y mayúsculas");
Check(StatusParser.Parse("Estado GNSS: FAIL\nFIX_IN: 0\nHAS: NO\nSolución: NOFIX") is { Gnss: "FAIL", FixIn: 0, Solution: "NOFIX" }, "acentos y estado sin fijación");
Check(StatusParser.Parse("GNSS=??? FIX_IN=abc HAS=??? SOL=???")?.MissingKeys.Length == 4, "valores malformados");
Check(StatusParser.Parse("GNSS=OK FIX_IN=99 HAS=ON SOL=HAS")?.FixIn is null, "fix fuera de rango");
Check(StatusParser.Parse("basura\nsin campos\n\n") is null, "líneas no reconocidas");
Check(StatusParser.Parse(rev4[..rev4.IndexOf("Salida", StringComparison.Ordinal)]) is null, "respuesta enmarcada truncada");
Check(StatusParser.Parse("[DIAG]\nGNSS=OK FIX_IN=5 HAS=ON SOL=HAS") is null, "diagnóstico espontáneo no reemplaza status");
Check(StatusParser.Parse("[DIAG]\nGNSS=FAIL FIX_IN=0 HAS=OFF SOL=NOFIX\n" + rev4)?.Gnss == "OK", "status no mezcla el diagnóstico espontáneo");
Check(StatusParser.Parse("=== ESTADO ACTUAL ===\nlinea malformada\n===================")?.MissingKeys.Length == 4, "bloque vacío señala todas las claves ausentes");
Check(StatusParser.Parse(new string('x', 65537)) is null, "límite del parser");

Check(StatusPresentation.Gnss("OK").Level == SignalLevel.Green
    && StatusPresentation.Gnss("FAIL").Level == SignalLevel.Red
    && StatusPresentation.Gnss(null).Level == SignalLevel.Yellow, "semáforo GNSS y dato ausente");
Check(StatusPresentation.Fix(0).Level == SignalLevel.Red
    && StatusPresentation.Fix(4).Level == SignalLevel.Green, "semáforo FIX_IN sin fijación y RTK fijo");
foreach (int fix in new[] { 1, 2, 3, 5, 6, 7, 8 })
    Check(StatusPresentation.Fix(fix).Level == SignalLevel.Yellow, $"FIX_IN {fix} no se presenta como RTK fijo");
Check(StatusPresentation.Fix(null).Level == SignalLevel.Yellow, "FIX_IN ausente no implica sin fijación");
foreach (string active in new[] { "ON", "SI", "ACTIVO", "TRUE", "1" })
    Check(StatusPresentation.Has(active).Level == SignalLevel.Green, $"HAS activo: {active}");
foreach (string inactive in new[] { "OFF", "NO", "INACTIVO", "FALSE", "0" })
    Check(StatusPresentation.Has(inactive) is { Text: "Inactivo", Level: SignalLevel.Yellow }, $"HAS inactivo: {inactive}");
Check(StatusPresentation.Has(null).Text == "No disponible", "HAS ausente no implica inactivo");
foreach (string solution in new[] { "HAS", "RTK_FIX" })
    Check(StatusPresentation.Solution(solution).Level == SignalLevel.Green, $"solución precisa: {solution}");
foreach (string solution in new[] { "GPS", "DGPS", "RTK", "RTK_FLOAT", "UNKNOWN" })
    Check(StatusPresentation.Solution(solution).Level == SignalLevel.Yellow, $"solución con atención: {solution}");
Check(StatusPresentation.Solution("NOFIX").Level == SignalLevel.Red
    && StatusPresentation.Solution(null).Level == SignalLevel.Yellow, "solución ausente distinta de sin solución");
var good = StatusPresentation.Gnss("OK");
Check(StatusPresentation.Freshness(good, false) is { Text: "Correcto", Level: SignalLevel.Yellow }
    && StatusPresentation.Freshness(good, true) == good, "lectura retenida no muestra verde sin datos actualizados");
Check(parsed is { Gnss: "OK", FixIn: 5, Has: "ACTIVO", Solution: "HAS" },
    "mapeo visual no modifica el estado recibido");

var filter = new StatusResponseFilter();
Check(StatusParser.Parse(filter.Append(rev4 + "\r\n[DIA"))?.FixIn == 5, "status seguido de cabecera espontánea incompleta");
Check(StatusParser.Parse(filter.Append("G]\r\nGNSS=OK FIX_IN=5 HAS=ON SOL=HAS\r\n")) is null, "fragmento espontáneo entre consultas no se acepta como status");
Check(StatusParser.Parse(filter.Append(rev4 + "\r\n"))?.FixIn == 5, "nueva cabecera status recupera el flujo");

var decoder = new TelnetDecoder();
Check(decoder.Decode(new byte[] { 255 }).Text == "", "IAC fragmentado");
Check(decoder.Decode(new byte[] { 251 }).Reply.Length == 0, "verbo fragmentado");
Check(decoder.Decode(new byte[] { 1 }).Reply.SequenceEqual(new byte[] { 255, 254, 1 }), "rechazar WILL con DONT");
Check(decoder.Decode(new byte[] { 255, 253, 3 }).Reply.SequenceEqual(new byte[] { 255, 252, 3 }), "rechazar DO con WONT");
Check(decoder.Decode(new byte[] { 255, 252, 3, 255, 254, 1 }).Reply.Length == 0, "no provocar bucle de negociación");
Check(decoder.Decode(new byte[] { 255, 250, 1, 2, 255 }).Text == "", "subnegociación fragmentada");
Check(decoder.Decode(new byte[] { 240, (byte)'G' }).Text == "G", "fin de subnegociación");
byte[] accent = Encoding.UTF8.GetBytes("ó");
Check(decoder.Decode(accent.AsSpan(0, 1)).Text == "" && decoder.Decode(accent.AsSpan(1)).Text == "ó", "UTF-8 fragmentado");

using var deadline = new CancellationTokenSource(TimeSpan.FromSeconds(20));
var listener = new TcpListener(IPAddress.Loopback, 0);
listener.Start();
try
{
    int port = ((IPEndPoint)listener.LocalEndpoint).Port;
    using var session = new TelnetSession();
    Task<TcpClient> accepting = listener.AcceptTcpClientAsync(deadline.Token).AsTask();
    await session.ConnectAsync(IPAddress.Loopback, port, deadline.Token);
    using TcpClient peer = await accepting;
    NetworkStream stream = peer.GetStream();
    var reader = new StreamReader(stream, Encoding.UTF8, leaveOpen: true);
    Task<PollResult> polling = session.PollAsync(deadline.Token);
    Check(await reader.ReadLineAsync(deadline.Token) == "status", "único comando enviado: status");
    byte[] response = Encoding.UTF8.GetBytes(rev4 + "\r\n");
    await stream.WriteAsync(response.AsMemory(0, 25), deadline.Token);
    await Task.Delay(350, deadline.Token);
    await stream.WriteAsync(response.AsMemory(25), deadline.Token);
    Check((await polling).Status?.FixIn == 5, "respuesta TCP fragmentada con pausa");

    polling = session.PollAsync(deadline.Token);
    Check(await reader.ReadLineAsync(deadline.Token) == "status", "consulta IRI conserva protocolo");
    await stream.WriteAsync(Encoding.UTF8.GetBytes(iri + "\r\n"), deadline.Token);
    PollResult iriResult = await polling;
    Check(iriResult.Status?.FixIn == 5 && iriResult.Raw.Contains("Velocidad: 12.345 m/s"),
        "Telnet IRI conserva velocidad en el registro de diagnóstico");

    polling = session.PollAsync(deadline.Token);
    Check(await reader.ReadLineAsync(deadline.Token) == "status", "segunda consulta sigue siendo solo status");
    await stream.WriteAsync("GNSS=OK HAS=ON\r\n"u8.ToArray(), deadline.Token);
    Check((await polling).Status?.MissingKeys.Length == 2, "respuesta TCP parcial");

    polling = session.PollAsync(deadline.Token);
    await reader.ReadLineAsync(deadline.Token);
    await stream.WriteAsync(Encoding.UTF8.GetBytes(rev4 + "\r\n[DIAG]\r\n"), deadline.Token);
    Check((await polling).Status?.FixIn == 5, "status válido antes de diagnóstico fragmentado");
    polling = session.PollAsync(deadline.Token);
    await reader.ReadLineAsync(deadline.Token);
    await stream.WriteAsync("GNSS=OK FIX_IN=5 HAS=ON SOL=HAS\r\n"u8.ToArray(), deadline.Token);
    Check((await polling).Status is null, "cuerpo espontáneo recibido en la siguiente consulta no renueva el estado");

    polling = session.PollAsync(deadline.Token);
    await reader.ReadLineAsync(deadline.Token);
    await stream.WriteAsync(Encoding.UTF8.GetBytes(rev4 + "\r\n"), deadline.Token);
    Check((await polling).Status?.FixIn == 5, "recuperación de status enmarcado tras fragmento espontáneo");

    var clock = Stopwatch.StartNew();
    polling = session.PollAsync(deadline.Token);
    await reader.ReadLineAsync(deadline.Token);
    await stream.WriteAsync("[DIAG]\r\nGNSS=OK FIX_IN=5 HAS=ON SOL=HAS\r\n"u8.ToArray(), deadline.Token);
    Check((await polling).Status is null && clock.ElapsedMilliseconds >= 2800, "timeout incluso con diagnóstico espontáneo");

    using var cancel = new CancellationTokenSource();
    polling = session.PollAsync(cancel.Token);
    await reader.ReadLineAsync(deadline.Token);
    cancel.Cancel();
    bool cancelled = false;
    try { await polling; }
    catch (OperationCanceledException) { cancelled = true; }
    Check(cancelled, "cancelación durante lectura pendiente");

    polling = session.PollAsync(deadline.Token);
    await reader.ReadLineAsync(deadline.Token);
    peer.Close();
    bool disconnected = false;
    try { await polling; }
    catch (IOException) { disconnected = true; }
    Check(disconnected, "cierre remoto detectado");
}
finally { listener.Stop(); }
Console.WriteLine("Todas las pruebas de protocolo finalizaron correctamente.");
