using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;

namespace DiagTelnetWin;

public sealed class Form1 : Form
{
    private readonly TextBox deviceIp = new() { Text = "192.168.1.22", Width = 140 };
    private readonly NumericUpDown port = new() { Minimum = 1, Maximum = 65535, Value = 15920, Width = 85 };
    private readonly NumericUpDown interval = new() { Minimum = 250, Maximum = 60000, Value = 1000, Width = 85 };
    private readonly Button connect = new() { Text = "Conectar", AutoSize = true };
    private readonly Button disconnect = new() { Text = "Desconectar", AutoSize = true, Enabled = false };
    private readonly Button export = new() { Text = "Exportar diagnóstico", AutoSize = true };
    private readonly Label connection = new() { Text = "Desconectado", AutoSize = true };
    private readonly Label gnss = new() { AutoSize = true };
    private readonly Label fix = new() { AutoSize = true };
    private readonly Label has = new() { AutoSize = true };
    private readonly Label solution = new() { AutoSize = true };
    private readonly Label age = new() { AutoSize = true };
    private readonly Label lastResponse = new() { AutoSize = true };
    private readonly Label pollInterval = new() { AutoSize = true };
    private readonly Label alerts = new() { AutoSize = true, Dock = DockStyle.Fill };
    private readonly Label alarmCount = new() { AutoSize = true };
    private readonly Dictionary<Label, Label> indicators = new();
    private readonly RichTextBox logView = new()
    {
        ReadOnly = true, Dock = DockStyle.Fill, BackColor = Color.FromArgb(20, 29, 45),
        ForeColor = Color.FromArgb(222, 232, 244), BorderStyle = BorderStyle.None,
        Font = new Font("Consolas", 9F), DetectUrls = false
    };
    private readonly System.Windows.Forms.Timer timer = new() { Interval = 250 };
    private readonly Queue<LogEntry> logs = new();
    private CancellationTokenSource? cancellation;
    private Task? sessionTask;
    private Task? exportTask;
    private DeviceStatus? latest;
    private DateTimeOffset? latestAt;
    private long? latestTick;
    private long startedTick;
    private string responseAlert = "";
    private string lastAlert = "";
    private bool connected;
    private bool closing;
    private int pollMs = 1000;
    private string? sessionIp;
    private int sessionPort;

    private sealed record LogEntry(DateTimeOffset Fecha, string Mensaje);

    public Form1()
    {
        Text = "Diagnóstico GPS por Telnet — sin COM2";
        MinimumSize = new Size(1040, 760);
        Size = new Size(1180, 900);
        Font = new Font("Segoe UI", 10F);
        BackColor = Color.FromArgb(239, 243, 249);
        ForeColor = Color.FromArgb(30, 45, 65);
        AutoScaleMode = AutoScaleMode.Font;
        var layout = new TableLayoutPanel { Dock = DockStyle.Fill, Padding = new Padding(16), ColumnCount = 1, RowCount = 5 };
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 60));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 40));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        var heading = new Label
        {
            Text = "GPS / Diagnóstico en vivo", AutoSize = true,
            Font = new Font(Font, FontStyle.Bold), Margin = new Padding(3, 0, 3, 12)
        };
        layout.Controls.Add(heading, 0, 0);
        var settings = new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Fill, WrapContents = true };
        settings.Controls.AddRange(new Control[]
        {
            new Label { Text = "IP del dispositivo", AutoSize = true }, deviceIp,
            new Label { Text = "Puerto Telnet", AutoSize = true }, port,
            new Label { Text = "Intervalo (ms)", AutoSize = true }, interval, connect, disconnect
        });
        layout.Controls.Add(settings, 0, 1);
        var dashboard = new TableLayoutPanel
        {
            AutoSize = true, Dock = DockStyle.Top, ColumnCount = 1,
            Padding = new Padding(0, 8, 0, 8)
        };
        var cards = new TableLayoutPanel { AutoSize = true, Dock = DockStyle.Top, ColumnCount = 5 };
        string[] titles = { "GNSS", "FIX_IN", "HAS", "Tipo de solución", "Telnet" };
        Label[] values = { gnss, fix, has, solution, connection };
        for (int i = 0; i < values.Length; i++)
        {
            cards.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 20));
            cards.Controls.Add(CreateStatusCard(titles[i], values[i]), i, 0);
        }
        dashboard.Controls.Add(cards, 0, 0);
        var details = new TableLayoutPanel { AutoSize = true, Dock = DockStyle.Top, ColumnCount = 2 };
        details.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55));
        details.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 45));
        var position = new TableLayoutPanel { AutoSize = true, Dock = DockStyle.Top, ColumnCount = 2 };
        AddField(position, "Latitud / Longitud", new Label { Text = "No disponible", AutoSize = true });
        AddField(position, "Altitud", new Label { Text = "No disponible", AutoSize = true });
        var positionNote = new Label
        {
            Text = "El estado actual no expone coordenadas ni altitud. No se infieren del FIX.",
            AutoSize = true, MaximumSize = new Size(400, 0), ForeColor = Color.DimGray
        };
        AddField(position, "Origen", positionNote);
        AddField(position, "Última respuesta válida (UTC)", lastResponse);
        AddField(position, "Edad de respuesta Telnet", age);
        AddField(position, "Intervalo de consulta", pollInterval);
        details.Controls.Add(CreatePanel("Posición y tiempos", position), 0, 0);
        var alarms = new TableLayoutPanel { AutoSize = true, Dock = DockStyle.Top, ColumnCount = 1 };
        alarms.Controls.Add(alarmCount);
        alarms.Controls.Add(alerts);
        details.Controls.Add(CreatePanel("Alarmas activas", alarms), 1, 0);
        dashboard.Controls.Add(details, 0, 1);
        var scroll = new Panel { Dock = DockStyle.Fill, AutoScroll = true };
        scroll.Controls.Add(dashboard);
        layout.Controls.Add(scroll, 0, 2);
        layout.Controls.Add(CreatePanel("Consola / registro Telnet", logView), 0, 3);
        var footer = new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Fill };
        footer.Controls.Add(export);
        footer.Controls.Add(new Label { Text = "Solo Telnet. COM2 queda reservado al diagnóstico directo del usuario.", AutoSize = true });
        layout.Controls.Add(footer, 0, 4);
        Controls.Add(layout);
        connect.Click += async (_, _) => await StartAsync();
        disconnect.Click += async (_, _) => await StopAsync();
        export.Click += async (_, _) =>
        {
            exportTask = ExportAsync();
            await exportTask;
        };
        timer.Tick += (_, _) => RefreshDashboard();
        timer.Start();
        RefreshDashboard();
    }

    private static void AddField(TableLayoutPanel panel, string title, Label value)
    {
        int row = panel.RowCount++;
        value.MaximumSize = new Size(250, 0);
        panel.Controls.Add(new Label { Text = title, AutoSize = true, MaximumSize = new Size(180, 0), Margin = new Padding(4, 5, 20, 5) }, 0, row);
        panel.Controls.Add(value, 1, row);
    }

    private static GroupBox CreatePanel(string title, Control content)
    {
        var panel = new GroupBox
        {
            Text = title, Dock = DockStyle.Fill, AutoSize = true,
            BackColor = Color.White, Padding = new Padding(12), Margin = new Padding(4, 8, 4, 8)
        };
        panel.Controls.Add(content);
        return panel;
    }

    private Control CreateStatusCard(string title, Label value)
    {
        var card = new TableLayoutPanel
        {
            AutoSize = true, Dock = DockStyle.Fill, ColumnCount = 1,
            BackColor = Color.White, Padding = new Padding(12), Margin = new Padding(4),
            MinimumSize = new Size(0, 125)
        };
        card.Controls.Add(new Label { Text = title, AutoSize = true, ForeColor = Color.DimGray });
        value.Font = new Font(Font, FontStyle.Bold);
        value.MaximumSize = new Size(165, 0);
        value.Margin = new Padding(0, 8, 0, 8);
        card.Controls.Add(value);
        var indicator = new Label { AutoSize = true, MaximumSize = new Size(165, 0) };
        card.Controls.Add(indicator);
        indicators.Add(value, indicator);
        return card;
    }

    private void ShowSignal(Label value, StatusVisual visual, bool live = true)
    {
        value.Text = visual.Text;
        SignalLevel level = StatusPresentation.Freshness(visual, live).Level;
        Color color = level switch
        {
            SignalLevel.Green => Color.FromArgb(22, 110, 60),
            SignalLevel.Red => Color.FromArgb(173, 38, 45),
            _ => Color.FromArgb(130, 86, 0)
        };
        Label indicator = indicators[value];
        indicator.Text = !live ? "● Sin actualizar" : level switch
        {
            SignalLevel.Green => "● Correcto",
            SignalLevel.Red => "● Alarma",
            _ => "● Atención"
        };
        indicator.ForeColor = value.ForeColor = color;
    }

    private async Task StartAsync()
    {
        if (sessionTask is not null)
            return;
        if (!IPAddress.TryParse(deviceIp.Text.Trim(), out IPAddress? address))
        {
            MessageBox.Show(this, "Introduce una dirección IP válida.", "Configuración", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }
        sessionIp = address.ToString();
        sessionPort = (int)port.Value;
        pollMs = (int)interval.Value;
        latest = null;
        latestAt = null;
        latestTick = null;
        responseAlert = "";
        cancellation = new CancellationTokenSource();
        SetRunning(true);
        connection.Text = "Conectando…";
        sessionTask = RunSessionAsync(address, sessionPort, cancellation.Token);
        try { await sessionTask; }
        finally
        {
            sessionTask = null;
            cancellation.Dispose();
            cancellation = null;
            SetRunning(false);
            RefreshDashboard();
        }
    }

    private async Task RunSessionAsync(IPAddress address, int telnetPort, CancellationToken token)
    {
        using var session = new TelnetSession();
        try
        {
            AddLog($"Conectando a {address}:{telnetPort} por Telnet.");
            await session.ConnectAsync(address, telnetPort, token);
            connected = true;
            startedTick = Stopwatch.GetTimestamp();
            connection.Text = "Conectado";
            AddLog("Telnet conectado. Solo se solicita status.");
            while (true)
            {
                token.ThrowIfCancellationRequested();
                var cycle = Stopwatch.StartNew();
                PollResult result = await session.PollAsync(token);
                if (!string.IsNullOrWhiteSpace(result.Raw))
                    AddLog("Respuesta Telnet:\n" + result.Raw.Trim());
                if (result.Status is null)
                {
                    responseAlert = "Tiempo de espera de status agotado o respuesta no válida.";
                }
                else
                {
                    latest = result.Status;
                    string[] missing = latest.MissingKeys;
                    responseAlert = missing.Length > 0
                        ? "Faltan claves esperadas o contienen valores no válidos: " + string.Join(", ", missing)
                        : "";
                    if (missing.Length == 0)
                    {
                        latestAt = DateTimeOffset.UtcNow;
                        latestTick = Stopwatch.GetTimestamp();
                    }
                }
                RefreshDashboard();
                int delay = pollMs - (int)cycle.ElapsedMilliseconds;
                if (delay > 0)
                    await Task.Delay(delay, token);
            }
        }
        catch (OperationCanceledException) when (token.IsCancellationRequested)
        {
            AddLog("Desconexión solicitada por el usuario.");
        }
        catch (OperationCanceledException)
        {
            AddLog("No se pudo conectar: tiempo de espera agotado.");
        }
        catch (SocketException)
        {
            AddLog("Error de red Telnet. Comprueba la IP, el puerto y la disponibilidad del dispositivo.");
        }
        catch (IOException)
        {
            AddLog("Telnet desconectado: conexión cerrada, respuesta demasiado grande o error de lectura/escritura.");
        }
        catch (Exception)
        {
            AddLog("No se pudo continuar el diagnóstico. Desconecta y vuelve a conectar.");
        }
        finally
        {
            connected = false;
            connection.Text = "Desconectado";
        }
    }

    private async Task StopAsync()
    {
        cancellation?.Cancel();
        if (sessionTask is not null)
            await sessionTask;
    }

    private void SetRunning(bool running)
    {
        connect.Enabled = !running;
        disconnect.Enabled = running;
        deviceIp.Enabled = port.Enabled = interval.Enabled = !running;
    }

    private void RefreshDashboard()
    {
        double? elapsed = latestTick is long tick ? Stopwatch.GetElapsedTime(tick).TotalMilliseconds : null;
        bool live = connected && elapsed.HasValue && elapsed.Value <= StaleThresholdMs && responseAlert.Length == 0;
        ShowSignal(gnss, StatusPresentation.Gnss(latest?.Gnss), live);
        ShowSignal(fix, StatusPresentation.Fix(latest?.FixIn), live);
        ShowSignal(has, StatusPresentation.Has(latest?.Has), live);
        ShowSignal(solution, StatusPresentation.Solution(latest?.Solution), live);
        ShowSignal(connection, new StatusVisual(connection.Text,
            connected ? (live ? SignalLevel.Green : SignalLevel.Yellow)
                : connection.Text == "Conectando…" ? SignalLevel.Yellow : SignalLevel.Red));
        age.Text = elapsed.HasValue ? $"{elapsed.Value / 1000:F1} s" : "Sin respuesta válida completa";
        lastResponse.Text = latestAt?.ToString("yyyy-MM-dd HH:mm:ss 'UTC'", CultureInfo.InvariantCulture) ?? "No disponible";
        pollInterval.Text = $"{(sessionTask is null ? (int)interval.Value : pollMs)} ms";
        var warnings = new List<string>();
        if (!connected && connection.Text != "Conectando…")
            warnings.Add("Telnet desconectado.");
        if (connected)
        {
            if (latest?.Gnss is "FAIL" or "KO" or "DOWN")
                warnings.Add("GNSS en fallo.");
            if (latest?.FixIn == 0)
                warnings.Add("FIX_IN sin fijación.");
            if (StatusPresentation.Has(latest?.Has).Text == "Inactivo")
                warnings.Add("HAS inactivo.");
            if (latest?.Solution == "NOFIX")
                warnings.Add("Sin solución de posición.");
            if (responseAlert.Length > 0)
                warnings.Add(responseAlert);
            double silence = elapsed ?? Stopwatch.GetElapsedTime(startedTick).TotalMilliseconds;
            if (silence > StaleThresholdMs)
                warnings.Add("Datos obsoletos: no se recibe un estado completo actualizado.");
        }
        string message = string.Join(" · ", warnings);
        alarmCount.Text = warnings.Count == 0 ? "Sin alarmas activas" : $"{warnings.Count} aviso(s) · Revisar estado y frescura";
        alerts.Text = message.Length == 0 ? "Sin alertas." : string.Join(Environment.NewLine + Environment.NewLine, warnings.Select(w => "• " + w));
        alerts.MaximumSize = new Size(Math.Max(180, alerts.Parent?.ClientSize.Width - 12 ?? 360), 0);
        alerts.ForeColor = message.Length == 0 ? Color.DarkGreen : Color.Firebrick;
        if (message != lastAlert)
        {
            if (message.Length > 0)
                AddLog("Alerta: " + message);
            lastAlert = message;
        }
    }

    private int StaleThresholdMs => Math.Max(5000, pollMs * 3);

    private void AddLog(string message)
    {
        string safe = new(message.Where(c => !char.IsControl(c) || c is '\r' or '\n' or '\t').ToArray());
        if (safe.Length > 8192)
            safe = safe[..8192] + "\n[Registro truncado]";
        logs.Enqueue(new LogEntry(DateTimeOffset.Now, safe));
        while (logs.Count > 300)
            logs.Dequeue();
        logView.Text = string.Join(Environment.NewLine, logs.Select(entry => $"[{entry.Fecha:O}] {entry.Mensaje}"));
        logView.SelectionStart = logView.TextLength;
        logView.ScrollToCaret();
    }

    private async Task ExportAsync()
    {
        using var dialog = new SaveFileDialog
        {
            Title = "Guardar diagnóstico",
            Filter = "Texto (*.txt)|*.txt|JSON (*.json)|*.json",
            FileName = $"diagnostico_{DateTime.Now:yyyyMMdd_HHmmss}.txt",
            DefaultExt = "txt",
            AddExtension = true
        };
        if (dialog.ShowDialog(this) != DialogResult.OK)
            return;
        RefreshDashboard();
        var snapshot = new
        {
            FechaExportacion = DateTimeOffset.Now,
            Configuracion = new { IP = sessionIp ?? deviceIp.Text.Trim(), PuertoTelnet = sessionIp is null ? (int)port.Value : sessionPort, IntervaloMs = sessionIp is null ? (int)interval.Value : pollMs },
            Conexion = connection.Text,
            Estado = new { GNSS = gnss.Text, FIX_IN = latest?.FixIn, HAS = has.Text, Solucion = solution.Text },
            UltimaRespuestaValidaUtc = latestAt,
            EdadUltimaRespuestaValidaMs = latestTick is long tick ? (double?)Stopwatch.GetElapsedTime(tick).TotalMilliseconds : null,
            UmbralObsoletoMs = StaleThresholdMs,
            Alertas = alerts.Text,
            Registro = logs.ToArray()
        };
        string content = Path.GetExtension(dialog.FileName).Equals(".json", StringComparison.OrdinalIgnoreCase)
            ? JsonSerializer.Serialize(snapshot, new JsonSerializerOptions { WriteIndented = true })
            : $"Diagnóstico exclusivo por Telnet (sin COM2)\nFecha: {snapshot.FechaExportacion:O}\nIP: {snapshot.Configuracion.IP}\nPuerto: {snapshot.Configuracion.PuertoTelnet}\nIntervalo: {snapshot.Configuracion.IntervaloMs} ms\nConexión: {snapshot.Conexion}\nGNSS: {gnss.Text}\nFIX_IN: {fix.Text}\nHAS: {has.Text}\nSolución: {solution.Text}\nÚltima respuesta válida UTC: {latestAt:O}\nEdad: {age.Text}\nAlertas: {alerts.Text}\n\nRegistro:\n" + string.Join("\n", snapshot.Registro.Select(entry => $"[{entry.Fecha:O}] {entry.Mensaje}"));
        export.Enabled = false;
        try
        {
            await File.WriteAllTextAsync(dialog.FileName, content, new UTF8Encoding(false));
            AddLog("Diagnóstico exportado correctamente.");
        }
        catch (Exception)
        {
            MessageBox.Show(this, "No se pudo guardar el diagnóstico. Comprueba los permisos, la ruta y el espacio disponible.", "Error de exportación", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
        finally { export.Enabled = true; }
    }

    protected override async void OnFormClosing(FormClosingEventArgs e)
    {
        if (!closing)
        {
            e.Cancel = true;
            closing = true;
            Enabled = false;
            timer.Stop();
            await StopAsync();
            if (exportTask is not null)
                await exportTask;
            Close();
            return;
        }
        timer.Dispose();
        base.OnFormClosing(e);
    }
}
