using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace DiagTelnetWin;

public sealed record PollResult(DeviceStatus? Status, string Raw);

public sealed class TelnetSession : IDisposable
{
    private readonly TcpClient client = new();
    private readonly TelnetDecoder decoder = new();
    private NetworkStream? stream;
    public const int ResponseTimeoutMs = 3000;

    public async Task ConnectAsync(IPAddress address, int port, CancellationToken token)
    {
        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(token);
        timeout.CancelAfter(5000);
        await client.ConnectAsync(address, port, timeout.Token);
        stream = client.GetStream();
    }

    public async Task<PollResult> PollAsync(CancellationToken token)
    {
        NetworkStream connection = stream ?? throw new InvalidOperationException();
        await connection.WriteAsync("status\r\n"u8.ToArray(), token);
        var clock = Stopwatch.StartNew();
        var text = new StringBuilder();
        var buffer = new byte[2048];
        int receivedBytes = 0;
        while (clock.ElapsedMilliseconds < ResponseTimeoutMs)
        {
            int remaining = ResponseTimeoutMs - (int)clock.ElapsedMilliseconds;
            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(token);
            timeout.CancelAfter(text.Length > 0 ? Math.Min(250, remaining) : remaining);
            int count;
            try
            {
                count = await connection.ReadAsync(buffer, timeout.Token);
            }
            catch (OperationCanceledException) when (!token.IsCancellationRequested)
            {
                if (StatusParser.Parse(text.ToString()) is not null)
                    break;
                continue;
            }
            if (count == 0)
                throw new IOException("El dispositivo ha cerrado la conexión Telnet.");
            receivedBytes += count;
            if (receivedBytes > 65536)
                throw new IOException("La respuesta Telnet supera el límite de 64 KiB.");
            var decoded = decoder.Decode(buffer.AsSpan(0, count));
            if (decoded.Reply.Length > 0)
                await connection.WriteAsync(decoded.Reply, token);
            text.Append(decoded.Text);
        }
        string raw = text.ToString();
        return new PollResult(StatusParser.Parse(raw), raw);
    }

    public void Dispose() => client.Dispose();
}

// La negociación puede llegar fragmentada; se rechazan opciones sin activar eco.
public sealed class TelnetDecoder
{
    private int state;
    private byte verb;
    private readonly Decoder utf8 = Encoding.UTF8.GetDecoder();

    public (string Text, byte[] Reply) Decode(ReadOnlySpan<byte> input)
    {
        var data = new List<byte>();
        var reply = new List<byte>();
        foreach (byte value in input)
        {
            switch (state)
            {
                case 0:
                    if (value == 255) state = 1;
                    else data.Add(value);
                    break;
                case 1:
                    if (value == 255) { data.Add(value); state = 0; }
                    else if (value is >= 251 and <= 254) { verb = value; state = 2; }
                    else state = value == 250 ? 3 : 0;
                    break;
                case 2:
                    if (verb is 251 or 253)
                        reply.AddRange(new byte[] { 255, (byte)(verb == 251 ? 254 : 252), value });
                    state = 0;
                    break;
                case 3:
                    if (value == 255) state = 4;
                    break;
                case 4:
                    state = value == 240 ? 0 : 3;
                    break;
            }
        }
        byte[] bytes = data.ToArray();
        var chars = new char[Encoding.UTF8.GetMaxCharCount(bytes.Length)];
        int length = utf8.GetChars(bytes, 0, bytes.Length, chars, 0, false);
        return (new string(chars, 0, length), reply.ToArray());
    }
}
