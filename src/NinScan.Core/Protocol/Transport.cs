using System;
using System.IO.Ports;
using System.Text;
using NinScan.Core.Infrastructure;

namespace NinScan.Core.Protocol
{
    /// <summary>Sends one framed command and returns the reply line, or null on timeout.</summary>
    public interface ITransport : IDisposable
    {
        string Transact(string command);
        string Description { get; }
    }

    /// <summary>
    /// Serial link to the motor board. The Sky-Watcher USB adapter shows up as a COM port.
    /// The mount talks 8N1; older boards and the hand controller use 9600 baud,
    /// mounts with a built-in USB port use 115200.
    /// </summary>
    public sealed class SerialTransport : ITransport
    {
        private readonly SerialPort _port;
        private readonly ILog _log;
        private readonly int _timeoutMs;

        public SerialTransport(string portName, int baudRate, ILog log, int timeoutMs = 500)
        {
            _log = log ?? NullLog.Instance;
            _timeoutMs = timeoutMs;
            _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                Handshake = Handshake.None,
                ReadTimeout = timeoutMs,
                WriteTimeout = timeoutMs,
                Encoding = Encoding.ASCII,
                NewLine = "\r",
                DtrEnable = true,
                RtsEnable = true,
            };
            _port.Open();
        }

        public string Description => _port.PortName + " @ " + _port.BaudRate;

        public string Transact(string command)
        {
            _port.DiscardInBuffer();
            _port.Write(command);
            var sb = new StringBuilder();
            var deadline = DateTime.UtcNow.AddMilliseconds(_timeoutMs);
            while (DateTime.UtcNow < deadline)
            {
                int c;
                try
                {
                    c = _port.ReadChar();
                }
                catch (TimeoutException)
                {
                    break;
                }
                if (c == '\r')
                {
                    // Some links echo the command back; skip anything that is not a reply.
                    var line = sb.ToString();
                    if (line.Length > 0 && (line[0] == '=' || line[0] == '!'))
                    {
                        _log.Debug("TX " + command.TrimEnd('\r') + "  RX " + line);
                        return line;
                    }
                    sb.Clear();
                    continue;
                }
                sb.Append((char)c);
            }
            _log.Debug("TX " + command.TrimEnd('\r') + "  RX <timeout>");
            return null;
        }

        public void Dispose()
        {
            try { _port.Close(); } catch { }
            _port.Dispose();
        }

        public static string[] AvailablePorts()
        {
            try { return SerialPort.GetPortNames(); }
            catch { return new string[0]; }
        }
    }
}
