using System;
using System.Globalization;

namespace NinScan.Core.Protocol
{
    /// <summary>
    /// Encoding rules of the Sky-Watcher motor controller protocol.
    /// Commands look like ":" + command + axis + data + "\r".
    /// Replies are "=" + data + "\r" on success or "!" + error code + "\r" on failure.
    /// 24-bit numbers are sent as six hex digits, least significant byte first,
    /// so 0x123456 travels as "563412".
    /// </summary>
    public static class MotorCodec
    {
        /// <summary>Positions are offset by this value so that zero sits in the middle of the 24-bit range.</summary>
        public const int PositionOffset = 0x800000;

        public static string Encode24(int value)
        {
            if (value < 0 || value > 0xFFFFFF) throw new ArgumentOutOfRangeException(nameof(value), value, "Value must fit in 24 bits.");
            return (value & 0xFF).ToString("X2") + ((value >> 8) & 0xFF).ToString("X2") + ((value >> 16) & 0xFF).ToString("X2");
        }

        public static int Decode24(string data)
        {
            if (data == null || data.Length < 6) throw new FormatException("Expected six hex digits, got '" + data + "'.");
            int b0 = ParseHexByte(data, 0);
            int b1 = ParseHexByte(data, 2);
            int b2 = ParseHexByte(data, 4);
            return b0 | (b1 << 8) | (b2 << 16);
        }

        public static string Encode8(int value)
        {
            if (value < 0 || value > 0xFF) throw new ArgumentOutOfRangeException(nameof(value), value, "Value must fit in 8 bits.");
            return value.ToString("X2");
        }

        public static int Decode8(string data)
        {
            if (data == null || data.Length < 2) throw new FormatException("Expected two hex digits, got '" + data + "'.");
            return ParseHexByte(data, 0);
        }

        public static string EncodePosition(int steps) => Encode24(steps + PositionOffset);

        public static int DecodePosition(string data) => Decode24(data) - PositionOffset;

        public static string BuildCommand(char command, Axis axis, string data = "")
        {
            return ":" + command + (char)('0' + (int)axis) + (data ?? "") + "\r";
        }

        /// <summary>Parses a reply line (with or without the trailing carriage return).</summary>
        public static string ParseReply(string command, string reply)
        {
            if (reply == null) throw new MotorTimeoutException(command);
            reply = reply.TrimEnd('\r', '\n');
            if (reply.Length == 0) throw new MotorProtocolException(command, reply, "Empty reply.");
            if (reply[0] == '=') return reply.Substring(1);
            if (reply[0] == '!')
            {
                int code = -1;
                if (reply.Length > 1) int.TryParse(reply.Substring(1), NumberStyles.Integer, CultureInfo.InvariantCulture, out code);
                throw new MotorErrorException(command, code);
            }
            throw new MotorProtocolException(command, reply, "Reply does not start with '=' or '!'.");
        }

        private static int ParseHexByte(string s, int index)
        {
            return int.Parse(s.Substring(index, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
        }
    }

    public enum Axis
    {
        /// <summary>Axis 1: azimuth on an alt-az mount.</summary>
        Azimuth = 1,
        /// <summary>Axis 2: altitude on an alt-az mount.</summary>
        Altitude = 2,
        /// <summary>Both axes at once (only valid for a few commands such as F).</summary>
        Both = 3,
    }

    public class MotorException : Exception
    {
        public MotorException(string message) : base(message) { }
    }

    public sealed class MotorTimeoutException : MotorException
    {
        public MotorTimeoutException(string command)
            : base("No reply from the mount to " + Printable(command) + ". Check the USB cable, the COM port and that the mount is powered.") { }

        internal static string Printable(string command) => (command ?? "").Replace("\r", "");
    }

    public sealed class MotorProtocolException : MotorException
    {
        public MotorProtocolException(string command, string reply, string detail)
            : base("Unexpected reply '" + reply + "' to " + MotorTimeoutException.Printable(command) + ": " + detail) { }
    }

    public sealed class MotorErrorException : MotorException
    {
        public MotorErrorException(string command, int code)
            : base("Mount rejected " + MotorTimeoutException.Printable(command) + " with error " + code + " (" + Describe(code) + ").")
        {
            Code = code;
        }

        public int Code { get; }

        public static string Describe(int code)
        {
            switch (code)
            {
                case 0: return "unknown command";
                case 1: return "command length error";
                case 2: return "motor not stopped";
                case 3: return "invalid character";
                case 4: return "not initialised";
                case 5: return "driver sleeping";
                case 7: return "PEC training running";
                case 8: return "no valid PEC data";
                default: return "unknown error";
            }
        }
    }
}
