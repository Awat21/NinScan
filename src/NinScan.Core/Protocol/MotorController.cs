using System;
using System.Globalization;
using NinScan.Core.Infrastructure;

namespace NinScan.Core.Protocol
{
    /// <summary>How an axis moves once started with J.</summary>
    public enum MotionMode
    {
        /// <summary>G0: move a set number of steps at full speed, decelerating at the brake point.</summary>
        GotoFast = 0,
        /// <summary>G1: run continuously at the step period set with I (tracking and slow manual moves).</summary>
        SlewSlow = 1,
        /// <summary>G2: move a set number of steps at low speed (final approach).</summary>
        GotoSlow = 2,
        /// <summary>G3: run continuously at high speed; the effective rate is the I period divided by the high speed ratio.</summary>
        SlewFast = 3,
    }

    public struct AxisStatus
    {
        /// <summary>True in continuous (slew/tracking) mode, false in goto mode.</summary>
        public bool SlewMode;
        public bool Reverse;
        public bool HighSpeed;
        public bool Running;
        public bool Blocked;
        public bool Initialised;

        public override string ToString()
        {
            return (Running ? "running" : "stopped") + (SlewMode ? " slew" : " goto") + (Reverse ? " reverse" : " forward")
                + (HighSpeed ? " fast" : " slow") + (Blocked ? " BLOCKED" : "") + (Initialised ? "" : " not-initialised");
        }
    }

    public sealed class AxisParameters
    {
        /// <summary>Microsteps per full revolution of the axis.</summary>
        public int CountsPerRevolution;
        /// <summary>Frequency of the step timer; step period I = TimerFrequency / steps per second.</summary>
        public int TimerFrequency;
        /// <summary>Speed multiplier applied in the fast modes.</summary>
        public int HighSpeedRatio;

        public double StepsPerRadian => CountsPerRevolution / (2 * Math.PI);
    }

    public sealed class BoardInfo
    {
        public int RawVersion;
        public int MajorVersion => (RawVersion >> 16) & 0xFF;
        public int MinorVersion => (RawVersion >> 8) & 0xFF;
        public int MountCode => RawVersion & 0xFF;
        public string VersionText => MajorVersion.ToString("X") + "." + MinorVersion.ToString("X2");

        public string MountName
        {
            get
            {
                int c = MountCode;
                switch (c)
                {
                    case 0x00: return "EQ6";
                    case 0x01: return "HEQ5";
                    case 0x02: return "EQ5";
                    case 0x03: return "EQ3";
                    case 0x04: return "EQ8";
                    case 0x05: return "AZ-EQ6";
                    case 0x06: return "AZ-EQ5";
                    case 0xA5: return "AZ-GTi";
                }
                if (c >= 0x80 && c <= 0x8F) return "AZ GoTo (code 0x" + c.ToString("X2") + ")";
                if (c >= 0x90 && c <= 0x9F) return "Dobsonian (code 0x" + c.ToString("X2") + ")";
                return "Unknown (code 0x" + c.ToString("X2") + ")";
            }
        }
    }

    /// <summary>
    /// Typed wrapper over the Sky-Watcher motor controller command set.
    /// Not thread-safe; the mount engine serialises access.
    /// </summary>
    public sealed class MotorController : IDisposable
    {
        private readonly ITransport _transport;
        private readonly ILog _log;
        private const int Attempts = 3;

        public MotorController(ITransport transport, ILog log)
        {
            _transport = transport;
            _log = log ?? NullLog.Instance;
        }

        public ITransport Transport => _transport;

        public string Send(char command, Axis axis, string data = "")
        {
            string frame = MotorCodec.BuildCommand(command, axis, data);
            MotorException last = null;
            for (int attempt = 1; attempt <= Attempts; attempt++)
            {
                string reply = _transport.Transact(frame);
                try
                {
                    return MotorCodec.ParseReply(frame, reply);
                }
                catch (MotorErrorException)
                {
                    // The board understood and refused: retrying will not help.
                    throw;
                }
                catch (MotorException ex)
                {
                    last = ex;
                    _log.Warn("Attempt " + attempt + " failed: " + ex.Message);
                }
            }
            throw last;
        }

        public BoardInfo InquireVersion()
        {
            // The reply bytes are major, minor, mount code in transmission order.
            string data = Send('e', Axis.Azimuth);
            int raw = int.Parse(data.Substring(0, 6), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
            return new BoardInfo { RawVersion = raw };
        }

        public AxisParameters InquireAxisParameters(Axis axis)
        {
            var p = new AxisParameters
            {
                CountsPerRevolution = MotorCodec.Decode24(Send('a', axis)),
                TimerFrequency = MotorCodec.Decode24(Send('b', axis)),
            };
            try
            {
                p.HighSpeedRatio = MotorCodec.Decode8(Send('g', axis));
            }
            catch (MotorErrorException)
            {
                p.HighSpeedRatio = 16;
            }
            if (p.HighSpeedRatio <= 0) p.HighSpeedRatio = 16;
            if (p.CountsPerRevolution <= 0) throw new MotorException("Mount reported zero counts per revolution on axis " + axis + ".");
            if (p.TimerFrequency <= 0) throw new MotorException("Mount reported zero timer frequency on axis " + axis + ".");
            return p;
        }

        public int InquirePosition(Axis axis) => MotorCodec.DecodePosition(Send('j', axis));

        public void SetPosition(Axis axis, int steps) => Send('E', axis, MotorCodec.EncodePosition(steps));

        public AxisStatus InquireStatus(Axis axis)
        {
            string data = Send('f', axis);
            if (data.Length < 3) throw new MotorProtocolException("f", data, "Status needs three digits.");
            int a = Nibble(data[0]), b = Nibble(data[1]), c = Nibble(data[2]);
            return new AxisStatus
            {
                SlewMode = (a & 0x1) != 0,
                Reverse = (a & 0x2) != 0,
                HighSpeed = (a & 0x4) != 0,
                Running = (b & 0x1) != 0,
                Blocked = (b & 0x2) != 0,
                Initialised = (c & 0x1) != 0,
            };
        }

        public void InitialisationDone() => Send('F', Axis.Both);

        public void SetMotionMode(Axis axis, MotionMode mode, bool reverse)
        {
            // Second digit: bit 0 is direction, bit 1 is hemisphere (always north here; an alt-az mount does not care).
            Send('G', axis, ((int)mode).ToString(CultureInfo.InvariantCulture) + (reverse ? "1" : "0"));
        }

        public void SetStepPeriod(Axis axis, int period) => Send('I', axis, MotorCodec.Encode24(Clamp24(period)));

        public void SetGotoIncrement(Axis axis, int steps) => Send('H', axis, MotorCodec.Encode24(Clamp24(steps)));

        public void SetBrakeIncrement(Axis axis, int steps) => Send('M', axis, MotorCodec.Encode24(Clamp24(steps)));

        public void Start(Axis axis) => Send('J', axis);

        /// <summary>Decelerating stop.</summary>
        public void Stop(Axis axis) => Send('K', axis);

        public void InstantStop(Axis axis) => Send('L', axis);

        /// <summary>Raw extended-feature inquiry (q). The bit meanings vary by firmware, so this is reported as-is.</summary>
        public int? InquireFeatures(Axis axis)
        {
            try { return MotorCodec.Decode24(Send('q', axis, "010000")); }
            catch (MotorException) { return null; }
        }

        /// <summary>Auxiliary (Freedom Find) encoder reading, if the board supports it.</summary>
        public int? InquireAuxEncoder(Axis axis)
        {
            try { return MotorCodec.DecodePosition(Send('d', axis)); }
            catch (MotorException) { return null; }
        }

        /// <summary>
        /// Switches the auxiliary (Freedom Find) encoder on or off with the extended setting command
        /// (W 04 / W 05), as the SynScan App does. Not yet confirmed on a Flextube board.
        /// </summary>
        public void SetAuxEncoders(Axis axis, bool on) => Send('W', axis, on ? "040000" : "050000");

        public void Dispose() => _transport.Dispose();

        private static int Clamp24(int v) => v < 0 ? 0 : (v > 0xFFFFFF ? 0xFFFFFF : v);

        private static int Nibble(char c) => int.Parse(c.ToString(), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
    }
}
