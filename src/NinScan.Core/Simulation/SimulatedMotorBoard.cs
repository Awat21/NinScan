using System;
using System.Globalization;
using NinScan.Core.Infrastructure;
using NinScan.Core.Protocol;

namespace NinScan.Core.Simulation
{
    /// <summary>
    /// A software stand-in for the motor board that answers the same serial commands.
    /// It models speeds, acceleration, goto braking and the "motor not stopped" rule,
    /// so the mount engine can be exercised end to end without hardware.
    /// </summary>
    public sealed class SimulatedMotorBoard : ITransport
    {
        private readonly object _lock = new object();
        private readonly IClock _clock;
        private readonly SimAxis[] _axes = new SimAxis[3];

        public SimulatedMotorBoard(IClock clock, int countsPerRevolution = 3628800, int timerFrequency = 50000, int highSpeedRatio = 16,
            string version = "0350A0")
        {
            _clock = clock;
            CountsPerRevolution = countsPerRevolution;
            TimerFrequency = timerFrequency;
            HighSpeedRatio = highSpeedRatio;
            VersionReply = version;
            var now = clock.UtcNow;
            _axes[1] = new SimAxis(this, now);
            _axes[2] = new SimAxis(this, now);
        }

        public int CountsPerRevolution { get; }
        public int TimerFrequency { get; }
        public int HighSpeedRatio { get; }
        public string VersionReply { get; }

        /// <summary>Top speed of a fast goto, degrees per second.</summary>
        public double GotoFastDegPerSec { get; set; } = 3.4;
        /// <summary>Top speed of a slow goto, degrees per second.</summary>
        public double GotoSlowDegPerSec { get; set; } = 0.2;
        /// <summary>Acceleration, degrees per second squared.</summary>
        public double AccelerationDegPerSec2 { get; set; } = 2.0;
        public bool Initialised { get; private set; }
        public int CommandCount { get; private set; }
        /// <summary>When set, every command times out (for testing error handling).</summary>
        public bool Unresponsive { get; set; }

        public string Description => "Simulator";

        internal double StepsPerDegree => CountsPerRevolution / 360.0;

        /// <summary>Position in steps, as the board would report it.</summary>
        public double PositionOf(Axis axis)
        {
            lock (_lock)
            {
                Advance();
                return _axes[(int)axis].Position;
            }
        }

        /// <summary>Where the tube actually points, in steps. Differs from the motor count by the gear play and any slip.</summary>
        public double TubePositionOf(Axis axis)
        {
            lock (_lock)
            {
                Advance();
                return _axes[(int)axis].Tube;
            }
        }

        /// <summary>Gear backlash (full play) of an axis, in steps. The tube only follows once the motor has taken it up.</summary>
        public void SetBacklash(Axis axis, double steps)
        {
            lock (_lock)
            {
                Advance();
                _axes[(int)axis].Backlash = steps;
            }
        }

        /// <summary>Simulates a clutch slip: the tube ends up this many steps away from where the motors put it.</summary>
        public void Slip(Axis axis, double steps)
        {
            lock (_lock)
            {
                Advance();
                _axes[(int)axis].SlipOffset += steps;
            }
        }

        /// <summary>Simulates someone pushing the tube by hand (Freedom Find keeps the count).</summary>
        public void Push(Axis axis, double steps)
        {
            lock (_lock)
            {
                Advance();
                _axes[(int)axis].Position += steps;
            }
        }

        public bool IsRunning(Axis axis)
        {
            lock (_lock)
            {
                Advance();
                return _axes[(int)axis].Running;
            }
        }

        public string Transact(string command)
        {
            lock (_lock)
            {
                CommandCount++;
                if (Unresponsive) return null;
                Advance();
                try
                {
                    return Handle(command);
                }
                catch (FormatException)
                {
                    return "!3";
                }
                catch (ArgumentOutOfRangeException)
                {
                    return "!1";
                }
            }
        }

        public void Dispose() { }

        private void Advance()
        {
            var now = _clock.UtcNow;
            _axes[1].Advance(now);
            _axes[2].Advance(now);
        }

        private string Handle(string frame)
        {
            if (frame.Length < 4 || frame[0] != ':' || frame[frame.Length - 1] != '\r') return "!1";
            char cmd = frame[1];
            char axisChar = frame[2];
            string data = frame.Substring(3, frame.Length - 4);
            if (axisChar != '1' && axisChar != '2' && axisChar != '3') return "!3";
            if (axisChar == '3')
            {
                if (cmd == 'F') { Initialised = true; return "="; }
                return "!0";
            }
            var ax = _axes[axisChar - '0'];

            switch (cmd)
            {
                case 'e': return "=" + VersionReply;
                case 'a': return "=" + MotorCodec.Encode24(CountsPerRevolution);
                case 'b': return "=" + MotorCodec.Encode24(TimerFrequency);
                case 'g': return "=" + MotorCodec.Encode8(HighSpeedRatio);
                case 's': return "=" + MotorCodec.Encode24(CountsPerRevolution / 360);
                case 'j': return "=" + MotorCodec.EncodePosition((int)Math.Round(ax.Position));
                case 'd': return "=" + MotorCodec.EncodePosition((int)Math.Round(ax.Position));
                case 'q': return "=000000";
                case 'f': return "=" + ax.StatusText(Initialised);
                case 'F': Initialised = true; return "=";
                case 'E':
                    if (ax.Running) return "!2";
                    // Setting the count does not move anything; keep the tube where it is relative to it.
                    double shift = MotorCodec.DecodePosition(data) - ax.Position;
                    ax.Position += shift;
                    ax.SlipOffset += shift;
                    return "=";
                case 'G':
                    if (ax.Running) return "!2";
                    if (data.Length != 2) return "!1";
                    int mode = int.Parse(data.Substring(0, 1), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
                    int dir = int.Parse(data.Substring(1, 1), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
                    if (mode < 0 || mode > 3) return "!3";
                    ax.Mode = (MotionMode)mode;
                    ax.Reverse = (dir & 1) != 0;
                    return "=";
                case 'I':
                    if (data.Length != 6) return "!1";
                    ax.Period = MotorCodec.Decode24(data);
                    return "=";
                case 'H':
                    if (ax.Running) return "!2";
                    if (data.Length != 6) return "!1";
                    ax.GotoIncrement = MotorCodec.Decode24(data);
                    return "=";
                case 'M':
                    if (data.Length != 6) return "!1";
                    return "=";
                case 'J':
                    if (!Initialised) return "!4";
                    ax.Start();
                    return "=";
                case 'K':
                    ax.Stop(instant: false);
                    return "=";
                case 'L':
                    ax.Stop(instant: true);
                    return "=";
                case 'V':
                case 'O':
                case 'W':
                    return "=";
                default:
                    return "!0";
            }
        }

        private sealed class SimAxis
        {
            private readonly SimulatedMotorBoard _board;
            private DateTime _last;
            private double _speed;           // current speed, steps/s, always >= 0
            private double _gotoRemaining;   // steps left in a goto
            private bool _stopping;

            public SimAxis(SimulatedMotorBoard board, DateTime now)
            {
                _board = board;
                _last = now;
            }

            public double Position;
            /// <summary>Tube position in steps: the motor count less the gear play, plus any clutch slip.</summary>
            public double Tube => Position - Play + SlipOffset;
            /// <summary>Motor minus tube through the gears, between -Backlash/2 and +Backlash/2.</summary>
            public double Play;
            public double SlipOffset;
            public double Backlash;
            public bool Running;
            public MotionMode Mode = MotionMode.SlewSlow;
            public bool Reverse;
            public int Period = 0xFFFFFF;
            public int GotoIncrement;

            private bool IsGoto => Mode == MotionMode.GotoFast || Mode == MotionMode.GotoSlow;

            public string StatusText(bool initialised)
            {
                int a = (IsGoto ? 0 : 1) | (Reverse ? 2 : 0) | ((Mode == MotionMode.GotoFast || Mode == MotionMode.SlewFast) ? 4 : 0);
                int b = Running ? 1 : 0;
                int c = initialised ? 1 : 0;
                return a.ToString("X") + b.ToString("X") + c.ToString("X");
            }

            public void Start()
            {
                if (Running && !_stopping) return;
                Running = true;
                _stopping = false;
                if (IsGoto) _gotoRemaining = GotoIncrement;
            }

            public void Stop(bool instant)
            {
                if (!Running) return;
                if (instant)
                {
                    Running = false;
                    _speed = 0;
                    _stopping = false;
                }
                else
                {
                    _stopping = true;
                }
            }

            private double TargetSpeed()
            {
                double spd = _board.StepsPerDegree;
                switch (Mode)
                {
                    case MotionMode.SlewSlow: return Period <= 0 ? 0 : (double)_board.TimerFrequency / Period;
                    case MotionMode.SlewFast: return Period <= 0 ? 0 : (double)_board.TimerFrequency * _board.HighSpeedRatio / Period;
                    case MotionMode.GotoFast: return _board.GotoFastDegPerSec * spd;
                    default: return _board.GotoSlowDegPerSec * spd;
                }
            }

            public void Advance(DateTime now)
            {
                double total = (now - _last).TotalSeconds;
                _last = now;
                if (!Running || total <= 0) return;
                double accel = _board.AccelerationDegPerSec2 * _board.StepsPerDegree;
                const double maxStep = 0.02;
                while (total > 1e-9 && Running)
                {
                    double dt = Math.Min(total, maxStep);
                    total -= dt;

                    double target = _stopping ? 0 : TargetSpeed();
                    if (IsGoto && !_stopping)
                    {
                        // Brake so the axis arrives at the target with zero speed.
                        double brakeLimit = Math.Sqrt(2 * accel * Math.Max(0, _gotoRemaining));
                        target = Math.Min(target, Math.Max(brakeLimit, accel * maxStep));
                    }

                    // Slow tracking speeds change instantly, like the real board does at sidereal rates.
                    double instantLimit = 64 * _board.StepsPerDegree * 15.041 / 3600.0;
                    if (target <= instantLimit && _speed <= instantLimit) _speed = target;
                    else if (_speed < target) _speed = Math.Min(target, _speed + accel * dt);
                    else _speed = Math.Max(target, _speed - accel * dt);

                    double move = _speed * dt;
                    if (IsGoto && !_stopping)
                    {
                        if (move >= _gotoRemaining)
                        {
                            move = _gotoRemaining;
                            _gotoRemaining = 0;
                            Running = false;
                            _speed = 0;
                        }
                        else
                        {
                            _gotoRemaining -= move;
                        }
                    }
                    Position += Reverse ? -move : move;
                    double half = Backlash / 2;
                    Play = Math.Max(-half, Math.Min(half, Play + (Reverse ? -move : move)));

                    if (_stopping && _speed <= 0)
                    {
                        Running = false;
                        _stopping = false;
                    }
                }
            }
        }
    }
}
