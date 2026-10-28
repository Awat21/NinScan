using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using System.Threading;
using NinScan.Core.Astronomy;
using NinScan.Core.Infrastructure;
using NinScan.Core.Pointing;
using NinScan.Core.Protocol;
using NinScan.Core.Simulation;

namespace NinScan.Core.Mount
{
    public enum TrackingRateKind
    {
        Sidereal,
        Lunar,
        Solar,
    }

    /// <summary>
    /// The alt-az mount engine. It owns the serial link, turns sky coordinates into axis angles through
    /// the pointing model, runs gotos and keeps both axes tracking. The motor board itself only
    /// understands step counts and step rates, so everything else happens here.
    ///
    /// All public members are thread-safe. A background loop calls <see cref="Tick"/> to refresh the
    /// position, supervise gotos and update tracking rates.
    /// </summary>
    public sealed class AltAzMount : IDisposable
    {
        private readonly object _sync = new object();
        private readonly IClock _clock;
        private readonly ILog _log;
        private readonly string _stateDirectory;
        private readonly bool _runBackgroundLoop;
        private readonly Func<MountSettings, ITransport> _transportFactory;
        private MountSettings _settings;

        private SimulatedMotorBoard _simulator;
        private MotorController _mc;
        private AxisParameters _pAz, _pAlt;
        private readonly AxisDrive _driveAz = new AxisDrive(Axis.Azimuth);
        private readonly AxisDrive _driveAlt = new AxisDrive(Axis.Altitude);

        private AltAz _mountPos;        // mount axis angles, azimuth unwrapped
        private DateTime _posTime;

        private bool _tracking;
        private TrackingRateKind _rate = TrackingRateKind.Sidereal;
        private RaDec _trackTarget;
        private DateTime _trackEpoch;
        private int _trackTicks;

        private GotoJob _goto;
        private readonly bool[] _moving = new bool[3];
        private bool _atPark;

        private Thread _thread;
        private volatile bool _run;
        private int _consecutiveErrors;

        public AltAzMount(MountSettings settings, IClock clock, ILog log, string stateDirectory,
            bool runBackgroundLoop = true, Func<MountSettings, ITransport> transportFactory = null)
        {
            _settings = (settings ?? new MountSettings()).Clone();
            _clock = clock ?? SystemClock.Instance;
            _log = log ?? NullLog.Instance;
            _stateDirectory = stateDirectory;
            _runBackgroundLoop = runBackgroundLoop;
            _transportFactory = transportFactory;
            Model = new PointingModel();
            ApplyModelSettings();
        }

        public PointingModel Model { get; }
        public BoardInfo BoardInfo { get; private set; }
        public AxisParameters AzimuthParameters => _pAz;
        public AxisParameters AltitudeParameters => _pAlt;
        public string LastError { get; private set; }
        public string ConnectionDescription { get; private set; }

        /// <summary>The simulated board, when connected to the simulator (for tests and demos).</summary>
        public SimulatedMotorBoard Simulator => _simulator;

        public MountSettings Settings
        {
            get { lock (_sync) return _settings.Clone(); }
        }

        /// <summary>Replaces the settings. The serial port only changes on the next connect.</summary>
        public void UpdateSettings(MountSettings settings)
        {
            settings.Validate();
            lock (_sync)
            {
                _settings = settings.Clone();
                ApplyModelSettings();
            }
        }

        public Site Site
        {
            get
            {
                lock (_sync)
                {
                    return new Site
                    {
                        LatitudeDeg = _settings.LatitudeDeg,
                        LongitudeDeg = _settings.LongitudeDeg,
                        ElevationM = _settings.ElevationM,
                        PressureHPa = _settings.PressureHPa,
                        TemperatureC = _settings.TemperatureC,
                    };
                }
            }
        }

        // ------------------------------------------------------------------ connection

        public bool Connected
        {
            get { lock (_sync) return _mc != null; }
        }

        public void Connect()
        {
            lock (_sync)
            {
                if (_mc != null) return;
                _settings.Validate();
                var transport = OpenTransport();
                var mc = new MotorController(transport, _log);
                try
                {
                    BoardInfo = mc.InquireVersion();
                    _pAz = mc.InquireAxisParameters(Axis.Azimuth);
                    _pAlt = mc.InquireAxisParameters(Axis.Altitude);
                    var status = mc.InquireStatus(Axis.Azimuth);
                    _mc = mc;
                    ConnectionDescription = transport.Description;
                    _log.Info("Connected via " + transport.Description + ": motor board " + BoardInfo.VersionText + ", " + BoardInfo.MountName
                        + "; az " + _pAz.CountsPerRevolution + " steps/rev, timer " + _pAz.TimerFrequency + " Hz, high-speed x" + _pAz.HighSpeedRatio
                        + "; alt " + _pAlt.CountsPerRevolution + " steps/rev, timer " + _pAlt.TimerFrequency + " Hz, high-speed x" + _pAlt.HighSpeedRatio);
                    LogFeatures(mc);
                    ApplyAuxEncoderSetting(mc);

                    StopAndWait(_driveAz, force: true);
                    StopAndWait(_driveAlt, force: true);

                    if (!status.Initialised)
                    {
                        // Fresh power-on: the board counts from zero, which by convention is the start position.
                        _mc.SetPosition(Axis.Azimuth, AngleToSteps(Axis.Azimuth, AstroMath.ToRad(_settings.StartAzimuthDeg)));
                        _mc.SetPosition(Axis.Altitude, AngleToSteps(Axis.Altitude, AstroMath.ToRad(_settings.StartAltitudeDeg)));
                        _mc.InitialisationDone();
                        Model.Clear();
                        _atPark = false;
                        _log.Info("Mount was just powered on: assuming the tube is at the start position (az "
                            + _settings.StartAzimuthDeg + "°, alt " + _settings.StartAltitudeDeg + "°). Pointing model cleared.");
                        SaveState();
                    }
                    else
                    {
                        LoadState();
                        _log.Info("Mount was already running, so it kept its position. Restored " + Model.PointCount
                            + " model point(s); parked = " + _atPark + ".");
                    }

                    _tracking = false;
                    _goto = null;
                    _moving[1] = _moving[2] = false;
                    ReadPosition();
                    LastError = null;
                }
                catch
                {
                    _mc = null;
                    mc.Dispose();
                    throw;
                }
            }

            if (_runBackgroundLoop)
            {
                _run = true;
                _thread = new Thread(Loop) { IsBackground = true, Name = "NinScan mount loop" };
                _thread.Start();
            }
        }

        public void Disconnect()
        {
            _run = false;
            var t = _thread;
            if (t != null && t != Thread.CurrentThread) t.Join(5000);
            _thread = null;

            lock (_sync)
            {
                if (_mc == null) return;
                try
                {
                    // Never leave the motors running at a stale rate once nobody is steering them.
                    StopAndWait(_driveAz, force: true);
                    StopAndWait(_driveAlt, force: true);
                }
                catch (Exception ex)
                {
                    _log.Warn("Could not stop the motors while disconnecting: " + ex.Message);
                }
                SaveState();
                _mc.Dispose();
                _mc = null;
                _tracking = false;
                _goto = null;
                _log.Info("Disconnected.");
            }
        }

        public void Dispose() => Disconnect();

        private ITransport OpenTransport()
        {
            if (_transportFactory != null) return _transportFactory(_settings);
            if (string.Equals(_settings.ComPort, MountSettings.SimulatorPort, StringComparison.OrdinalIgnoreCase))
            {
                if (_simulator == null) _simulator = new SimulatedMotorBoard(_clock);
                return new NonDisposingTransport(_simulator);
            }

            var bauds = _settings.BaudRate > 0 ? new[] { _settings.BaudRate } : new[] { 9600, 115200 };
            Exception last = null;
            foreach (var baud in bauds)
            {
                SerialTransport t = null;
                try
                {
                    t = new SerialTransport(_settings.ComPort, baud, _log);
                    for (int i = 0; i < 2; i++)
                    {
                        var reply = t.Transact(":e1\r");
                        if (reply != null && reply.StartsWith("=")) return t;
                    }
                    _log.Info("No reply on " + _settings.ComPort + " at " + baud + " baud.");
                }
                catch (Exception ex)
                {
                    last = ex;
                    _log.Warn("Could not open " + _settings.ComPort + " at " + baud + " baud: " + ex.Message);
                }
                t?.Dispose();
            }
            throw new MotorException("The mount did not answer on " + _settings.ComPort + " (tried " + string.Join(", ", bauds) + " baud). "
                + "Check the port in the setup dialog, that the mount is switched on, and that no other program (such as the SynScan App) has the port open."
                + (last != null ? " Last error: " + last.Message : ""));
        }

        private void LogFeatures(MotorController mc)
        {
            var f = mc.InquireFeatures(Axis.Azimuth);
            var encAz = mc.InquireAuxEncoder(Axis.Azimuth);
            var encAlt = mc.InquireAuxEncoder(Axis.Altitude);
            _log.Info("Extended features: " + (f.HasValue ? "0x" + f.Value.ToString("X6") : "not reported")
                + "; aux encoders az " + (encAz?.ToString() ?? "n/a") + ", alt " + (encAlt?.ToString() ?? "n/a"));
        }

        private void ApplyAuxEncoderSetting(MotorController mc)
        {
            if (_settings.AuxEncoders == AuxEncoderMode.LeaveAsIs) return;
            bool on = _settings.AuxEncoders == AuxEncoderMode.On;
            foreach (var axis in new[] { Axis.Azimuth, Axis.Altitude })
            {
                try
                {
                    mc.SetAuxEncoders(axis, on);
                }
                catch (MotorException ex)
                {
                    _log.Warn("Could not switch the " + axis + " Freedom Find encoder " + (on ? "on" : "off") + ": " + ex.Message);
                }
            }
            _log.Info("Freedom Find encoders switched " + (on ? "on" : "off") + ".");
        }

        // ------------------------------------------------------------------ position and conversions

        /// <summary>Mount axis angles in radians (azimuth unwrapped), as of the last refresh.</summary>
        public AltAz MountPosition
        {
            get { lock (_sync) return _mountPos; }
        }

        /// <summary>Apparent (refracted) altitude and azimuth the tube points at, radians.</summary>
        public AltAz SkyPosition
        {
            get { lock (_sync) return ApparentFromMount(_mountPos); }
        }

        public RaDec Equatorial
        {
            get
            {
                lock (_sync) return RaDecFromApparent(ApparentFromMount(_mountPos), _posTime == default ? _clock.UtcNow : _posTime);
            }
        }

        public double LocalSiderealTimeHours => AstroMath.LocalSiderealHours(_clock.UtcNow, Site.LongitudeDeg);

        public DateTime UtcNow => _clock.UtcNow;

        public AltAz ApparentFromRaDec(RaDec rd, DateTime utc)
        {
            var site = Site;
            var h = AstroMath.RaDecToAltAz(rd, utc, site);
            if (_settings.Refraction) h.Alt = AstroMath.TrueToApparent(h.Alt, site);
            return h;
        }

        public RaDec RaDecFromApparent(AltAz apparent, DateTime utc)
        {
            var site = Site;
            var h = apparent;
            if (_settings.Refraction) h.Alt = AstroMath.ApparentToTrue(h.Alt, site);
            return AstroMath.AltAzToRaDec(h, utc, site);
        }

        private AltAz ApparentFromMount(AltAz mount)
        {
            return Model.MountToSky(new AltAz(AstroMath.Wrap2Pi(mount.Az), mount.Alt));
        }

        /// <summary>Mount angles for a sky position, with azimuth unwrapped to the closest legal turn. Az is NaN if unreachable.</summary>
        private AltAz MountFromApparent(AltAz apparent, double referenceAz)
        {
            var m = Model.SkyToMount(apparent);
            return new AltAz(ChooseAzimuth(m.Az, referenceAz), m.Alt);
        }

        private double ChooseAzimuth(double wrapped, double reference)
        {
            double start = AstroMath.ToRad(_settings.StartAzimuthDeg);
            double travel = AstroMath.ToRad(_settings.AzimuthTravelDeg);
            double best = double.NaN, bestDist = double.MaxValue;
            for (int k = -4; k <= 4; k++)
            {
                double c = wrapped + 2 * Math.PI * k;
                if (c < start - travel - 1e-9 || c > start + travel + 1e-9) continue;
                double d = Math.Abs(c - reference);
                if (d < bestDist)
                {
                    bestDist = d;
                    best = c;
                }
            }
            return best;
        }

        private double Sign(Axis axis)
        {
            bool rev = axis == Axis.Azimuth ? _settings.ReverseAzimuth : _settings.ReverseAltitude;
            return rev ? -1 : 1;
        }

        private double StartAngle(Axis axis) => AstroMath.ToRad(axis == Axis.Azimuth ? _settings.StartAzimuthDeg : _settings.StartAltitudeDeg);

        private AxisParameters Params(Axis axis) => axis == Axis.Azimuth ? _pAz : _pAlt;

        private AxisDrive Drive(Axis axis) => axis == Axis.Azimuth ? _driveAz : _driveAlt;

        private double StepsToAngle(Axis axis, int steps) => StartAngle(axis) + Sign(axis) * steps / Params(axis).StepsPerRadian;

        private int AngleToSteps(Axis axis, double angle)
        {
            return (int)Math.Round((angle - StartAngle(axis)) * Params(axis).StepsPerRadian * Sign(axis));
        }

        private void ReadPosition()
        {
            int az = _mc.InquirePosition(Axis.Azimuth);
            int alt = _mc.InquirePosition(Axis.Altitude);
            _mountPos = new AltAz(StepsToAngle(Axis.Azimuth, az), StepsToAngle(Axis.Altitude, alt));
            _posTime = _clock.UtcNow;
        }

        // ------------------------------------------------------------------ state

        public bool Slewing
        {
            get { lock (_sync) return _goto != null || _moving[1] || _moving[2]; }
        }

        public bool AtPark
        {
            get { lock (_sync) return _atPark; }
        }

        public bool IsParking
        {
            get { lock (_sync) return _goto != null && _goto.Park; }
        }

        public bool Tracking
        {
            get { lock (_sync) return _tracking; }
            set
            {
                lock (_sync)
                {
                    RequireConnected();
                    if (value == _tracking) return;
                    if (value)
                    {
                        if (_atPark) throw new InvalidOperationException("The mount is parked. Unpark it before tracking.");
                        ReadPosition();
                        CaptureTrackTarget();
                        _tracking = true;
                        _log.Info("Tracking on at " + _rate + " rate, target " + _trackTarget + ".");
                        if (_goto == null && !_moving[1] && !_moving[2]) UpdateTracking();
                    }
                    else
                    {
                        _tracking = false;
                        if (_goto == null && !_moving[1] && !_moving[2])
                        {
                            StopAndWait(_driveAz, force: false);
                            StopAndWait(_driveAlt, force: false);
                        }
                        _log.Info("Tracking off.");
                    }
                }
            }
        }

        public TrackingRateKind TrackingRate
        {
            get { lock (_sync) return _rate; }
            set
            {
                lock (_sync)
                {
                    if (_tracking)
                    {
                        // Re-anchor so the change of rate does not make the target jump.
                        _trackTarget = TrackTargetAt(_clock.UtcNow);
                        _trackEpoch = _clock.UtcNow;
                    }
                    _rate = value;
                }
            }
        }

        /// <summary>The RA/Dec currently being tracked (or the goto target), for diagnostics.</summary>
        public RaDec? TrackTarget
        {
            get { lock (_sync) return _tracking || (_goto != null && _goto.IsRaDec) ? TrackTargetAt(_clock.UtcNow) : (RaDec?)null; }
        }

        private void CaptureTrackTarget()
        {
            var now = _clock.UtcNow;
            _trackTarget = RaDecFromApparent(ApparentFromMount(_mountPos), now);
            _trackEpoch = now;
            _trackTicks = 0;
        }

        private RaDec TrackTargetAt(DateTime utc)
        {
            double rate;
            switch (_rate)
            {
                case TrackingRateKind.Lunar: rate = AstroMath.LunarRateArcsecPerSec; break;
                case TrackingRateKind.Solar: rate = AstroMath.SolarRateArcsecPerSec; break;
                default: rate = AstroMath.SiderealRateArcsecPerSec; break;
            }
            // Non-sidereal targets drift east against the stars; RA is in hours (1 h = 54000″).
            double driftHours = (AstroMath.SiderealRateArcsecPerSec - rate) * (utc - _trackEpoch).TotalSeconds / 54000.0;
            return new RaDec(AstroMath.Wrap24(_trackTarget.RaHours + driftHours), _trackTarget.DecDeg);
        }

        // ------------------------------------------------------------------ gotos

        /// <summary>Starts a goto to an RA/Dec and tracks it on arrival. Returns at once.</summary>
        public void SlewToEquatorial(RaDec target)
        {
            lock (_sync)
            {
                RequireConnected();
                RequireUnparked();
                if (target.DecDeg < -90 || target.DecDeg > 90) throw new ArgumentOutOfRangeException(nameof(target), "Declination must be between -90 and 90.");
                var now = _clock.UtcNow;
                var apparent = ApparentFromRaDec(target, now);
                var m = MountFromApparent(apparent, _mountPos.Az);
                CheckReachable(m, apparent);

                CancelManualMoves();
                _trackTarget = new RaDec(AstroMath.Wrap24(target.RaHours), target.DecDeg);
                _trackEpoch = now;
                _tracking = true;
                _trackTicks = 0;
                var job = new GotoJob { IsRaDec = true, TrackAfter = true, Description = target.ToString() };
                _log.Info("Goto " + target + " (apparent " + apparent + ", mount " + m + ").");
                IssueGoto(job, refine: false);
            }
        }

        /// <summary>Starts a goto to an apparent alt/az. Tracking is switched off. Returns at once.</summary>
        public void SlewToHorizontal(AltAz apparent)
        {
            lock (_sync)
            {
                RequireConnected();
                RequireUnparked();
                var m = MountFromApparent(apparent, _mountPos.Az);
                CheckReachable(m, apparent);
                CancelManualMoves();
                _tracking = false;
                var job = new GotoJob { IsRaDec = false, MountTarget = m, Description = apparent.ToString() };
                _log.Info("Goto " + apparent + " (mount " + m + ").");
                IssueGoto(job, refine: false);
            }
        }

        public void Park()
        {
            lock (_sync)
            {
                RequireConnected();
                if (_atPark || (_goto != null && _goto.Park)) return;
                CancelManualMoves();
                _tracking = false;
                double az = AstroMath.ToRad(_settings.ParkAzimuthDeg);
                double alt = AstroMath.ToRad(_settings.ParkAltitudeDeg);
                var job = new GotoJob { IsRaDec = false, Park = true, MountTarget = new AltAz(az, alt), Description = "park" };
                _log.Info("Parking at mount az " + _settings.ParkAzimuthDeg + "°, alt " + _settings.ParkAltitudeDeg + "°.");
                IssueGoto(job, refine: false);
            }
        }

        public void Unpark()
        {
            lock (_sync)
            {
                RequireConnected();
                if (!_atPark) return;
                _atPark = false;
                SaveState();
                _log.Info("Unparked.");
            }
        }

        /// <summary>Makes the current position the park position. The caller persists the returned settings.</summary>
        public MountSettings SetParkHere()
        {
            lock (_sync)
            {
                RequireConnected();
                ReadPosition();
                _settings.ParkAzimuthDeg = AstroMath.ToDeg(_mountPos.Az);
                _settings.ParkAltitudeDeg = AstroMath.ToDeg(_mountPos.Alt);
                _log.Info("Park position set to mount az " + _settings.ParkAzimuthDeg.ToString("F2") + "°, alt " + _settings.ParkAltitudeDeg.ToString("F2") + "°.");
                return _settings.Clone();
            }
        }

        public void AbortSlew()
        {
            lock (_sync)
            {
                RequireConnected();
                var job = _goto;
                _goto = null;
                bool wasMoving = _moving[1] || _moving[2];
                _moving[1] = _moving[2] = false;
                StopAndWait(_driveAz, force: true);
                StopAndWait(_driveAlt, force: true);
                ReadPosition();
                if (job != null) _log.Info("Goto to " + job.Description + " aborted.");
                if (job != null && job.Park) _tracking = false;
                if (_tracking && (job != null || wasMoving))
                {
                    CaptureTrackTarget();
                    UpdateTracking();
                }
            }
        }

        private void IssueGoto(GotoJob job, bool refine)
        {
            ReadPosition();
            AltAz m;
            if (job.IsRaDec)
            {
                // Aim where the target will be when the slew finishes.
                var now = _clock.UtcNow;
                m = TrackedMountAt(now);
                double lead = EstimateSlewSeconds(m);
                m = TrackedMountAt(now.AddSeconds(lead));
                if (double.IsNaN(m.Az)) m = TrackedMountAt(now);
            }
            else
            {
                m = job.MountTarget;
            }

            GotoAxis(Axis.Azimuth, m.Az);
            GotoAxis(Axis.Altitude, m.Alt);
            job.Stage++;
            job.StartedUtc = _clock.UtcNow;
            _goto = job;
            if (refine) _log.Debug("Refining goto, stage " + job.Stage + ".");
        }

        private AltAz TrackedMountAt(DateTime utc)
        {
            return MountFromApparent(ApparentFromRaDec(TrackTargetAt(utc), utc), _mountPos.Az);
        }

        private double EstimateSlewSeconds(AltAz target)
        {
            double d = Math.Max(Math.Abs(target.Az - _mountPos.Az), Math.Abs(target.Alt - _mountPos.Alt));
            if (double.IsNaN(d)) return 0;
            return AstroMath.ToDeg(d) / 3.0 + 3.0;
        }

        private void GotoAxis(Axis axis, double angle)
        {
            var drive = Drive(axis);
            int target = AngleToSteps(axis, angle);
            int current = _mc.InquirePosition(axis);
            int delta = target - current;
            StopAndWait(drive, force: true);
            if (Math.Abs(delta) < 2) return;

            int abs = Math.Abs(delta);
            int margin = LowSpeedGotoMargin(axis);
            var mode = abs > margin ? MotionMode.GotoFast : MotionMode.GotoSlow;
            _mc.SetMotionMode(axis, mode, delta < 0);
            _mc.SetGotoIncrement(axis, abs);
            _mc.SetBrakeIncrement(axis, abs > margin ? abs - margin : abs);
            _mc.Start(axis);
            drive.Running = true;
            drive.Mode = mode;
            drive.Reverse = delta < 0;
            drive.Period = 0;
        }

        /// <summary>Gotos shorter than this run entirely at low speed (the same margin the INDI driver uses).</summary>
        private int LowSpeedGotoMargin(Axis axis) => (int)(640 * AstroMath.EarthRateRadPerSec * Params(axis).StepsPerRadian);

        private void CheckReachable(AltAz mount, AltAz apparent)
        {
            if (double.IsNaN(mount.Az))
                throw new ArgumentOutOfRangeException("target", "Azimuth " + AstroMath.ToDeg(apparent.Az).ToString("F1") + "° is outside the allowed azimuth travel.");
            double alt = AstroMath.ToDeg(mount.Alt);
            if (alt < _settings.MinAltitudeDeg || alt > _settings.MaxAltitudeDeg)
                throw new ArgumentOutOfRangeException("target", "Altitude " + AstroMath.ToDeg(apparent.Alt).ToString("F1") + "° is outside the limits ("
                    + _settings.MinAltitudeDeg + "° to " + _settings.MaxAltitudeDeg + "°).");
        }

        // ------------------------------------------------------------------ manual moves

        /// <summary>Moves an axis continuously at a rate in degrees per second (0 stops). Positive is increasing angle.</summary>
        public void MoveAxis(Axis axis, double degPerSec)
        {
            lock (_sync)
            {
                RequireConnected();
                RequireUnparked();
                if (axis != Axis.Azimuth && axis != Axis.Altitude) throw new ArgumentException("Only the azimuth and altitude axes can be moved.");
                if (Math.Abs(degPerSec) > _settings.MaxMoveRateDegPerSec + 1e-9)
                    throw new ArgumentOutOfRangeException(nameof(degPerSec), "Rate must be at most " + _settings.MaxMoveRateDegPerSec + "°/s.");

                if (_goto != null)
                {
                    _goto = null;
                    StopAndWait(_driveAz, force: true);
                    StopAndWait(_driveAlt, force: true);
                }

                int i = (int)axis;
                if (degPerSec == 0)
                {
                    if (!_moving[i]) return;
                    _moving[i] = false;
                    StopAndWait(Drive(axis), force: true);
                    if (!_moving[1] && !_moving[2] && _tracking)
                    {
                        ReadPosition();
                        CaptureTrackTarget();
                        UpdateTracking();
                    }
                    return;
                }

                ReadPosition();
                if (WouldPassLimit(axis, degPerSec)) throw new InvalidOperationException("That direction is blocked by the " + axis + " limit.");
                _moving[i] = true;
                ApplyRate(axis, AstroMath.ToRad(degPerSec));
            }
        }

        private bool WouldPassLimit(Axis axis, double rate)
        {
            if (axis == Axis.Altitude)
            {
                double alt = AstroMath.ToDeg(_mountPos.Alt);
                return (rate < 0 && alt <= _settings.MinAltitudeDeg) || (rate > 0 && alt >= _settings.MaxAltitudeDeg);
            }
            double az = AstroMath.ToDeg(_mountPos.Az) - _settings.StartAzimuthDeg;
            return (rate < 0 && az <= -_settings.AzimuthTravelDeg) || (rate > 0 && az >= _settings.AzimuthTravelDeg);
        }

        private void CancelManualMoves()
        {
            _moving[1] = _moving[2] = false;
        }

        // ------------------------------------------------------------------ sync and model

        public void SyncToEquatorial(RaDec rd)
        {
            lock (_sync)
            {
                RequireConnected();
                RequireUnparked();
                ReadPosition();
                var now = _clock.UtcNow;
                AddSync(ApparentFromRaDec(rd, now), now);
                if (_tracking)
                {
                    _trackTarget = rd;
                    _trackEpoch = now;
                    _trackTicks = 0;
                }
            }
        }

        public void SyncToHorizontal(AltAz apparent)
        {
            lock (_sync)
            {
                RequireConnected();
                RequireUnparked();
                ReadPosition();
                var now = _clock.UtcNow;
                AddSync(apparent, now);
                if (_tracking) CaptureTrackTarget();
            }
        }

        private void AddSync(AltAz apparent, DateTime now)
        {
            var mount = new AltAz(AstroMath.Wrap2Pi(_mountPos.Az), _mountPos.Alt);
            var before = ApparentFromMount(_mountPos);
            string note = Model.AddPoint(apparent, mount, now);
            _log.Info("Sync to " + apparent + " (mount said " + before + ", off by "
                + (AstroMath.ToDeg(PointingModel.Separation(before, apparent)) * 60).ToString("F1") + "′). " + note);
            SaveState();
        }

        public void ClearModel()
        {
            lock (_sync)
            {
                Model.Clear();
                SaveState();
                _log.Info("Pointing model cleared.");
            }
        }

        // ------------------------------------------------------------------ the loop

        private void Loop()
        {
            while (_run)
            {
                var started = DateTime.UtcNow;
                Tick();
                int interval;
                lock (_sync) interval = _settings.TrackingIntervalMs;
                var wait = TimeSpan.FromMilliseconds(interval) - (DateTime.UtcNow - started);
                if (wait > TimeSpan.Zero) Thread.Sleep(wait);
            }
        }

        /// <summary>One control cycle. Called by the background loop, or directly by tests.</summary>
        public void Tick()
        {
            lock (_sync)
            {
                if (_mc == null) return;
                try
                {
                    ReadPosition();
                    if (_goto != null) SuperviseGoto();
                    else if (_moving[1] || _moving[2]) SuperviseManualMove();

                    if (_tracking && _goto == null && !_moving[1] && !_moving[2]) UpdateTracking();
                    _consecutiveErrors = 0;
                }
                catch (Exception ex)
                {
                    _consecutiveErrors++;
                    LastError = ex.Message;
                    if (_consecutiveErrors <= 3 || _consecutiveErrors % 50 == 0) _log.Error("Control loop error (" + _consecutiveErrors + " in a row)", ex);
                }
            }
        }

        private void SuperviseGoto()
        {
            var job = _goto;
            if (job.Settling)
            {
                UpdateTracking();
                if (!_tracking)
                {
                    _goto = null;
                    return;
                }
                double err = AngularError(TrackedMountAt(_clock.UtcNow), _mountPos);
                double waited = (_clock.UtcNow - job.SettleStartedUtc).TotalSeconds;
                if (err <= AstroMath.ToRad(_settings.GotoToleranceArcsec / 3600.0) || waited > 30)
                {
                    _log.Info("Arrived at " + job.Description + " after " + job.Stage + " goto stage(s) and " + waited.ToString("F1")
                        + " s settling, " + (AstroMath.ToDeg(err) * 3600).ToString("F0") + "″ from target; tracking.");
                    _goto = null;
                }
                return;
            }
            var sAz = _mc.InquireStatus(Axis.Azimuth);
            var sAlt = _mc.InquireStatus(Axis.Altitude);
            if (sAz.Running || sAlt.Running)
            {
                if ((_clock.UtcNow - job.StartedUtc).TotalSeconds > 300)
                {
                    _log.Error("Goto to " + job.Description + " took over 5 minutes; stopping.");
                    _goto = null;
                    StopAndWait(_driveAz, force: true);
                    StopAndWait(_driveAlt, force: true);
                    LastError = "Goto timed out.";
                }
                return;
            }
            _driveAz.Running = false;
            _driveAlt.Running = false;

            if (job.IsRaDec)
            {
                var m = TrackedMountAt(_clock.UtcNow);
                double err = AngularError(m, _mountPos);
                // A long slew can land a little off (the target moved more than predicted): go again.
                // Small residuals are pulled in by the tracking loop while Slewing stays true.
                if (err > AstroMath.ToRad(0.25) && job.Stage < 3 && !double.IsNaN(m.Az))
                {
                    IssueGoto(job, refine: true);
                    return;
                }
                job.Settling = true;
                job.SettleStartedUtc = _clock.UtcNow;
                _tracking = job.TrackAfter;
                _trackTicks = 0;
                return;
            }

            _goto = null;
            if (job.Park)
            {
                _atPark = true;
                _tracking = false;
                SaveState();
                _log.Info("Parked.");
            }
            else
            {
                _log.Info("Arrived at " + job.Description + ".");
            }
        }

        private void SuperviseManualMove()
        {
            foreach (var axis in new[] { Axis.Azimuth, Axis.Altitude })
            {
                if (!_moving[(int)axis]) continue;
                var drive = Drive(axis);
                double rate = drive.Reverse ? -Sign(axis) : Sign(axis);
                if (WouldPassLimit(axis, rate))
                {
                    _log.Warn("Manual move stopped at the " + axis + " limit.");
                    _moving[(int)axis] = false;
                    StopAndWait(drive, force: true);
                }
            }
        }

        private void UpdateTracking()
        {
            var now = _clock.UtcNow;
            double lead = Math.Max(_settings.TrackingIntervalMs / 1000.0, 0.25);
            var t0 = TrackedMountAt(now);
            var t1 = TrackedMountAt(now.AddSeconds(lead));

            if (_settings.ManualMoveThresholdDeg > 0 && _trackTicks > 4 && !double.IsNaN(t0.Az)
                && AngularError(t0, _mountPos) > AstroMath.ToRad(_settings.ManualMoveThresholdDeg))
            {
                _log.Info("Tube moved " + AstroMath.ToDeg(AngularError(t0, _mountPos)).ToString("F2")
                    + "° without being told to (pushed by hand?); tracking from the new position.");
                CaptureTrackTarget();
                t1 = TrackedMountAt(now.AddSeconds(lead));
            }

            double altDeg = AstroMath.ToDeg(t1.Alt);
            if (double.IsNaN(t1.Az) || altDeg < _settings.MinAltitudeDeg || altDeg > _settings.MaxAltitudeDeg)
            {
                _tracking = false;
                StopAndWait(_driveAz, force: true);
                StopAndWait(_driveAlt, force: true);
                LastError = double.IsNaN(t1.Az)
                    ? "Tracking stopped: the target would take the azimuth past its travel limit."
                    : "Tracking stopped: the target reached the altitude limit (" + altDeg.ToString("F1") + "°).";
                _log.Warn(LastError);
                return;
            }

            ApplyRate(Axis.Azimuth, (t1.Az - _mountPos.Az) / lead);
            ApplyRate(Axis.Altitude, (t1.Alt - _mountPos.Alt) / lead);
            _trackTicks++;
        }

        /// <summary>Sets an axis running at a signed rate in radians per second of axis angle.</summary>
        private void ApplyRate(Axis axis, double radPerSec)
        {
            var drive = Drive(axis);
            var p = Params(axis);
            double sps = radPerSec * p.StepsPerRadian * Sign(axis);
            double abs = Math.Abs(sps);
            bool reverse = sps < 0;

            // Above this the board must run in its high-speed mode (same threshold as INDI).
            double fastAbove = 128 * AstroMath.EarthRateRadPerSec * p.StepsPerRadian;
            bool fast = abs > fastAbove;
            double periodD = fast ? (double)p.TimerFrequency * p.HighSpeedRatio / abs : p.TimerFrequency / Math.Max(abs, 1e-9);
            if (abs < 1e-3 || periodD > 0xFFFFFF)
            {
                if (drive.Running) StopAndWait(drive, force: false);
                return;
            }
            int period = Math.Max(1, (int)Math.Round(periodD));
            var mode = fast ? MotionMode.SlewFast : MotionMode.SlewSlow;

            if (drive.Running && drive.Mode == mode && drive.Reverse == reverse)
            {
                if (drive.Period != period)
                {
                    _mc.SetStepPeriod(axis, period);
                    drive.Period = period;
                }
                return;
            }

            StopAndWait(drive, force: drive.Running);
            _mc.SetMotionMode(axis, mode, reverse);
            _mc.SetStepPeriod(axis, period);
            _mc.Start(axis);
            drive.Running = true;
            drive.Mode = mode;
            drive.Reverse = reverse;
            drive.Period = period;
        }

        /// <summary>Stops an axis and waits until the board reports it stopped.</summary>
        /// <param name="force">Ask the board even if we believe the axis is already stopped.</param>
        private void StopAndWait(AxisDrive drive, bool force)
        {
            if (!drive.Running && !force) return;
            var axis = drive.Axis;
            if (!_mc.InquireStatus(axis).Running)
            {
                drive.Running = false;
                return;
            }
            _mc.Stop(axis);
            var deadline = _clock.UtcNow.AddSeconds(10);
            while (_clock.UtcNow < deadline)
            {
                if (!_mc.InquireStatus(axis).Running)
                {
                    drive.Running = false;
                    return;
                }
                _clock.Sleep(TimeSpan.FromMilliseconds(50));
            }
            _log.Warn(axis + " did not stop within 10 s; forcing an instant stop.");
            _mc.InstantStop(axis);
            drive.Running = false;
        }

        private static double AngularError(AltAz a, AltAz b)
        {
            double dAz = (a.Az - b.Az) * Math.Cos(b.Alt);
            double dAlt = a.Alt - b.Alt;
            return Math.Sqrt(dAz * dAz + dAlt * dAlt);
        }

        private void RequireConnected()
        {
            if (_mc == null) throw new InvalidOperationException("Not connected to the mount.");
        }

        private void RequireUnparked()
        {
            if (_atPark) throw new InvalidOperationException("The mount is parked. Unpark it first.");
        }

        private void ApplyModelSettings()
        {
            Model.ResetThreshold = AstroMath.ToRad(_settings.ModelResetThresholdDeg);
        }

        // ------------------------------------------------------------------ persistence

        private string StatePath => _stateDirectory == null ? null : Path.Combine(_stateDirectory, "state.json");

        private void SaveState()
        {
            if (StatePath == null) return;
            try
            {
                var s = new SessionState { AtPark = _atPark, Points = new List<SyncPoint>(Model.Points) };
                Directory.CreateDirectory(_stateDirectory);
                using (var fs = File.Create(StatePath))
                {
                    new DataContractJsonSerializer(typeof(SessionState)).WriteObject(fs, s);
                }
            }
            catch (Exception ex)
            {
                _log.Warn("Could not save state: " + ex.Message);
            }
        }

        private void LoadState()
        {
            if (StatePath == null || !File.Exists(StatePath)) return;
            try
            {
                SessionState s;
                using (var fs = File.OpenRead(StatePath))
                {
                    s = (SessionState)new DataContractJsonSerializer(typeof(SessionState)).ReadObject(fs);
                }
                _atPark = s.AtPark;
                Model.ReplacePoints(s.Points ?? new List<SyncPoint>());
            }
            catch (Exception ex)
            {
                _log.Warn("Could not restore state: " + ex.Message);
            }
        }

        [DataContract]
        private sealed class SessionState
        {
            [DataMember] public bool AtPark;
            [DataMember] public List<SyncPoint> Points;
        }

        private sealed class AxisDrive
        {
            public AxisDrive(Axis axis) { Axis = axis; }
            public readonly Axis Axis;
            public bool Running;
            public MotionMode Mode;
            public bool Reverse;
            public int Period;
        }

        private sealed class GotoJob
        {
            public bool IsRaDec;
            public bool TrackAfter;
            public bool Park;
            public AltAz MountTarget;
            public int Stage;
            public DateTime StartedUtc;
            /// <summary>Motors have arrived; tracking is pulling in the last few arcseconds.</summary>
            public bool Settling;
            public DateTime SettleStartedUtc;
            public string Description;
        }

        /// <summary>Keeps the shared simulator alive across connect/disconnect.</summary>
        private sealed class NonDisposingTransport : ITransport
        {
            private readonly ITransport _inner;
            public NonDisposingTransport(ITransport inner) { _inner = inner; }
            public string Transact(string command) => _inner.Transact(command);
            public string Description => _inner.Description;
            public void Dispose() { }
        }
    }
}
