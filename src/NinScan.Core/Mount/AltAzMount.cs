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

        private AltAz _mountPos;        // tube axis angles (motor angles less gear play), azimuth unwrapped
        private AltAz _rawPos;          // motor axis angles from the step counts
        private DateTime _posTime;
        private readonly GearState _gearAz = new GearState();
        private readonly GearState _gearAlt = new GearState();
        private LearnedMechanics _mech = new LearnedMechanics();
        private SyncSnapshot _lastSync;
        private bool _pushSinceSync;
        private bool _solarViewing;

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
            _mech = LearnedMechanics.Load(MechanicsPath);
            ApplyModelSettings();
        }

        /// <summary>Backlash and clutch-slip knowledge carried across sessions.</summary>
        public LearnedMechanics Mechanics
        {
            get { lock (_sync) return _mech.Clone(); }
        }

        /// <summary>What the last sync revealed about how the mount really moved, for display.</summary>
        public string LastSyncReport { get; private set; }

        /// <summary>Forgets learned backlash and slip counts (for example after adjusting the clutches).</summary>
        public void ResetLearnedMechanics()
        {
            lock (_sync)
            {
                _mech.Reset();
                SaveMechanics();
                ApplyModelSettings();
                _log.Info("Learned backlash and slip counts reset.");
            }
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
                        _gearAz.Reset();
                        _gearAlt.Reset();
                        _lastSync = null;
                        _mech.RollSession();
                        SaveMechanics();
                        ApplyModelSettings();
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
                    _solarViewing = false;
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
            _rawPos = new AltAz(StepsToAngle(Axis.Azimuth, az), StepsToAngle(Axis.Altitude, alt));
            _gearAz.Update(_rawPos.Az, Model.BacklashAz);
            _gearAlt.Update(_rawPos.Alt, Model.BacklashAlt);
            _mountPos = new AltAz(_rawPos.Az - _gearAz.Play, _rawPos.Alt - _gearAlt.Play);
            _posTime = _clock.UtcNow;
        }

        private GearState Gear(Axis axis) => axis == Axis.Azimuth ? _gearAz : _gearAlt;

        private double Backlash(Axis axis) => axis == Axis.Azimuth ? Model.BacklashAz : Model.BacklashAlt;

        /// <summary>
        /// Motor movement needed for the tube to move by <paramref name="tubeDelta"/>: in a new direction
        /// the motor first has to take up the gear play.
        /// </summary>
        private double MotorDeltaFor(Axis axis, double tubeDelta)
        {
            if (tubeDelta == 0) return 0;
            double half = Backlash(axis) / 2;
            double target = tubeDelta > 0 ? half : -half;
            return tubeDelta + (target - Gear(axis).Play);
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
                CheckSunTarget(apparent);
                var route = PlanRoute(_mountPos, m);

                CancelManualMoves();
                _trackTarget = new RaDec(AstroMath.Wrap24(target.RaHours), target.DecDeg);
                _trackEpoch = now;
                _tracking = true;
                _trackTicks = 0;
                var job = new GotoJob { IsRaDec = true, TrackAfter = true, Description = target.ToString(), Waypoints = route };
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
                CheckSunTarget(apparent);
                var route = PlanRoute(_mountPos, m);
                CancelManualMoves();
                _tracking = false;
                var job = new GotoJob { IsRaDec = false, MountTarget = m, Description = apparent.ToString(), Waypoints = route };
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
                var route = PlanRoute(_mountPos, new AltAz(az, alt));
                var job = new GotoJob { IsRaDec = false, Park = true, MountTarget = new AltAz(az, alt), Description = "park", Waypoints = route };
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
            if (job.Stage == 0 && job.Waypoints != null && job.Waypoints.Count > 0)
            {
                // Detour around the Sun: visit each waypoint before heading for the target.
                var w = job.Waypoints.Dequeue();
                _log.Info("Detouring via mount az " + AstroMath.ToDeg(w.Az).ToString("F1") + "°, alt " + AstroMath.ToDeg(w.Alt).ToString("F1") + "° to keep clear of the Sun.");
                GotoAxis(Axis.Azimuth, w.Az);
                GotoAxis(Axis.Altitude, w.Alt);
                job.StartedUtc = _clock.UtcNow;
                job.StartSunSeparationDeg = SunSeparationDeg(_mountPos);
                _goto = job;
                return;
            }
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

            if (job.Stage == 0) ChooseApproach(job, m);
            GotoWithApproach(job, Axis.Azimuth, m.Az);
            GotoWithApproach(job, Axis.Altitude, m.Alt);
            job.Stage++;
            job.StartedUtc = _clock.UtcNow;
            job.StartSunSeparationDeg = SunSeparationDeg(_mountPos);
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

        /// <summary>Decides which way each axis makes its final move for this goto.</summary>
        private void ChooseApproach(GotoJob job, AltAz target)
        {
            var mode = _settings.Approach;
            if (mode == ApproachMode.TrackingDirection && !job.IsRaDec) mode = ApproachMode.UpAndRight;
            switch (mode)
            {
                case ApproachMode.Off:
                    job.ApproachAz = job.ApproachAlt = 0;
                    break;
                case ApproachMode.UpAndRight:
                    job.ApproachAz = job.ApproachAlt = 1;
                    break;
                case ApproachMode.DownAndLeft:
                    job.ApproachAz = job.ApproachAlt = -1;
                    break;
                default:
                {
                    // The direction each axis will be moving a minute after arrival.
                    var arrive = _clock.UtcNow.AddSeconds(EstimateSlewSeconds(target));
                    var a = TrackedMountAt(arrive);
                    var b = TrackedMountAt(arrive.AddSeconds(60));
                    double tiny = AstroMath.ToRad(1.0 / 3600);
                    job.ApproachAz = double.IsNaN(a.Az) || double.IsNaN(b.Az) || Math.Abs(b.Az - a.Az) < tiny ? 1 : Math.Sign(b.Az - a.Az);
                    job.ApproachAlt = Math.Abs(b.Alt - a.Alt) < tiny ? 1 : Math.Sign(b.Alt - a.Alt);
                    break;
                }
            }
        }

        /// <summary>
        /// Starts the goto on one axis. If the final move would come from the wrong side, it first stops
        /// short on the far side and the second leg (issued when the axes stop) comes in the right way.
        /// </summary>
        private void GotoWithApproach(GotoJob job, Axis axis, double tubeTarget)
        {
            int approach = axis == Axis.Azimuth ? job.ApproachAz : job.ApproachAlt;
            double now = axis == Axis.Azimuth ? _mountPos.Az : _mountPos.Alt;
            double delta = tubeTarget - now;
            double overshoot = Math.Max(AstroMath.ToRad(_settings.MinApproachOvershootDeg), 2 * Backlash(axis));
            SetPending(job, axis, false);

            if (approach != 0 && Math.Abs(delta) > AstroMath.ToRad(0.5 / 3600) && Math.Sign(delta) != approach)
            {
                double first = tubeTarget - approach * overshoot;
                if (WithinTravel(axis, first))
                {
                    GotoAxis(axis, first);
                    SetPending(job, axis, true);
                    return;
                }
            }
            GotoAxis(axis, tubeTarget);
        }

        private static void SetPending(GotoJob job, Axis axis, bool pending)
        {
            if (axis == Axis.Azimuth) job.PendingAz = pending; else job.PendingAlt = pending;
        }

        private bool WithinTravel(Axis axis, double tubeAngle)
        {
            double deg = AstroMath.ToDeg(tubeAngle);
            if (axis == Axis.Altitude) return deg >= _settings.MinAltitudeDeg && deg <= _settings.MaxAltitudeDeg;
            double fromStart = deg - _settings.StartAzimuthDeg;
            return Math.Abs(fromStart) <= _settings.AzimuthTravelDeg;
        }

        /// <summary>Moves an axis so the tube (not just the motor) ends at <paramref name="tubeAngle"/>.</summary>
        private void GotoAxis(Axis axis, double tubeAngle)
        {
            var drive = Drive(axis);
            double tubeNow = axis == Axis.Azimuth ? _mountPos.Az : _mountPos.Alt;
            double rawNow = axis == Axis.Azimuth ? _rawPos.Az : _rawPos.Alt;
            int target = AngleToSteps(axis, rawNow + MotorDeltaFor(axis, tubeAngle - tubeNow));
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

        // ------------------------------------------------------------------ Sun safety

        /// <summary>
        /// Solar viewing: when off (the default at every connect), the telescope is kept away from the Sun.
        /// Only switch on with a proper solar filter over the front of the telescope and the finder capped.
        /// </summary>
        public bool SolarViewing
        {
            get { lock (_sync) return _solarViewing; }
            set
            {
                lock (_sync)
                {
                    if (_solarViewing == value) return;
                    _solarViewing = value;
                    if (value) _log.Warn("Solar viewing switched ON: the telescope may now point at the Sun. A solar filter must be fitted.");
                    else _log.Info("Solar viewing switched off: keeping " + _settings.SunAvoidanceDeg + "° from the Sun.");
                }
            }
        }

        /// <summary>Apparent position of the Sun now.</summary>
        public AltAz SunPosition => ApparentFromRaDec(AstroMath.SunPosition(_clock.UtcNow), _clock.UtcNow);

        /// <summary>True between sunrise and sunset at the configured site.</summary>
        public bool IsSunUp => AstroMath.ToDeg(SunPosition.Alt) > -0.833;

        /// <summary>The Sun is up and solar viewing is off: the moment to ask whether solar viewing is intended.</summary>
        public bool ShouldOfferSolarViewing
        {
            get { lock (_sync) return !_solarViewing && IsSunUp; }
        }

        private double SunSeparationDeg(AltAz tube)
        {
            return AstroMath.ToDeg(PointingModel.Separation(ApparentFromMount(tube), SunPosition));
        }

        /// <summary>Within <paramref name="fraction"/> of the avoidance radius of the Sun, with solar viewing off and the Sun not well below the horizon.</summary>
        private bool SunTooClose(AltAz tube, double fraction)
        {
            if (_solarViewing) return false;
            var sun = SunPosition;
            if (AstroMath.ToDeg(sun.Alt) < -_settings.SunAvoidanceDeg) return false;
            return AstroMath.ToDeg(PointingModel.Separation(ApparentFromMount(tube), sun)) < _settings.SunAvoidanceDeg * fraction;
        }

        private void CheckSunTarget(AltAz apparent)
        {
            if (_solarViewing) return;
            var sun = SunPosition;
            if (AstroMath.ToDeg(sun.Alt) < -_settings.SunAvoidanceDeg) return;
            double sep = AstroMath.ToDeg(PointingModel.Separation(apparent, sun));
            if (sep < _settings.SunAvoidanceDeg)
                throw new InvalidOperationException("That target is " + sep.ToString("F1") + "° from the Sun. Solar viewing is off, so NinScan keeps "
                    + _settings.SunAvoidanceDeg + "° away. Turn solar viewing on (with a solar filter fitted) to point there.");
        }

        private bool MovesTowardSun(Axis axis, double rate)
        {
            if (!SunTooClose(_mountPos, 1.0)) return false;
            var ahead = _mountPos;
            double step = AstroMath.ToRad(0.5) * Math.Sign(rate);
            if (axis == Axis.Azimuth) ahead.Az += step; else ahead.Alt += step;
            return SunSeparationDeg(ahead) < SunSeparationDeg(_mountPos);
        }

        /// <summary>
        /// Checks the path a goto would take (both axes moving at the same speed) and finds a detour if it
        /// would pass the Sun: altitude first, azimuth first, or round underneath or over the top.
        /// Returns the waypoints to visit, empty when the direct path is clear.
        /// </summary>
        private Queue<AltAz> PlanRoute(AltAz from, AltAz to)
        {
            var none = new Queue<AltAz>();
            if (_solarViewing || AstroMath.ToDeg(SunPosition.Alt) < -_settings.SunAvoidanceDeg) return none;
            if (PathClear(from, to)) return none;

            double low = AstroMath.ToRad(_settings.MinAltitudeDeg + 2);
            double high = AstroMath.ToRad(_settings.MaxAltitudeDeg - 2);
            var options = new[]
            {
                new[] { new AltAz(from.Az, to.Alt) },
                new[] { new AltAz(to.Az, from.Alt) },
                new[] { new AltAz(from.Az, low), new AltAz(to.Az, low) },
                new[] { new AltAz(from.Az, high), new AltAz(to.Az, high) },
            };
            foreach (var option in options)
            {
                var prev = from;
                bool ok = true;
                foreach (var w in option)
                {
                    if (!PathClear(prev, w)) { ok = false; break; }
                    prev = w;
                }
                if (ok && PathClear(prev, to)) return new Queue<AltAz>(option);
            }
            throw new InvalidOperationException("Could not find a path to that target that stays " + _settings.SunAvoidanceDeg
                + "° from the Sun. Solar viewing is off.");
        }

        /// <summary>
        /// True if the path never comes within the avoidance radius of the Sun. A path that starts inside the
        /// radius (solar viewing was just switched off) only fails if it gets closer still, so it can always leave.
        /// </summary>
        private bool PathClear(AltAz from, AltAz to)
        {
            double startSep = SunSeparationDeg(from);
            double dAz = to.Az - from.Az, dAlt = to.Alt - from.Alt;
            double longest = Math.Max(Math.Abs(dAz), Math.Abs(dAlt));
            int n = Math.Max(10, (int)(AstroMath.ToDeg(longest) * 2));
            for (int i = 0; i <= n; i++)
            {
                double s = longest * i / n;
                var p = new AltAz(from.Az + Math.Sign(dAz) * Math.Min(s, Math.Abs(dAz)), from.Alt + Math.Sign(dAlt) * Math.Min(s, Math.Abs(dAlt)));
                if (SunTooClose(p, 1.0) && SunSeparationDeg(p) < startSep - 0.01) return false;
            }
            return true;
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
                if (MovesTowardSun(axis, degPerSec))
                    throw new InvalidOperationException("That direction is blocked: the telescope is within " + _settings.SunAvoidanceDeg
                        + "° of the Sun and solar viewing is off.");
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
            var before = ApparentFromMount(_mountPos);
            var point = new SyncPoint
            {
                SkyAz = apparent.Az, SkyAlt = apparent.Alt,
                MountAz = AstroMath.Wrap2Pi(_rawPos.Az), MountAlt = _rawPos.Alt, TimeUtc = now,
                PlayAz = _gearAz.Play, PlayAlt = _gearAlt.Play,
                HistoryAz = _gearAz.History(), HistoryAlt = _gearAlt.History(),
            };

            string motion = AnalyseMotionSinceLastSync(apparent, now);
            string note = Model.AddPoint(point);
            _log.Info("Sync to " + apparent + " (mount said " + before + ", off by "
                + (AstroMath.ToDeg(PointingModel.Separation(before, apparent)) * 60).ToString("F1") + "′). " + note);
            if (motion != null) _log.Info(motion);

            _lastSync = new SyncSnapshot
            {
                Sky = apparent, RawAz = _rawPos.Az, RawAlt = _rawPos.Alt, PlayAz = _gearAz.Play, PlayAlt = _gearAlt.Play,
                HistoryAz = _gearAz.History(), HistoryAlt = _gearAlt.History(), Utc = now,
            };
            _pushSinceSync = false;
            LearnFromModel(now);
            SaveState();
        }

        /// <summary>
        /// Compares how far the motors moved since the previous sync with how far the plate solves say the
        /// tube moved. Gear play explains some of the difference; anything left over means the tube did not
        /// follow the motors (a slipping clutch or a stall) and is reported.
        /// </summary>
        private string AnalyseMotionSinceLastSync(AltAz apparent, DateTime now)
        {
            var last = _lastSync;
            if (last == null || _pushSinceSync || Model.PointCount == 0 || (now - last.Utc).TotalMinutes > 30) return null;

            var oPrev = Model.SkyToMount(last.Sky);
            var oNow = Model.SkyToMount(apparent);
            double measuredAz = AstroMath.WrapPi(oNow.Az - oPrev.Az);
            double measuredAlt = oNow.Alt - oPrev.Alt;
            double motorAz = _rawPos.Az - last.RawAz;
            double motorAlt = _rawPos.Alt - last.RawAlt;
            double expectedAz = motorAz - (_gearAz.Play - last.PlayAz);
            double expectedAlt = motorAlt - (_gearAlt.Play - last.PlayAlt);
            double lostAz = (expectedAz - measuredAz) * Math.Cos(apparent.Alt);
            double lostAlt = expectedAlt - measuredAlt;

            string Fmt(double r) => (AstroMath.ToDeg(r) * 60).ToString("+0.0;-0.0") + "′";
            var report = "Since the last sync: azimuth motors " + Fmt(motorAz) + ", tube " + Fmt(measuredAz)
                + "; altitude motors " + Fmt(motorAlt) + ", tube " + Fmt(measuredAlt) + ".";

            var warnings = new List<string>();
            CheckLostMotion(0, "azimuth", lostAz, SameLoading(last.HistoryAz, _gearAz.History()), Model.RmsAz, warnings);
            CheckLostMotion(1, "altitude", lostAlt, SameLoading(last.HistoryAlt, _gearAlt.History()), Model.RmsAlt, warnings);
            if (warnings.Count > 0)
            {
                report += " " + string.Join(" ", warnings);
                _log.Warn(string.Join(" ", warnings));
                SaveMechanics();
            }
            LastSyncReport = report;
            return report;
        }

        /// <summary>
        /// True when the gears were loaded the same way at both syncs whatever the backlash is, so backlash
        /// cannot account for any difference between motor and tube movement.
        /// </summary>
        private static bool SameLoading(double[] before, double[] after)
        {
            if (before == null || after == null) return false;
            foreach (double deg in new[] { 0.2, 0.5, PointingModel.MaxBacklash * 180 / Math.PI })
            {
                double b = AstroMath.ToRad(deg);
                if (Math.Abs(PointingModel.PlayOf(before, 0, b) - PointingModel.PlayOf(after, 0, b)) > AstroMath.ToRad(1.0 / 3600)) return false;
            }
            return true;
        }

        private void CheckLostMotion(int axis, string name, double lost, bool sameLoading, double rms, List<string> warnings)
        {
            double threshold = Math.Max(AstroMath.ToRad(3.0 / 60), 3 * rms);
            if (Math.Abs(lost) <= threshold) return;
            string amount = (AstroMath.ToDeg(Math.Abs(lost)) * 60).ToString("F1") + "′";
            string motors = "The " + name + " motors moved " + amount + (lost > 0 ? " more" : " less") + " than the tube did";
            bool backlashKnown = Model.BacklashFitted(axis) || _settings.Backlash != BacklashMode.Learn;
            if (!sameLoading && !backlashKnown)
            {
                // The gears changed direction and the backlash is not pinned down yet: could be either.
                warnings.Add(motors + " after a change of direction. That is probably backlash still being learned; if it keeps happening, check the clutch.");
                return;
            }
            _mech.SlipEvents[axis]++;
            warnings.Add(motors + (sameLoading ? ", with the gears loaded the same way both times, so it is not backlash" : ", more than the learned backlash explains")
                + ": the clutch may be slipping or the tube is out of balance (" + _mech.SlipEvents[axis] + " time(s) so far).");
        }

        /// <summary>Folds the model's backlash fit into what is remembered across sessions.</summary>
        private void LearnFromModel(DateTime now)
        {
            if (_settings.Backlash != BacklashMode.Learn) return;
            bool changed = false;
            for (int i = 0; i < 2; i++)
            {
                if (!Model.BacklashFitted(i)) continue;
                double b = i == 0 ? Model.BacklashAz : Model.BacklashAlt;
                _mech.SetSession(i, b, Model.InformativePoints(i), now);
                changed = true;
            }
            if (!changed) return;
            SaveMechanics();
            _log.Info("Learned backlash: azimuth " + Arcmin(_mech.Effective(0)) + " (" + _mech.TotalWeight(0) + " syncs), altitude "
                + Arcmin(_mech.Effective(1)) + " (" + _mech.TotalWeight(1) + " syncs).");
        }

        private static string Arcmin(double rad) => (AstroMath.ToDeg(rad) * 60).ToString("F1") + "′";

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
                if (SunTooClose(_mountPos, 0.6) && SunSeparationDeg(_mountPos) < job.StartSunSeparationDeg - 1)
                {
                    _log.Error("Goto to " + job.Description + " came within " + SunSeparationDeg(_mountPos).ToString("F1") + "° of the Sun; stopped.");
                    _goto = null;
                    _tracking = false;
                    StopAndWait(_driveAz, force: true);
                    StopAndWait(_driveAlt, force: true);
                    LastError = "Stopped: the telescope came too close to the Sun.";
                    return;
                }
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

            if (job.Stage == 0)
            {
                // Reached a detour waypoint: on to the next one, or the target.
                IssueGoto(job, refine: false);
                return;
            }

            if (job.PendingAz || job.PendingAlt)
            {
                // Second leg: come in from the approach side.
                var t = job.IsRaDec ? TrackedMountAt(_clock.UtcNow.AddSeconds(2)) : job.MountTarget;
                if (job.PendingAz) { job.PendingAz = false; if (!double.IsNaN(t.Az)) GotoAxis(Axis.Azimuth, t.Az); }
                if (job.PendingAlt) { job.PendingAlt = false; GotoAxis(Axis.Altitude, t.Alt); }
                return;
            }

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
                else if (MovesTowardSun(axis, rate))
                {
                    _log.Warn("Manual move stopped " + SunSeparationDeg(_mountPos).ToString("F1") + "° from the Sun (solar viewing is off).");
                    LastError = "Manual move stopped near the Sun.";
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
                _pushSinceSync = true;
                CaptureTrackTarget();
                t0 = TrackedMountAt(now);
                t1 = TrackedMountAt(now.AddSeconds(lead));
            }

            double altDeg = AstroMath.ToDeg(t1.Alt);
            if (!double.IsNaN(t1.Az) && SunTooClose(t1, 1.0))
            {
                _tracking = false;
                StopAndWait(_driveAz, force: true);
                StopAndWait(_driveAlt, force: true);
                LastError = "Tracking stopped: the target came within " + _settings.SunAvoidanceDeg + "° of the Sun and solar viewing is off.";
                _log.Warn(LastError);
                return;
            }
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

            ApplyRate(Axis.Azimuth, TrackingMotorRate(Axis.Azimuth, t1.Az - t0.Az, t0.Az - _mountPos.Az, lead));
            ApplyRate(Axis.Altitude, TrackingMotorRate(Axis.Altitude, t1.Alt - t0.Alt, t0.Alt - _mountPos.Alt, lead));
            _trackTicks++;
        }

        /// <summary>Share of the position error corrected per cycle. Below 1 so the loop cannot overshoot and ring.</summary>
        private const double TrackingGain = 0.6;

        /// <summary>
        /// Fastest correction on top of the target's own motion, radians per second. Kept below the speed
        /// where the board needs to accelerate, so corrections take effect at once.
        /// </summary>
        private static readonly double MaxCorrectionRate = AstroMath.ToRad(0.2);

        /// <summary>
        /// Motor rate for one tracking cycle: the target's own motion plus part of the remaining error.
        /// When the axis has to reverse, the gear play is taken up quickly instead of the tube stalling for
        /// minutes. Tiny reversals are ignored so the gears are not rattled back and forth at a turning point.
        /// </summary>
        private double TrackingMotorRate(Axis axis, double targetMotion, double error, double lead)
        {
            double tubeDelta = targetMotion + TrackingGain * error;
            var gear = Gear(axis);
            double half = Backlash(axis) / 2;
            bool loadedThatWay = half <= 0 || (tubeDelta * gear.Play > 0 && Math.Abs(gear.Play) >= half - 1e-9);
            if (!loadedThatWay && Math.Abs(tubeDelta) < AstroMath.ToRad(2.0 / 3600)) return 0;
            double rate = MotorDeltaFor(axis, tubeDelta) / lead;
            double limit = Math.Abs(targetMotion / lead) + MaxCorrectionRate;
            return Math.Max(-limit, Math.Min(limit, rate));
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
            switch (_settings.Backlash)
            {
                case BacklashMode.Off:
                    Model.ConfigureBacklash(true, new double[2], new double[2], new double[2]);
                    break;
                case BacklashMode.Manual:
                    var manual = new[] { AstroMath.ToRad(_settings.ManualBacklashAzArcmin / 60), AstroMath.ToRad(_settings.ManualBacklashAltArcmin / 60) };
                    Model.ConfigureBacklash(true, manual, manual, new double[2]);
                    break;
                default:
                    Model.ConfigureBacklash(false, new[] { _mech.Effective(0), _mech.Effective(1) },
                        (double[])_mech.PriorBacklash.Clone(), (double[])_mech.PriorWeight.Clone());
                    break;
            }
        }

        // ------------------------------------------------------------------ persistence

        private string StatePath => _stateDirectory == null ? null : Path.Combine(_stateDirectory, "state.json");

        private string MechanicsPath => _stateDirectory == null ? null : Path.Combine(_stateDirectory, "learned.json");

        private void SaveMechanics()
        {
            try { _mech.Save(MechanicsPath); }
            catch (Exception ex) { _log.Warn("Could not save learned mechanics: " + ex.Message); }
        }

        private void SaveState()
        {
            if (StatePath == null) return;
            try
            {
                var s = new SessionState { AtPark = _atPark, Points = new List<SyncPoint>(Model.Points), GearAz = _gearAz, GearAlt = _gearAlt };
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
                _gearAz.CopyFrom(s.GearAz);
                _gearAlt.CopyFrom(s.GearAlt);
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
            [DataMember] public GearState GearAz;
            [DataMember] public GearState GearAlt;
        }

        /// <summary>
        /// Tracks the gear play of one axis from the motor movements. Play is motor angle minus tube angle,
        /// between -backlash/2 and +backlash/2: moving one way pushes it to one end, and the tube only moves
        /// once it gets there.
        /// </summary>
        [DataContract]
        internal sealed class GearState
        {
            private const int MaxHistory = 64;

            [DataMember] public double Play;
            [DataMember] public double LastRaw;
            [DataMember] public bool Initialised;
            /// <summary>Direction of the last move longer than <see cref="PointingModel.LearningMove"/>, 0 if none yet.</summary>
            [DataMember] public int Anchor;
            /// <summary>Signed lengths of the moves since the anchor (the last one still in progress).</summary>
            [DataMember] public List<double> Segments = new List<double>();
            /// <summary>Number of direction changes, so a sync can tell whether the axis reversed since the last one.</summary>
            [DataMember] public int Reversals;

            public int Dir => Segments.Count > 0 ? Math.Sign(Segments[Segments.Count - 1]) : Anchor;

            public void Reset()
            {
                Play = 0; Initialised = false; Anchor = 0; Segments = new List<double>(); Reversals = 0;
            }

            public void CopyFrom(GearState g)
            {
                if (g == null) return;
                Play = g.Play; LastRaw = g.LastRaw; Initialised = g.Initialised; Anchor = g.Anchor;
                Segments = new List<double>(g.Segments ?? new List<double>()); Reversals = g.Reversals;
            }

            /// <summary>The history to store with a sync: anchor direction then the moves since. Null without an anchor.</summary>
            public double[] History()
            {
                if (Anchor == 0) return null;
                var h = new double[Segments.Count + 1];
                h[0] = Anchor;
                Segments.CopyTo(h, 1);
                return h;
            }

            public void Update(double raw, double backlash)
            {
                double half = backlash / 2;
                if (Segments == null) Segments = new List<double>();
                if (!Initialised)
                {
                    LastRaw = raw;
                    Initialised = true;
                    Play = Math.Max(-half, Math.Min(half, Play));
                    return;
                }
                double delta = raw - LastRaw;
                LastRaw = raw;
                Play = Math.Max(-half, Math.Min(half, Play + delta));
                if (delta == 0) return;

                int last = Segments.Count - 1;
                if (last >= 0 && Math.Sign(Segments[last]) == Math.Sign(delta))
                {
                    Segments[last] += delta;
                }
                else
                {
                    if (last >= 0) Reversals++;
                    Segments.Add(delta);
                }

                // A long move fully loads the gears one way: it becomes the new anchor.
                last = Segments.Count - 1;
                if (Math.Abs(Segments[last]) >= PointingModel.LearningMove)
                {
                    Anchor = Math.Sign(Segments[last]);
                    Segments.Clear();
                }
                else if (Segments.Count > MaxHistory)
                {
                    // Too much wandering to replay; forget the anchor until the next long move.
                    Anchor = 0;
                    Segments.RemoveRange(0, Segments.Count - MaxHistory);
                }
            }
        }

        private sealed class SyncSnapshot
        {
            public AltAz Sky;
            public double RawAz, RawAlt, PlayAz, PlayAlt;
            public double[] HistoryAz, HistoryAlt;
            public DateTime Utc;
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
            /// <summary>Final-move direction per axis (+1, -1, or 0 for straight in).</summary>
            public int ApproachAz, ApproachAlt;
            /// <summary>The axis stopped short on the far side and still needs its final approach leg.</summary>
            public bool PendingAz, PendingAlt;
            /// <summary>Detour points (tube angles) to visit before the target, to keep clear of the Sun.</summary>
            public Queue<AltAz> Waypoints;
            /// <summary>Distance from the Sun when the current leg started, degrees.</summary>
            public double StartSunSeparationDeg;
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
