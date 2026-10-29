using System;
using System.Collections;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using ASCOM.DeviceInterface;
using NinScan.Core.Astronomy;
using NinScan.Core.Mount;
using NinScan.Core.Protocol;

namespace NinScan.Driver
{
    /// <summary>
    /// ASCOM telescope driver for a Sky-Watcher alt-az mount (Flextube GoTo Dobsonian) on the
    /// Sky-Watcher USB adapter. COM starts this server on demand, so no app has to be open.
    /// </summary>
    [ComVisible(true)]
    [Guid("076b22b4-8302-41d7-bb06-eb60510833c7")]
    [ProgId(ProgIdValue)]
    [ClassInterface(ClassInterfaceType.None)]
    public sealed class Telescope : ReferenceCountedObject, ITelescopeV4
    {
        public const string ProgIdValue = "ASCOM.NinScan.Telescope";
        public const string ChooserName = "NinScan (Sky-Watcher Alt-Az)";

        private readonly Guid _client = Guid.NewGuid();
        private double? _targetRa;
        private double? _targetDec;
        private short _slewSettleTime;

        private static AltAzMount Engine => SharedMount.Engine;

        // ------------------------------------------------------------------ common

        public string Description => "NinScan driver for Sky-Watcher alt-az GoTo mounts on the USB adapter";

        public string DriverInfo => "NinScan " + DriverVersion + ". Runs the alt-az conversion, tracking and pointing model itself, so the SynScan App is not needed. https://github.com/awat21/NinScan";

        public string DriverVersion
        {
            get
            {
                var v = typeof(Telescope).Assembly.GetName().Version;
                return v.Major + "." + v.Minor;
            }
        }

        public short InterfaceVersion => 4;

        public string Name => "NinScan";

        public bool Connected
        {
            get => SharedMount.IsConnected(_client);
            set
            {
                Guard("Connected=" + value, () =>
                {
                    if (value) SharedMount.Connect(_client);
                    else SharedMount.Disconnect(_client);
                });
            }
        }

        public void Connect() => Connected = true;

        public void Disconnect() => Connected = false;

        public bool Connecting => false;

        public void SetupDialog()
        {
            using (var f = new SetupForm()) f.ShowDialog();
        }

        public ArrayList SupportedActions => new ArrayList { "NinScan:ModelInfo", "NinScan:ClearModel", "NinScan:Status", "NinScan:Mechanics" };

        public string Action(string actionName, string actionParameters)
        {
            switch ((actionName ?? "").ToLowerInvariant())
            {
                case "ninscan:modelinfo":
                    return Engine.Model.PointCount + " point(s); " + Engine.Model.Terms;
                case "ninscan:clearmodel":
                    Engine.ClearModel();
                    return "cleared";
                case "ninscan:mechanics":
                {
                    var m = Engine.Mechanics;
                    return "backlash az " + (AstroMath.ToDeg(m.Effective(0)) * 60).ToString("F1") + "′ (" + m.TotalWeight(0) + " syncs), alt "
                        + (AstroMath.ToDeg(m.Effective(1)) * 60).ToString("F1") + "′ (" + m.TotalWeight(1) + " syncs); slips az " + m.SlipEvents[0]
                        + ", alt " + m.SlipEvents[1] + (Engine.LastSyncReport != null ? "; last sync: " + Engine.LastSyncReport : "");
                }
                case "ninscan:status":
                    return Engine.Connected
                        ? "mount " + Engine.MountPosition + "; sky " + Engine.SkyPosition + "; tracking " + Engine.Tracking + "; slewing " + Engine.Slewing
                            + (Engine.LastError != null ? "; last error: " + Engine.LastError : "")
                        : "not connected";
                default:
                    throw new ASCOM.ActionNotImplementedException(actionName);
            }
        }

        public void CommandBlind(string command, bool raw) => throw new ASCOM.MethodNotImplementedException("CommandBlind");
        public bool CommandBool(string command, bool raw) => throw new ASCOM.MethodNotImplementedException("CommandBool");
        public string CommandString(string command, bool raw) => throw new ASCOM.MethodNotImplementedException("CommandString");

        public void Dispose()
        {
            try { SharedMount.Disconnect(_client); } catch { }
        }

        public IStateValueCollection DeviceState
        {
            get
            {
                RequireConnected("DeviceState");
                var c = new StateValueCollection();
                TryAdd(c, nameof(Altitude), () => Altitude);
                TryAdd(c, nameof(AtHome), () => AtHome);
                TryAdd(c, nameof(AtPark), () => AtPark);
                TryAdd(c, nameof(Azimuth), () => Azimuth);
                TryAdd(c, nameof(Declination), () => Declination);
                TryAdd(c, nameof(IsPulseGuiding), () => IsPulseGuiding);
                TryAdd(c, nameof(RightAscension), () => RightAscension);
                TryAdd(c, nameof(SiderealTime), () => SiderealTime);
                TryAdd(c, nameof(Slewing), () => Slewing);
                TryAdd(c, nameof(Tracking), () => Tracking);
                TryAdd(c, nameof(UTCDate), () => UTCDate);
                c.AddUtcDateTime();
                return c;
            }
        }

        private static void TryAdd(StateValueCollection c, string name, Func<object> get)
        {
            try { c.Add(name, get()); } catch { }
        }

        // ------------------------------------------------------------------ capabilities

        public AlignmentModes AlignmentMode => AlignmentModes.algAltAz;
        public EquatorialCoordinateType EquatorialSystem => EquatorialCoordinateType.equTopocentric;
        public bool CanFindHome => false;
        public bool CanPark => true;
        public bool CanPulseGuide => false;
        public bool CanSetDeclinationRate => false;
        public bool CanSetGuideRates => false;
        public bool CanSetPark => true;
        public bool CanSetPierSide => false;
        public bool CanSetRightAscensionRate => false;
        public bool CanSetTracking => true;
        public bool CanSlew => true;
        public bool CanSlewAltAz => true;
        public bool CanSlewAltAzAsync => true;
        public bool CanSlewAsync => true;
        public bool CanSync => true;
        public bool CanSyncAltAz => true;
        public bool CanUnpark => true;

        public bool CanMoveAxis(TelescopeAxes axis) => axis == TelescopeAxes.axisPrimary || axis == TelescopeAxes.axisSecondary;

        public IAxisRates AxisRates(TelescopeAxes axis)
        {
            return CanMoveAxis(axis) ? new AxisRates(Engine.Settings.MaxMoveRateDegPerSec) : new AxisRates(0);
        }

        public ITrackingRates TrackingRates => new TrackingRates();

        public double ApertureDiameter => Engine.Settings.ApertureMm / 1000.0;

        public double ApertureArea
        {
            get
            {
                double r = Engine.Settings.ApertureMm / 2000.0;
                return Math.PI * r * r;
            }
        }

        public double FocalLength => Engine.Settings.FocalLengthMm / 1000.0;

        // ------------------------------------------------------------------ position

        public double Altitude
        {
            get { RequireConnected("Altitude"); return AstroMath.ToDeg(Engine.SkyPosition.Alt); }
        }

        public double Azimuth
        {
            get { RequireConnected("Azimuth"); return AstroMath.ToDeg(AstroMath.Wrap2Pi(Engine.SkyPosition.Az)); }
        }

        public double RightAscension
        {
            get { RequireConnected("RightAscension"); return Engine.Equatorial.RaHours; }
        }

        public double Declination
        {
            get { RequireConnected("Declination"); return Engine.Equatorial.DecDeg; }
        }

        public double SiderealTime
        {
            get { RequireConnected("SiderealTime"); return Engine.LocalSiderealTimeHours; }
        }

        public DateTime UTCDate
        {
            get => Engine.UtcNow;
            set => throw new ASCOM.PropertyNotImplementedException("UTCDate", true);
        }

        public bool AtHome => false;

        public bool AtPark
        {
            get { RequireConnected("AtPark"); return Engine.AtPark; }
        }

        public bool Slewing
        {
            get { RequireConnected("Slewing"); return Engine.Slewing; }
        }

        public PierSide SideOfPier
        {
            get => throw new ASCOM.PropertyNotImplementedException("SideOfPier", false);
            set => throw new ASCOM.PropertyNotImplementedException("SideOfPier", true);
        }

        public PierSide DestinationSideOfPier(double rightAscension, double declination) => throw new ASCOM.MethodNotImplementedException("DestinationSideOfPier");

        // ------------------------------------------------------------------ site

        public double SiteLatitude
        {
            get => Engine.Settings.LatitudeDeg;
            set
            {
                if (value < -90 || value > 90) throw new ASCOM.InvalidValueException("SiteLatitude", value.ToString(), "-90 to 90");
                UpdateSettings(s => s.LatitudeDeg = value);
            }
        }

        public double SiteLongitude
        {
            get => Engine.Settings.LongitudeDeg;
            set
            {
                if (value < -180 || value > 180) throw new ASCOM.InvalidValueException("SiteLongitude", value.ToString(), "-180 to 180");
                UpdateSettings(s => s.LongitudeDeg = value);
            }
        }

        public double SiteElevation
        {
            get => Engine.Settings.ElevationM;
            set
            {
                if (value < -300 || value > 10000) throw new ASCOM.InvalidValueException("SiteElevation", value.ToString(), "-300 to 10000");
                UpdateSettings(s => s.ElevationM = value);
            }
        }

        public bool DoesRefraction
        {
            get => Engine.Settings.Refraction;
            set => UpdateSettings(s => s.Refraction = value);
        }

        private static void UpdateSettings(Action<MountSettings> change)
        {
            var s = Engine.Settings;
            change(s);
            SharedMount.SaveSettings(s);
        }

        // ------------------------------------------------------------------ tracking

        public bool Tracking
        {
            get { RequireConnected("Tracking"); return Engine.Tracking; }
            set
            {
                RequireConnected("Tracking");
                Guard("Tracking=" + value, () => Engine.Tracking = value);
            }
        }

        public DriveRates TrackingRate
        {
            get
            {
                switch (Engine.TrackingRate)
                {
                    case TrackingRateKind.Lunar: return DriveRates.driveLunar;
                    case TrackingRateKind.Solar: return DriveRates.driveSolar;
                    default: return DriveRates.driveSidereal;
                }
            }
            set
            {
                switch (value)
                {
                    case DriveRates.driveSidereal: Engine.TrackingRate = TrackingRateKind.Sidereal; break;
                    case DriveRates.driveLunar: Engine.TrackingRate = TrackingRateKind.Lunar; break;
                    case DriveRates.driveSolar: Engine.TrackingRate = TrackingRateKind.Solar; break;
                    default: throw new ASCOM.InvalidValueException("TrackingRate", value.ToString(), "sidereal, lunar or solar");
                }
            }
        }

        public double RightAscensionRate
        {
            get => 0;
            set => throw new ASCOM.PropertyNotImplementedException("RightAscensionRate", true);
        }

        public double DeclinationRate
        {
            get => 0;
            set => throw new ASCOM.PropertyNotImplementedException("DeclinationRate", true);
        }

        // Pulse guiding is planned for v2.
        public bool IsPulseGuiding => false;
        public void PulseGuide(GuideDirections direction, int duration) => throw new ASCOM.MethodNotImplementedException("PulseGuide");

        public double GuideRateRightAscension
        {
            get => throw new ASCOM.PropertyNotImplementedException("GuideRateRightAscension", false);
            set => throw new ASCOM.PropertyNotImplementedException("GuideRateRightAscension", true);
        }

        public double GuideRateDeclination
        {
            get => throw new ASCOM.PropertyNotImplementedException("GuideRateDeclination", false);
            set => throw new ASCOM.PropertyNotImplementedException("GuideRateDeclination", true);
        }

        // ------------------------------------------------------------------ targets and slews

        public double TargetRightAscension
        {
            get => _targetRa ?? throw new ASCOM.InvalidOperationException("TargetRightAscension has not been set.");
            set
            {
                CheckRa(value);
                _targetRa = value;
            }
        }

        public double TargetDeclination
        {
            get => _targetDec ?? throw new ASCOM.InvalidOperationException("TargetDeclination has not been set.");
            set
            {
                CheckDec(value);
                _targetDec = value;
            }
        }

        public short SlewSettleTime
        {
            get => _slewSettleTime;
            set
            {
                if (value < 0) throw new ASCOM.InvalidValueException("SlewSettleTime", value.ToString(), "0 or more seconds");
                _slewSettleTime = value;
            }
        }

        public void SlewToCoordinatesAsync(double rightAscension, double declination)
        {
            RequireConnected("SlewToCoordinatesAsync");
            CheckRa(rightAscension);
            CheckDec(declination);
            RequireUnparked();
            if (!Engine.Tracking) throw new ASCOM.InvalidOperationException("Tracking must be on to slew to RA/Dec.");
            _targetRa = rightAscension;
            _targetDec = declination;
            Guard("SlewToCoordinates", () => Engine.SlewToEquatorial(new RaDec(rightAscension, declination)));
        }

        public void SlewToCoordinates(double rightAscension, double declination)
        {
            SlewToCoordinatesAsync(rightAscension, declination);
            WaitForSlew();
        }

        public void SlewToTargetAsync() => SlewToCoordinatesAsync(TargetRightAscension, TargetDeclination);

        public void SlewToTarget() => SlewToCoordinates(TargetRightAscension, TargetDeclination);

        public void SlewToAltAzAsync(double azimuth, double altitude)
        {
            RequireConnected("SlewToAltAzAsync");
            CheckAz(azimuth);
            CheckAlt(altitude);
            RequireUnparked();
            if (Engine.Tracking) throw new ASCOM.InvalidOperationException("Tracking must be off to slew to Alt/Az.");
            Guard("SlewToAltAz", () => Engine.SlewToHorizontal(new AltAz(AstroMath.ToRad(azimuth), AstroMath.ToRad(altitude))));
        }

        public void SlewToAltAz(double azimuth, double altitude)
        {
            SlewToAltAzAsync(azimuth, altitude);
            WaitForSlew();
        }

        public void SyncToCoordinates(double rightAscension, double declination)
        {
            RequireConnected("SyncToCoordinates");
            CheckRa(rightAscension);
            CheckDec(declination);
            RequireUnparked();
            if (!Engine.Tracking) throw new ASCOM.InvalidOperationException("Tracking must be on to sync to RA/Dec.");
            _targetRa = rightAscension;
            _targetDec = declination;
            Guard("SyncToCoordinates", () => Engine.SyncToEquatorial(new RaDec(rightAscension, declination)));
        }

        public void SyncToTarget() => SyncToCoordinates(TargetRightAscension, TargetDeclination);

        public void SyncToAltAz(double azimuth, double altitude)
        {
            RequireConnected("SyncToAltAz");
            CheckAz(azimuth);
            CheckAlt(altitude);
            RequireUnparked();
            if (Engine.Tracking) throw new ASCOM.InvalidOperationException("Tracking must be off to sync to Alt/Az.");
            Guard("SyncToAltAz", () => Engine.SyncToHorizontal(new AltAz(AstroMath.ToRad(azimuth), AstroMath.ToRad(altitude))));
        }

        public void AbortSlew()
        {
            RequireConnected("AbortSlew");
            RequireUnparked();
            Guard("AbortSlew", () => Engine.AbortSlew());
        }

        public void MoveAxis(TelescopeAxes axis, double rate)
        {
            RequireConnected("MoveAxis");
            if (!CanMoveAxis(axis)) throw new ASCOM.InvalidValueException("MoveAxis", axis.ToString(), "primary (azimuth) or secondary (altitude)");
            double max = Engine.Settings.MaxMoveRateDegPerSec;
            if (Math.Abs(rate) > max) throw new ASCOM.InvalidValueException("MoveAxis rate", rate.ToString(), "-" + max + " to " + max);
            RequireUnparked();
            Guard("MoveAxis", () => Engine.MoveAxis(axis == TelescopeAxes.axisPrimary ? Axis.Azimuth : Axis.Altitude, rate));
        }

        // ------------------------------------------------------------------ park and home

        public void Park()
        {
            RequireConnected("Park");
            // Interface V4: Park returns at once; Slewing is true until AtPark becomes true.
            Guard("Park", () => Engine.Park());
        }

        public void Unpark()
        {
            RequireConnected("Unpark");
            Guard("Unpark", () => Engine.Unpark());
        }

        public void SetPark()
        {
            RequireConnected("SetPark");
            Guard("SetPark", () => SharedMount.SaveSettings(Engine.SetParkHere()));
        }

        public void FindHome() => throw new ASCOM.MethodNotImplementedException("FindHome");

        // ------------------------------------------------------------------ helpers

        private void WaitForSlew()
        {
            while (Engine.Slewing)
            {
                Thread.Sleep(100);
                Application.DoEvents();
            }
            if (_slewSettleTime > 0) Thread.Sleep(_slewSettleTime * 1000);
        }

        private void RequireConnected(string member)
        {
            if (!SharedMount.IsConnected(_client)) throw new ASCOM.NotConnectedException(member + " needs the telescope to be connected.");
        }

        private static void RequireUnparked()
        {
            if (Engine.AtPark) throw new ASCOM.ParkedException("The mount is parked. Unpark it first.");
        }

        private static void CheckRa(double ra)
        {
            if (ra < 0 || ra >= 24 || double.IsNaN(ra)) throw new ASCOM.InvalidValueException("RightAscension", ra.ToString(), "0 to 24 hours");
        }

        private static void CheckDec(double dec)
        {
            if (dec < -90 || dec > 90 || double.IsNaN(dec)) throw new ASCOM.InvalidValueException("Declination", dec.ToString(), "-90 to 90 degrees");
        }

        private static void CheckAz(double az)
        {
            if (az < 0 || az >= 360 || double.IsNaN(az)) throw new ASCOM.InvalidValueException("Azimuth", az.ToString(), "0 to 360 degrees");
        }

        private static void CheckAlt(double alt)
        {
            if (alt < -90 || alt > 90 || double.IsNaN(alt)) throw new ASCOM.InvalidValueException("Altitude", alt.ToString(), "-90 to 90 degrees");
        }

        /// <summary>Runs an engine call, logging failures and translating them to ASCOM exceptions.</summary>
        private static void Guard(string what, Action action)
        {
            try
            {
                action();
            }
            catch (ASCOM.DriverException)
            {
                throw;
            }
            catch (ArgumentOutOfRangeException ex)
            {
                SharedMount.Log.Warn(what + " refused: " + ex.Message);
                throw new ASCOM.InvalidValueException(ex.Message);
            }
            catch (ArgumentException ex)
            {
                SharedMount.Log.Warn(what + " refused: " + ex.Message);
                throw new ASCOM.InvalidValueException(ex.Message);
            }
            catch (System.InvalidOperationException ex)
            {
                SharedMount.Log.Warn(what + " refused: " + ex.Message);
                if (Engine.AtPark) throw new ASCOM.ParkedException(ex.Message);
                throw new ASCOM.InvalidOperationException(ex.Message);
            }
            catch (Exception ex)
            {
                SharedMount.Log.Error(what + " failed", ex);
                throw new ASCOM.DriverException(what + " failed: " + ex.Message, ex);
            }
        }
    }
}
