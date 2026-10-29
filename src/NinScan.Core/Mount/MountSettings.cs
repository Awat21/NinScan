using System;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;

namespace NinScan.Core.Mount
{
    public enum BacklashMode
    {
        /// <summary>Learn backlash from syncs and plate solves, and compensate for it.</summary>
        Learn = 0,
        /// <summary>Use the values typed into the settings.</summary>
        Manual = 1,
        /// <summary>No backlash compensation.</summary>
        Off = 2,
    }

    /// <summary>Which way a goto makes its final move, so the gears always end up loaded the same way.</summary>
    public enum ApproachMode
    {
        /// <summary>Finish moving the way tracking will continue, so tracking starts with the gears already loaded.</summary>
        TrackingDirection = 0,
        /// <summary>Always finish moving up and to the right (increasing altitude and azimuth), like the SynScan App.</summary>
        UpAndRight = 1,
        /// <summary>Always finish moving down and to the left.</summary>
        DownAndLeft = 2,
        /// <summary>Go straight to the target.</summary>
        Off = 3,
    }

    public enum AuxEncoderMode
    {
        /// <summary>Do not touch the board's Freedom Find setting.</summary>
        LeaveAsIs = 0,
        On = 1,
        Off = 2,
    }

    /// <summary>User settings. Stored as JSON so the core can be tested without the Windows registry.</summary>
    [DataContract]
    public sealed class MountSettings
    {
        public const string SimulatorPort = "SIMULATOR";

        /// <summary>COM port of the Sky-Watcher USB adapter, or <see cref="SimulatorPort"/>.</summary>
        [DataMember] public string ComPort = SimulatorPort;
        /// <summary>0 tries 9600 then 115200.</summary>
        [DataMember] public int BaudRate = 0;

        [DataMember] public double LatitudeDeg = 51.5;
        /// <summary>East positive.</summary>
        [DataMember] public double LongitudeDeg = 0.0;
        [DataMember] public double ElevationM = 50;
        [DataMember] public double PressureHPa = 1010;
        [DataMember] public double TemperatureC = 10;
        [DataMember] public bool Refraction = true;

        /// <summary>Flip if a move towards increasing azimuth turns the base the wrong way.</summary>
        [DataMember] public bool ReverseAzimuth = false;
        /// <summary>Flip if a move towards increasing altitude lowers the tube.</summary>
        [DataMember] public bool ReverseAltitude = false;

        /// <summary>Where the tube points when the mount is powered on: by Sky-Watcher convention level and facing north.</summary>
        [DataMember] public double StartAzimuthDeg = 0;
        [DataMember] public double StartAltitudeDeg = 0;

        [DataMember] public double MinAltitudeDeg = 0;
        /// <summary>Kept short of the zenith, where an alt-az mount cannot keep up in azimuth.</summary>
        [DataMember] public double MaxAltitudeDeg = 88;
        /// <summary>How far the base may turn either way from the start position, to stop cables winding up.</summary>
        [DataMember] public double AzimuthTravelDeg = 270;

        [DataMember] public double ParkAzimuthDeg = 0;
        [DataMember] public double ParkAltitudeDeg = 0;

        /// <summary>How often tracking rates are recalculated.</summary>
        [DataMember] public int TrackingIntervalMs = 500;
        /// <summary>Fastest manual (MoveAxis) speed offered to clients, degrees per second.</summary>
        [DataMember] public double MaxMoveRateDegPerSec = 3.0;
        /// <summary>A goto keeps refining until it is this close, arcseconds.</summary>
        [DataMember] public double GotoToleranceArcsec = 20;
        /// <summary>
        /// While tracking, an unexplained jump larger than this (degrees) is taken as the tube being
        /// pushed by hand, and tracking carries on from the new position. 0 disables.
        /// </summary>
        [DataMember] public double ManualMoveThresholdDeg = 0.5;
        /// <summary>A sync further than this from the model (degrees) restarts the model. 0 disables.</summary>
        [DataMember] public double ModelResetThresholdDeg = 2.0;

        /// <summary>
        /// Freedom Find: with the auxiliary encoders on, the board keeps its position when the tube is pushed by hand.
        /// </summary>
        [DataMember] public AuxEncoderMode AuxEncoders = AuxEncoderMode.LeaveAsIs;

        [DataMember] public BacklashMode Backlash = BacklashMode.Learn;
        /// <summary>Used when <see cref="Backlash"/> is Manual, arcminutes.</summary>
        [DataMember] public double ManualBacklashAzArcmin = 0;
        [DataMember] public double ManualBacklashAltArcmin = 0;
        [DataMember] public ApproachMode Approach = ApproachMode.TrackingDirection;
        /// <summary>Smallest overshoot before the final approach move, degrees. It grows to twice the backlash when that is larger.</summary>
        [DataMember] public double MinApproachOvershootDeg = 1.0;

        /// <summary>
        /// While solar viewing is off, the telescope is kept at least this far from the Sun (degrees).
        /// Solar viewing itself is not a stored setting: it is switched on per connection, after confirming a filter is fitted.
        /// </summary>
        [DataMember] public double SunAvoidanceDeg = 15;

        /// <summary>Reported to clients only. Defaults are the Flextube 250P.</summary>
        [DataMember] public double ApertureMm = 254;
        [DataMember] public double FocalLengthMm = 1200;

        [DataMember] public bool DebugLogging = false;

        public MountSettings Clone() => (MountSettings)MemberwiseClone();

        public static MountSettings Load(string path)
        {
            try
            {
                if (!File.Exists(path)) return new MountSettings();
                using (var fs = File.OpenRead(path))
                {
                    return (MountSettings)new DataContractJsonSerializer(typeof(MountSettings)).ReadObject(fs) ?? new MountSettings();
                }
            }
            catch
            {
                return new MountSettings();
            }
        }

        public void Save(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            using (var fs = File.Create(path))
            {
                new DataContractJsonSerializer(typeof(MountSettings)).WriteObject(fs, this);
            }
        }

        public void Validate()
        {
            if (LatitudeDeg < -90 || LatitudeDeg > 90) throw new ArgumentException("Latitude must be between -90 and 90 degrees.");
            if (LongitudeDeg < -180 || LongitudeDeg > 180) throw new ArgumentException("Longitude must be between -180 and 180 degrees.");
            if (MinAltitudeDeg >= MaxAltitudeDeg) throw new ArgumentException("Minimum altitude must be below maximum altitude.");
            if (MaxAltitudeDeg > 90) throw new ArgumentException("Maximum altitude cannot exceed 90 degrees.");
            if (AzimuthTravelDeg < 180) throw new ArgumentException("Azimuth travel must be at least 180 degrees either way, or some of the sky is unreachable.");
            if (TrackingIntervalMs < 100 || TrackingIntervalMs > 5000) throw new ArgumentException("Tracking interval must be 100–5000 ms.");
            if (MaxMoveRateDegPerSec <= 0) throw new ArgumentException("Maximum move rate must be positive.");
            if (ManualBacklashAzArcmin < 0 || ManualBacklashAltArcmin < 0 || ManualBacklashAzArcmin > 54 || ManualBacklashAltArcmin > 54)
                throw new ArgumentException("Manual backlash must be between 0 and 54 arcminutes.");
            if (SunAvoidanceDeg < 5 || SunAvoidanceDeg > 45) throw new ArgumentException("Sun avoidance must be 5–45 degrees.");
            if (MinApproachOvershootDeg < 0 || MinApproachOvershootDeg > 5) throw new ArgumentException("Approach overshoot must be 0–5 degrees.");
        }
    }
}
