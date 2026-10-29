using System;
using System.IO;
using NinScan.Core.Astronomy;
using NinScan.Core.Infrastructure;
using NinScan.Core.Mount;
using NinScan.Core.Protocol;
using NinScan.Core.Simulation;
using Xunit;

namespace NinScan.Core.Tests
{
    /// <summary>
    /// Backlash learning and compensation, against a simulated mount whose tube lags the motors by real
    /// gear play. "Plate solves" are taken from where the simulated tube really points.
    /// </summary>
    public class BacklashTests : IDisposable
    {
        private readonly ManualClock _clock = new ManualClock(new DateTime(2026, 10, 28, 21, 0, 0, DateTimeKind.Utc));
        private readonly string _stateDir = Path.Combine(Path.GetTempPath(), "ninscan-bl-" + Guid.NewGuid().ToString("N"));
        private SimulatedMotorBoard _sim;
        internal static readonly ListLog Log = new ListLog();

        internal sealed class ListLog : ILog
        {
            public readonly System.Collections.Generic.List<string> Lines = new System.Collections.Generic.List<string>();
            public void Info(string m) { lock (Lines) Lines.Add("I " + m); }
            public void Warn(string m) { lock (Lines) Lines.Add("W " + m); }
            public void Error(string m, Exception ex = null) { lock (Lines) Lines.Add("E " + m + ex); }
            public void Debug(string m) { lock (Lines) Lines.Add("D " + m); }
        }

        // The user started with the tube a little off north and not quite level.
        private static readonly double OffAz = AstroMath.ToRad(-4);
        private static readonly double OffAlt = AstroMath.ToRad(0.6);

        public void Dispose()
        {
            try { Directory.Delete(_stateDir, true); } catch { }
        }

        private AltAzMount NewMount(MountSettings s, double backlashAzDeg, double backlashAltDeg, bool newBoard = true)
        {
            if (newBoard || _sim == null)
            {
                _sim = new SimulatedMotorBoard(_clock);
                _sim.SetBacklash(Axis.Azimuth, backlashAzDeg * _sim.CountsPerRevolution / 360.0);
                _sim.SetBacklash(Axis.Altitude, backlashAltDeg * _sim.CountsPerRevolution / 360.0);
            }
            var sim = _sim;
            return new AltAzMount(s, _clock, Log, _stateDir, runBackgroundLoop: false, transportFactory: _ => new KeepAlive(sim));
        }

        private static MountSettings Settings(ApproachMode approach, BacklashMode mode = BacklashMode.Learn)
            => new MountSettings { LatitudeDeg = 51.5, LongitudeDeg = -0.1, Approach = approach, Backlash = mode, ManualMoveThresholdDeg = 0 };

        private sealed class KeepAlive : ITransport
        {
            private readonly ITransport _t;
            public KeepAlive(ITransport t) { _t = t; }
            public string Transact(string c) => _t.Transact(c);
            public string Description => "test";
            public void Dispose() { }
        }

        private double StepsPerRad => _sim.CountsPerRevolution / (2 * Math.PI);

        /// <summary>Where the tube really points (what a plate solve would report).</summary>
        private AltAz TrueSky() => new AltAz(AstroMath.Wrap2Pi(_sim.TubePositionOf(Axis.Azimuth) / StepsPerRad + OffAz),
            _sim.TubePositionOf(Axis.Altitude) / StepsPerRad + OffAlt);

        private RaDec PlateSolve(AltAzMount m) => m.RaDecFromApparent(TrueSky(), _clock.UtcNow);

        private void Run(AltAzMount m, double seconds)
        {
            for (double t = 0; t < seconds; t += 0.5)
            {
                _clock.Advance(TimeSpan.FromSeconds(0.5));
                m.Tick();
            }
        }

        private void Settle(AltAzMount m)
        {
            for (int i = 0; i < 1200 && m.Slewing; i++)
            {
                _clock.Advance(TimeSpan.FromSeconds(0.5));
                m.Tick();
            }
            Assert.False(m.Slewing);
        }

        private RaDec Star(AltAzMount m, double azDeg, double altDeg)
            => m.RaDecFromApparent(new AltAz(AstroMath.ToRad(azDeg), AstroMath.ToRad(altDeg)), _clock.UtcNow);

        private static double SepArcmin(RaDec a, RaDec b)
        {
            double d1 = AstroMath.ToRad(a.DecDeg), d2 = AstroMath.ToRad(b.DecDeg);
            double c = Math.Sin(d1) * Math.Sin(d2) + Math.Cos(d1) * Math.Cos(d2) * Math.Cos(AstroMath.ToRad((a.RaHours - b.RaHours) * 15));
            return AstroMath.ToDeg(Math.Acos(Math.Min(1, c))) * 60;
        }

        // Targets that make both axes change direction often.
        private static readonly double[][] Tour =
        {
            new[] { 100.0, 30 }, new[] { 60.0, 55 }, new[] { 140.0, 25 }, new[] { 90.0, 60 }, new[] { 200.0, 35 },
            new[] { 150.0, 65 }, new[] { 250.0, 30 }, new[] { 210.0, 50 }, new[] { 300.0, 40 }, new[] { 260.0, 20 },
            new[] { 330.0, 55 }, new[] { 280.0, 35 }, new[] { 30.0, 45 }, new[] { 350.0, 25 },
        };

        private void GotoAndSync(AltAzMount m, double az, double alt)
        {
            m.SlewToEquatorial(Star(m, az, alt));
            Settle(m);
            Run(m, 2);
            m.SyncToEquatorial(PlateSolve(m));
        }

        [Fact]
        public void LearnsBacklashFromPlateSolveSyncs()
        {
            using var m = NewMount(Settings(ApproachMode.Off), 0.40, 0.25);
            m.Connect();
            foreach (var t in Tour) GotoAndSync(m, t[0], t[1]);

            Assert.True(m.Model.BacklashFitted(0) && m.Model.BacklashFitted(1), "backlash should be fitted on both axes");
            Assert.InRange(AstroMath.ToDeg(m.Model.BacklashAz), 0.34, 0.46);
            Assert.InRange(AstroMath.ToDeg(m.Model.BacklashAlt), 0.20, 0.30);
            Assert.InRange(AstroMath.ToDeg(m.Mechanics.Effective(0)), 0.34, 0.46);

            // Gotos now land on target whichever way the axes have to reverse.
            double worst = 0;
            foreach (var t in new[] { new[] { 120.0, 40 }, new[] { 80.0, 50 }, new[] { 230.0, 30 }, new[] { 180.0, 45 } })
            {
                var target = Star(m, t[0], t[1]);
                m.SlewToEquatorial(target);
                Settle(m);
                Run(m, 2);
                worst = Math.Max(worst, SepArcmin(PlateSolve(m), m.TrackTarget.Value));
            }
            Assert.True(worst < 2.0, "worst goto error after learning " + worst + "′");
        }

        [Fact]
        public void WithoutCompensation_GotosMissByAboutHalfTheBacklash()
        {
            // The baseline the learning has to beat.
            using var m = NewMount(Settings(ApproachMode.Off, BacklashMode.Off), 0.40, 0.25);
            m.Connect();
            foreach (var t in Tour) GotoAndSync(m, t[0], t[1]);
            double worst = 0;
            foreach (var t in new[] { new[] { 120.0, 40 }, new[] { 80.0, 50 }, new[] { 230.0, 30 }, new[] { 180.0, 45 } })
            {
                m.SlewToEquatorial(Star(m, t[0], t[1]));
                Settle(m);
                Run(m, 2);
                worst = Math.Max(worst, SepArcmin(PlateSolve(m), m.TrackTarget.Value));
            }
            Assert.True(worst > 5, "expected a visible miss without compensation, got " + worst + "′");
        }

        [Fact]
        public void UpAndRightApproach_FinishesWithTheGearsLoadedUpAndRight()
        {
            var s = Settings(ApproachMode.UpAndRight);
            using var m = NewMount(s, 0.40, 0.25);
            m.Connect();
            m.SlewToHorizontal(new AltAz(AstroMath.ToRad(90), AstroMath.ToRad(60)));
            Settle(m);
            // Now go down and to the left: the last move must still be up and to the right.
            m.SlewToHorizontal(new AltAz(AstroMath.ToRad(60), AstroMath.ToRad(40)));
            Settle(m);
            double playAz = _sim.PositionOf(Axis.Azimuth) - _sim.TubePositionOf(Axis.Azimuth);
            double playAlt = _sim.PositionOf(Axis.Altitude) - _sim.TubePositionOf(Axis.Altitude);
            double spd = _sim.CountsPerRevolution / 360.0;
            Assert.Equal(0.20, playAz / spd, 3);
            Assert.Equal(0.125, playAlt / spd, 3);
        }

        [Fact]
        public void TrackingThroughAReversal_TakesUpThePlayInsteadOfStalling()
        {
            // Known backlash, and an approach that ends opposite to the tracking direction in altitude.
            var s = Settings(ApproachMode.UpAndRight, BacklashMode.Manual);
            s.ManualBacklashAzArcmin = 0;
            s.ManualBacklashAltArcmin = 18;
            using var m = NewMount(s, 0.0, 0.30);
            m.Connect();
            // Calibrate the zero points with one sync, then pick a star setting in the west (altitude falling).
            GotoAndSync(m, 250, 35);
            var star = Star(m, 260, 30);
            m.SlewToEquatorial(star);
            Settle(m);
            double worst = 0;
            for (int i = 0; i < 300; i++)
            {
                Run(m, 1);
                worst = Math.Max(worst, SepArcmin(PlateSolve(m), star));
            }
            Assert.True(worst < 0.5, "tracking error reached " + worst + "′");
        }

        [Fact]
        public void AClutchSlip_IsReportedAtTheNextSync()
        {
            using var m = NewMount(Settings(ApproachMode.TrackingDirection), 0, 0);
            m.Connect();
            GotoAndSync(m, 120, 40);
            GotoAndSync(m, 125, 42);
            Assert.Equal(0, m.Mechanics.SlipEvents[0]);

            // The base slips 0.3° while slewing to the next target.
            _sim.Slip(Axis.Azimuth, -0.3 * _sim.CountsPerRevolution / 360.0);
            GotoAndSync(m, 130, 44);
            Assert.Equal(1, m.Mechanics.SlipEvents[0]);
            Assert.Equal(0, m.Mechanics.SlipEvents[1]);
            Assert.Contains("clutch", m.LastSyncReport);

            // A normal move afterwards raises nothing new.
            GotoAndSync(m, 135, 46);
            Assert.Equal(1, m.Mechanics.SlipEvents[0]);
        }

        [Fact]
        public void LearnedBacklash_SurvivesAPowerCycle()
        {
            using (var m = NewMount(Settings(ApproachMode.Off), 0.40, 0.25))
            {
                m.Connect();
                foreach (var t in Tour) GotoAndSync(m, t[0], t[1]);
                m.Disconnect();
            }
            // Switched off and on again: new board, model cleared, backlash remembered.
            using (var m = NewMount(Settings(ApproachMode.Off), 0.40, 0.25))
            {
                m.Connect();
                Assert.Equal(0, m.Model.PointCount);
                Assert.InRange(AstroMath.ToDeg(m.Model.BacklashAz), 0.34, 0.46);
                Assert.InRange(AstroMath.ToDeg(m.Model.BacklashAlt), 0.20, 0.30);
            }
        }
    }
}
