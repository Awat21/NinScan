using System;
using System.IO;
using NinScan.Core.Astronomy;
using NinScan.Core.Infrastructure;
using NinScan.Core.Mount;
using NinScan.Core.Pointing;
using NinScan.Core.Protocol;
using NinScan.Core.Simulation;
using Xunit;

namespace NinScan.Core.Tests
{
    public class MountEngineTests : IDisposable
    {
        private readonly ManualClock _clock = new ManualClock(new DateTime(2026, 10, 28, 21, 0, 0, DateTimeKind.Utc));
        private readonly string _stateDir = Path.Combine(Path.GetTempPath(), "ninscan-tests-" + Guid.NewGuid().ToString("N"));
        private SimulatedMotorBoard _sim;

        public void Dispose()
        {
            try { Directory.Delete(_stateDir, true); } catch { }
        }

        private AltAzMount NewMount(MountSettings settings = null)
        {
            settings = settings ?? new MountSettings { LatitudeDeg = 51.5, LongitudeDeg = -0.1 };
            if (_sim == null) _sim = new SimulatedMotorBoard(_clock);
            var sim = _sim;
            return new AltAzMount(settings, _clock, NullLog.Instance, _stateDir, runBackgroundLoop: false, transportFactory: _ => new KeepAlive(sim));
        }

        private sealed class KeepAlive : ITransport
        {
            private readonly ITransport _t;
            public KeepAlive(ITransport t) { _t = t; }
            public string Transact(string c) => _t.Transact(c);
            public string Description => "test";
            public void Dispose() { }
        }

        private void Run(AltAzMount m, double seconds, double step = 0.5)
        {
            for (double t = 0; t < seconds; t += step)
            {
                _clock.Advance(TimeSpan.FromSeconds(step));
                m.Tick();
            }
        }

        private void RunUntilSettled(AltAzMount m, double maxSeconds = 600)
        {
            double t = 0;
            while (m.Slewing && t < maxSeconds)
            {
                _clock.Advance(TimeSpan.FromSeconds(0.5));
                m.Tick();
                t += 0.5;
            }
            Assert.False(m.Slewing, "Goto did not finish within " + maxSeconds + " s");
        }

        private RaDec StarAt(AltAz geometric, Site site) => AstroMath.AltAzToRaDec(geometric, _clock.UtcNow, site);

        private static double SepArcsec(RaDec a, RaDec b)
        {
            double d1 = AstroMath.ToRad(a.DecDeg), d2 = AstroMath.ToRad(b.DecDeg);
            double dra = AstroMath.ToRad((a.RaHours - b.RaHours) * 15);
            double c = Math.Sin(d1) * Math.Sin(d2) + Math.Cos(d1) * Math.Cos(d2) * Math.Cos(dra);
            return AstroMath.ToDeg(Math.Acos(Math.Min(1, c))) * 3600;
        }

        private static AltAz D(double az, double alt) => new AltAz(AstroMath.ToRad(az), AstroMath.ToRad(alt));

        [Fact]
        public void FreshPowerOn_StartsAtTheStartPosition()
        {
            using var m = NewMount();
            m.Connect();
            Assert.True(_sim.Initialised);
            Assert.Equal(0, m.MountPosition.Az, 9);
            Assert.Equal(0, m.MountPosition.Alt, 9);
            Assert.False(m.Tracking);
            Assert.False(m.Slewing);
        }

        [Fact]
        public void GotoThenTrack_StaysOnTargetForTenMinutes()
        {
            using var m = NewMount();
            m.Connect();
            var star = StarAt(D(135, 40), m.Site);
            m.SlewToEquatorial(star);
            Assert.True(m.Slewing);
            RunUntilSettled(m);
            Assert.True(m.Tracking);
            Assert.True(SepArcsec(m.Equatorial, star) < 30, "arrived " + SepArcsec(m.Equatorial, star) + "″ off");

            // "Arrived" means within the 20″ goto tolerance; tracking pulls in the rest within a few seconds.
            Run(m, 10);
            double worst = 0;
            for (int i = 0; i < 1200; i++)
            {
                Run(m, 0.5);
                worst = Math.Max(worst, SepArcsec(m.Equatorial, star));
            }
            Assert.True(worst < 2, "tracking error reached " + worst + "″");
            Assert.True(m.Tracking);
        }

        [Fact]
        public void Tracking_HandlesAStarCrossingTheMeridianHigh()
        {
            // Az speed peaks and changes direction for stars passing close to the zenith.
            using var m = NewMount();
            m.Connect();
            var site = m.Site;
            // A star 8° south of the zenith, an hour before it transits.
            var star = new RaDec(AstroMath.Wrap24(m.LocalSiderealTimeHours + 0.5), site.LatitudeDeg - 8);
            m.SlewToEquatorial(star);
            RunUntilSettled(m);
            double worst = 0;
            for (int i = 0; i < 7200; i++)
            {
                Run(m, 0.5);
                if (!m.Tracking) break;
                worst = Math.Max(worst, SepArcsec(m.Equatorial, star));
            }
            Assert.True(m.Tracking);
            Assert.True(worst < 15, "tracking error reached " + worst + "″");
        }

        [Fact]
        public void SyncsCorrectAMisalignedStart()
        {
            using var m = NewMount();
            m.Connect();
            var site = m.Site;
            // The user actually started with the tube 10° west of north and 1° nose-up.
            double azOffset = AstroMath.ToRad(-10), altOffset = AstroMath.ToRad(1);
            Func<AltAz> trueSky = () => new AltAz(AstroMath.Wrap2Pi(m.MountPosition.Az + azOffset), m.MountPosition.Alt + altOffset);
            Func<RaDec> trueRaDec = () => m.RaDecFromApparent(trueSky(), _clock.UtcNow);

            var a = StarAt(D(60, 35), site);
            m.SlewToEquatorial(a);
            RunUntilSettled(m);
            Assert.True(SepArcsec(trueRaDec(), a) > 3600, "should be well off before syncing");

            // Plate solve: the camera says where it really is.
            m.SyncToEquatorial(trueRaDec());
            m.SlewToEquatorial(a);
            RunUntilSettled(m);
            Assert.True(SepArcsec(trueRaDec(), a) < 30, "after one sync, " + SepArcsec(trueRaDec(), a) + "″ off");

            // A goto across the sky lands on target too, since this start error is a pure offset.
            var b = StarAt(D(250, 55), site);
            m.SlewToEquatorial(b);
            RunUntilSettled(m);
            Assert.True(SepArcsec(trueRaDec(), b) < 60, "second target " + SepArcsec(trueRaDec(), b) + "″ off");
        }

        [Fact]
        public void TrackingStopsAtTheAltitudeLimit()
        {
            var s = new MountSettings { LatitudeDeg = 51.5, LongitudeDeg = -0.1, MinAltitudeDeg = 10 };
            using var m = NewMount(s);
            m.Connect();
            // A star setting in the west at 11°.
            var star = StarAt(D(280, 11), m.Site);
            m.SlewToEquatorial(star);
            RunUntilSettled(m);
            Assert.True(m.Tracking);
            Run(m, 1800);
            Assert.False(m.Tracking);
            Assert.Contains("altitude limit", m.LastError);
            Assert.False(_sim.IsRunning(Axis.Azimuth));
            Assert.False(_sim.IsRunning(Axis.Altitude));
        }

        [Fact]
        public void GotoBelowTheHorizonIsRefused()
        {
            using var m = NewMount();
            m.Connect();
            var star = StarAt(D(10, -20), m.Site);
            Assert.Throws<ArgumentOutOfRangeException>(() => m.SlewToEquatorial(star));
            Assert.False(m.Slewing);
        }

        [Fact]
        public void AzimuthStaysWithinItsTravel()
        {
            var s = new MountSettings { LatitudeDeg = 51.5, LongitudeDeg = -0.1, AzimuthTravelDeg = 200 };
            using var m = NewMount(s);
            m.Connect();
            m.SlewToHorizontal(D(170, 30));
            RunUntilSettled(m);
            Assert.Equal(170, AstroMath.ToDeg(m.MountPosition.Az), 1);
            // 200° is the short way on from 170° and still inside the ±200° travel...
            m.SlewToHorizontal(D(200, 30));
            RunUntilSettled(m);
            Assert.Equal(200, AstroMath.ToDeg(m.MountPosition.Az), 1);
            // ...but 220° would pass the stop, so it goes the long way round to -140°.
            m.SlewToHorizontal(D(220, 30));
            RunUntilSettled(m);
            Assert.Equal(-140, AstroMath.ToDeg(m.MountPosition.Az), 1);
        }

        [Fact]
        public void MoveAxis_MovesUntilStopped()
        {
            using var m = NewMount();
            m.Connect();
            m.MoveAxis(Axis.Altitude, 1.0);
            Assert.True(m.Slewing);
            Run(m, 10);
            m.MoveAxis(Axis.Altitude, 0);
            Assert.False(m.Slewing);
            Assert.InRange(AstroMath.ToDeg(m.MountPosition.Alt), 9.0, 11.0);
            Assert.False(_sim.IsRunning(Axis.Altitude));
        }

        [Fact]
        public void MoveAxis_StopsAtTheLimit()
        {
            using var m = NewMount();
            m.Connect();
            Assert.Throws<InvalidOperationException>(() => m.MoveAxis(Axis.Altitude, -1.0));
            m.MoveAxis(Axis.Altitude, 2.0);
            Run(m, 120);
            Assert.False(_sim.IsRunning(Axis.Altitude));
            Assert.InRange(AstroMath.ToDeg(m.MountPosition.Alt), 87.5, 90);
        }

        [Fact]
        public void ParkAndUnpark()
        {
            using var m = NewMount();
            m.Connect();
            m.SlewToHorizontal(D(90, 45));
            RunUntilSettled(m);
            m.Park();
            Assert.True(m.Slewing);
            RunUntilSettled(m);
            Assert.True(m.AtPark);
            Assert.Equal(0, AstroMath.ToDeg(m.MountPosition.Az), 2);
            Assert.Equal(0, AstroMath.ToDeg(m.MountPosition.Alt), 2);
            Assert.Throws<InvalidOperationException>(() => m.SlewToHorizontal(D(90, 45)));
            Assert.Throws<InvalidOperationException>(() => m.Tracking = true);
            m.Unpark();
            Assert.False(m.AtPark);
        }

        [Fact]
        public void Reconnect_KeepsPositionModelAndParkState()
        {
            AltAz pos;
            int points;
            using (var m = NewMount())
            {
                m.Connect();
                m.SlewToHorizontal(D(120, 50));
                RunUntilSettled(m);
                m.SyncToHorizontal(D(121, 50.5));
                pos = m.MountPosition;
                points = m.Model.PointCount;
                m.Disconnect();
            }
            Assert.Equal(1, points);
            using (var m = NewMount())
            {
                m.Connect();
                Assert.Equal(pos.Az, m.MountPosition.Az, 6);
                Assert.Equal(1, m.Model.PointCount);
                Assert.Equal(121, AstroMath.ToDeg(m.SkyPosition.Az), 3);
            }
        }

        [Fact]
        public void PushingTheTubeWhileTracking_TracksTheNewSpot()
        {
            using var m = NewMount();
            m.Connect();
            var star = StarAt(D(200, 45), m.Site);
            m.SlewToEquatorial(star);
            RunUntilSettled(m);
            Run(m, 5);
            // Push 3° in azimuth by hand.
            _sim.Push(Axis.Azimuth, 3 * _sim.CountsPerRevolution / 360.0);
            m.Tick();
            var newSpot = m.Equatorial;
            Assert.True(SepArcsec(newSpot, star) > 3600);
            Run(m, 60);
            Assert.True(m.Tracking);
            Assert.True(SepArcsec(m.Equatorial, newSpot) < 10, "drifted " + SepArcsec(m.Equatorial, newSpot) + "″ from the pushed-to spot");
        }

        [Fact]
        public void AbortSlew_StopsAGoto()
        {
            using var m = NewMount();
            m.Connect();
            m.SlewToHorizontal(D(180, 60));
            Run(m, 3);
            Assert.True(m.Slewing);
            m.AbortSlew();
            Assert.False(m.Slewing);
            Assert.False(_sim.IsRunning(Axis.Azimuth));
            Assert.False(_sim.IsRunning(Axis.Altitude));
        }

        [Fact]
        public void Disconnect_StopsTheMotors()
        {
            var m = NewMount();
            m.Connect();
            m.Tracking = true;
            m.SlewToEquatorial(StarAt(D(100, 40), m.Site));
            RunUntilSettled(m);
            Assert.True(_sim.IsRunning(Axis.Azimuth) || _sim.IsRunning(Axis.Altitude));
            m.Disconnect();
            Assert.False(_sim.IsRunning(Axis.Azimuth));
            Assert.False(_sim.IsRunning(Axis.Altitude));
        }

        [Fact]
        public void LunarRate_FollowsTheMoonsDrift()
        {
            using var m = NewMount();
            m.Connect();
            var star = StarAt(D(150, 35), m.Site);
            m.SlewToEquatorial(star);
            RunUntilSettled(m);
            m.TrackingRate = TrackingRateKind.Lunar;
            var startRa = m.TrackTarget.Value.RaHours;
            Run(m, 3600);
            // The Moon gains about 0.356″/s of RA on the stars, so 1282″ in an hour.
            double gained = (m.TrackTarget.Value.RaHours - startRa) * 54000;
            Assert.InRange(gained, 1275, 1290);
            Assert.True(SepArcsec(m.Equatorial, m.TrackTarget.Value) < 5);
        }

        [Fact]
        public void ReverseAxisSetting_IsHonoured()
        {
            var s = new MountSettings { LatitudeDeg = 51.5, LongitudeDeg = -0.1, ReverseAzimuth = true };
            using var m = NewMount(s);
            m.Connect();
            m.SlewToHorizontal(D(45, 30));
            RunUntilSettled(m);
            Assert.True(_sim.PositionOf(Axis.Azimuth) < 0);
            Assert.Equal(45, AstroMath.ToDeg(m.MountPosition.Az), 1);
        }
    }
}
