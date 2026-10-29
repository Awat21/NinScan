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
    public class SunSafetyTests : IDisposable
    {
        // Midday in London at the end of October: the Sun is due south, about 25° up.
        private readonly ManualClock _clock = new ManualClock(new DateTime(2026, 10, 29, 11, 45, 0, DateTimeKind.Utc));
        private readonly string _stateDir = Path.Combine(Path.GetTempPath(), "ninscan-sun-" + Guid.NewGuid().ToString("N"));
        private SimulatedMotorBoard _sim;

        public void Dispose()
        {
            try { Directory.Delete(_stateDir, true); } catch { }
        }

        private AltAzMount NewMount()
        {
            _sim = new SimulatedMotorBoard(_clock);
            var sim = _sim;
            var s = new MountSettings { LatitudeDeg = 51.5, LongitudeDeg = -0.1 };
            return new AltAzMount(s, _clock, NullLog.Instance, _stateDir, runBackgroundLoop: false, transportFactory: _ => new KeepAlive(sim));
        }

        private sealed class KeepAlive : ITransport
        {
            private readonly ITransport _t;
            public KeepAlive(ITransport t) { _t = t; }
            public string Transact(string c) => _t.Transact(c);
            public string Description => "test";
            public void Dispose() { }
        }

        private static AltAz D(double az, double alt) => new AltAz(AstroMath.ToRad(az), AstroMath.ToRad(alt));

        private double SunSepDeg(AltAzMount m) => AstroMath.ToDeg(PointingModel.Separation(m.SkyPosition, m.SunPosition));

        private double RunWatchingSun(AltAzMount m, double maxSeconds = 600)
        {
            double closest = 180;
            for (double t = 0; t < maxSeconds && m.Slewing; t += 0.25)
            {
                _clock.Advance(TimeSpan.FromSeconds(0.25));
                m.Tick();
                closest = Math.Min(closest, SunSepDeg(m));
            }
            Assert.False(m.Slewing);
            return closest;
        }

        [Fact]
        public void SunPosition_MatchesMeeus()
        {
            // Meeus example 25.a: 1992-10-13 0h TD → RA 13h13m31.4s, Dec −7°47′01″ (TD ≈ UT + 59 s then).
            var sun = AstroMath.SunPosition(new DateTime(1992, 10, 12, 23, 59, 1, DateTimeKind.Utc));
            Assert.InRange((sun.RaHours - (13 + 13 / 60.0 + 31.4 / 3600)) * 15, -0.02, 0.02);
            Assert.InRange(sun.DecDeg + (7 + 47 / 60.0 + 1 / 3600.0), -0.02, 0.02);
        }

        [Fact]
        public void KnowsWhenItIsDaylight()
        {
            using var m = NewMount();
            m.Connect();
            Assert.True(m.IsSunUp);
            Assert.True(m.ShouldOfferSolarViewing);
            Assert.InRange(AstroMath.ToDeg(m.SunPosition.Alt), 20, 30);
            m.SolarViewing = true;
            Assert.False(m.ShouldOfferSolarViewing);

            _clock.Advance(TimeSpan.FromHours(10));
            Assert.False(m.IsSunUp);
        }

        [Fact]
        public void SolarViewingIsOffAtEveryConnect()
        {
            using var m = NewMount();
            m.Connect();
            m.SolarViewing = true;
            m.Disconnect();
            m.Connect();
            Assert.False(m.SolarViewing);
        }

        [Fact]
        public void GotoNearTheSun_IsRefusedUntilSolarViewingIsOn()
        {
            using var m = NewMount();
            m.Connect();
            var sun = m.SunPosition;
            var nearSun = new AltAz(sun.Az + AstroMath.ToRad(5), sun.Alt);
            var ex = Assert.Throws<InvalidOperationException>(() => m.SlewToHorizontal(nearSun));
            Assert.Contains("Sun", ex.Message);
            Assert.False(m.Slewing);

            m.SolarViewing = true;
            m.TrackingRate = TrackingRateKind.Solar;
            m.SlewToEquatorial(AstroMath.SunPosition(_clock.UtcNow));
            RunWatchingSun(m);
            for (int i = 0; i < 600; i++) { _clock.Advance(TimeSpan.FromSeconds(0.5)); m.Tick(); }
            Assert.True(m.Tracking);
            Assert.True(SunSepDeg(m) < 0.03, "after 5 minutes on the Sun, " + SunSepDeg(m) + "° off");
        }

        [Fact]
        public void GotoAcrossTheSun_TakesADetour()
        {
            using var m = NewMount();
            m.Connect();
            var sun = m.SunPosition;
            double sunAz = AstroMath.ToDeg(sun.Az), sunAlt = AstroMath.ToDeg(sun.Alt);
            // From east of the Sun to west of it at the same height: the direct path goes straight through.
            m.SlewToHorizontal(D(sunAz - 40, sunAlt));
            RunWatchingSun(m);
            m.SlewToHorizontal(D(sunAz + 40, sunAlt));
            double closest = RunWatchingSun(m);
            Assert.True(closest >= 14.5, "came within " + closest + "° of the Sun");
            Assert.Equal(sunAz + 40, AstroMath.ToDeg(AstroMath.Wrap2Pi(m.SkyPosition.Az)), 1);
        }

        [Fact]
        public void ManualMove_StopsAtTheEdgeOfTheZone()
        {
            using var m = NewMount();
            m.Connect();
            var sun = m.SunPosition;
            m.SlewToHorizontal(new AltAz(sun.Az - AstroMath.ToRad(40), sun.Alt));
            RunWatchingSun(m);
            m.MoveAxis(Axis.Azimuth, 2.0);
            for (int i = 0; i < 120; i++)
            {
                _clock.Advance(TimeSpan.FromSeconds(0.25));
                m.Tick();
            }
            Assert.False(m.Slewing);
            Assert.InRange(SunSepDeg(m), 13.5, 16);
            Assert.Throws<InvalidOperationException>(() => m.MoveAxis(Axis.Azimuth, 1.0));
            m.MoveAxis(Axis.Azimuth, -1.0); // moving away is always allowed
            Assert.True(m.Slewing);
        }

        [Fact]
        public void SwitchingSolarViewingOffOnTheSun_StillAllowsLeaving()
        {
            using var m = NewMount();
            m.Connect();
            m.SolarViewing = true;
            m.SlewToHorizontal(m.SunPosition);
            RunWatchingSun(m);
            m.SolarViewing = false;
            m.Park();
            RunWatchingSun(m);
            Assert.True(m.AtPark);
        }
    }
}
