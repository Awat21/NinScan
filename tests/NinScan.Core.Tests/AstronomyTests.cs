using System;
using NinScan.Core.Astronomy;
using Xunit;

namespace NinScan.Core.Tests
{
    public class AstronomyTests
    {
        [Fact]
        public void JulianDate_OfJ2000()
        {
            Assert.Equal(2451545.0, AstroMath.JulianDate(new DateTime(2000, 1, 1, 12, 0, 0, DateTimeKind.Utc)), 9);
        }

        [Fact]
        public void Gmst_MatchesMeeusExample()
        {
            // Meeus, Astronomical Algorithms, example 12.b: 1987-04-10 19:21:00 UT → 8h 34m 57.0896s.
            double gmst = AstroMath.GreenwichMeanSiderealHours(new DateTime(1987, 4, 10, 19, 21, 0, DateTimeKind.Utc));
            Assert.Equal(8 + 34 / 60.0 + 57.0896 / 3600.0, gmst, 5);
        }

        [Fact]
        public void HorizontalConversion_RoundTrips()
        {
            var site = new Site { LatitudeDeg = 51.5, LongitudeDeg = -0.12 };
            var t = new DateTime(2026, 10, 28, 21, 30, 0, DateTimeKind.Utc);
            var rnd = new Random(1);
            for (int i = 0; i < 200; i++)
            {
                var rd = new RaDec(rnd.NextDouble() * 24, rnd.NextDouble() * 170 - 85);
                var h = AstroMath.RaDecToAltAz(rd, t, site);
                var back = AstroMath.AltAzToRaDec(h, t, site);
                Assert.Equal(rd.DecDeg, back.DecDeg, 8);
                Assert.Equal(0, AstroMath.ToDeg(AstroMath.WrapPi(AstroMath.ToRad((back.RaHours - rd.RaHours) * 15))), 7);
            }
        }

        [Fact]
        public void Polaris_StandsAtAboutTheLatitude()
        {
            var site = new Site { LatitudeDeg = 51.5, LongitudeDeg = 0 };
            var h = AstroMath.RaDecToAltAz(new RaDec(3.0, 89.37), new DateTime(2026, 10, 28, 22, 0, 0, DateTimeKind.Utc), site);
            Assert.InRange(AstroMath.ToDeg(h.Alt), 50.8, 52.2);
            Assert.True(AstroMath.ToDeg(AstroMath.WrapPi(h.Az)) is > -1.5 and < 1.5);
        }

        [Fact]
        public void Refraction_IsAboutHalfADegreeAtTheHorizonAndRoundTrips()
        {
            var site = new Site();
            double r0 = AstroMath.ToDeg(AstroMath.RefractionFromTrue(0, site)) * 60;
            Assert.InRange(r0, 28, 36);
            double r45 = AstroMath.ToDeg(AstroMath.RefractionFromTrue(AstroMath.ToRad(45), site)) * 60;
            Assert.InRange(r45, 0.9, 1.1);
            for (double deg = -1; deg <= 90; deg += 0.5)
            {
                double t = AstroMath.ToRad(deg);
                Assert.Equal(t, AstroMath.ApparentToTrue(AstroMath.TrueToApparent(t, site), site), 9);
            }
        }
    }
}
