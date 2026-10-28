using System;
using NinScan.Core.Astronomy;
using NinScan.Core.Pointing;
using Xunit;

namespace NinScan.Core.Tests
{
    public class PointingModelTests
    {
        private static double Arcsec(double rad) => AstroMath.ToDeg(rad) * 3600;

        private static AltAz D(double azDeg, double altDeg) => new AltAz(AstroMath.ToRad(azDeg), AstroMath.ToRad(altDeg));

        private static double[] TruthTerms() => new[]
        {
            AstroMath.ToRad(37.5),     // IA: the base started 37.5° from north
            AstroMath.ToRad(-1.2),     // IE: tube not quite level at power-on
            AstroMath.ToRad(1.8),      // tilt north: an unlevel lawn
            AstroMath.ToRad(-0.9),     // tilt east
            AstroMath.ToRad(0.15),     // cone
            AstroMath.ToRad(0.05),     // NPAE
            AstroMath.ToRad(0.08),     // flexure
        };

        [Fact]
        public void EmptyModel_IsIdentity()
        {
            var m = new PointingModel();
            var p = D(123, 45);
            var q = m.SkyToMount(p);
            Assert.Equal(p.Az, q.Az, 12);
            Assert.Equal(p.Alt, q.Alt, 12);
        }

        [Fact]
        public void OneSync_FixesTheZeroPoints()
        {
            var truth = TruthTerms();
            var m = new PointingModel();
            var sky = D(200, 40);
            m.AddPoint(sky, PointingModel.Forward(truth, sky), DateTime.UtcNow);
            var back = m.MountToSky(PointingModel.Forward(truth, sky));
            Assert.True(Arcsec(PointingModel.Separation(back, sky)) < 1);
        }

        [Fact]
        public void ManySyncs_RecoverATiltedBaseAccurately()
        {
            var truth = TruthTerms();
            var m = new PointingModel { ResetThreshold = 0 };
            var rnd = new Random(7);
            for (int i = 0; i < 12; i++)
            {
                var sky = D(rnd.NextDouble() * 360, 15 + rnd.NextDouble() * 65);
                m.AddPoint(sky, PointingModel.Forward(truth, sky), DateTime.UtcNow);
            }
            Assert.Equal(12, m.PointCount);

            // Check gotos all over the sky, not just at the sync points.
            double worst = 0;
            for (int i = 0; i < 300; i++)
            {
                var sky = D(rnd.NextDouble() * 360, 10 + rnd.NextDouble() * 75);
                var want = PointingModel.Forward(truth, sky);
                var got = m.SkyToMount(sky);
                double err = PointingModel.Separation(new AltAz(AstroMath.Wrap2Pi(want.Az), want.Alt), got);
                worst = Math.Max(worst, err);
            }
            Assert.True(Arcsec(worst) < 5, "worst error " + Arcsec(worst) + "″");
        }

        [Fact]
        public void ThreeSpreadSyncs_GetGotosWithinAFewArcminutes()
        {
            var truth = TruthTerms();
            var m = new PointingModel();
            foreach (var sky in new[] { D(30, 30), D(150, 55), D(270, 40) })
                m.AddPoint(sky, PointingModel.Forward(truth, sky), DateTime.UtcNow);

            var rnd = new Random(3);
            double worst = 0;
            for (int i = 0; i < 300; i++)
            {
                var sky = D(rnd.NextDouble() * 360, 15 + rnd.NextDouble() * 65);
                var want = PointingModel.Forward(truth, sky);
                worst = Math.Max(worst, PointingModel.Separation(new AltAz(AstroMath.Wrap2Pi(want.Az), want.Alt), m.SkyToMount(sky)));
            }
            // Only tilt and zero points are solved with three points; the residual is cone, NPAE and flexure.
            Assert.True(AstroMath.ToDeg(worst) * 60 < 25, "worst error " + AstroMath.ToDeg(worst) * 60 + "′");
        }

        [Fact]
        public void InverseUndoesForward()
        {
            var truth = TruthTerms();
            var m = new PointingModel { ResetThreshold = 0 };
            var rnd = new Random(11);
            for (int i = 0; i < 10; i++)
            {
                var sky = D(rnd.NextDouble() * 360, 15 + rnd.NextDouble() * 65);
                m.AddPoint(sky, PointingModel.Forward(truth, sky), DateTime.UtcNow);
            }
            for (int i = 0; i < 200; i++)
            {
                var sky = D(rnd.NextDouble() * 360, 5 + rnd.NextDouble() * 80);
                var back = m.MountToSky(m.SkyToMount(sky));
                Assert.True(Arcsec(PointingModel.Separation(back, sky)) < 0.01);
            }
        }

        [Fact]
        public void SyncFarFromTheModel_RestartsIt()
        {
            var truth = TruthTerms();
            var m = new PointingModel();
            foreach (var sky in new[] { D(30, 30), D(150, 55), D(270, 40) })
                m.AddPoint(sky, PointingModel.Forward(truth, sky), DateTime.UtcNow);
            Assert.Equal(3, m.PointCount);

            // The base got kicked 10° round.
            var s2 = D(90, 45);
            var mount = PointingModel.Forward(truth, s2);
            mount.Az += AstroMath.ToRad(10);
            string note = m.AddPoint(s2, mount, DateTime.UtcNow);
            Assert.Equal(1, m.PointCount);
            Assert.Contains("restarting", note);
        }

        [Fact]
        public void NearbySync_ReplacesTheOldPoint()
        {
            var m = new PointingModel();
            m.AddPoint(D(100, 40), D(101, 40.5), DateTime.UtcNow);
            m.AddPoint(D(101, 41), D(102, 41.5), DateTime.UtcNow);
            Assert.Equal(1, m.PointCount);
        }
    }
}
