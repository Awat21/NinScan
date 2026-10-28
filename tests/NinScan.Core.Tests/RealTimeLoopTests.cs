using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using NinScan.Core.Astronomy;
using NinScan.Core.Infrastructure;
using NinScan.Core.Mount;
using Xunit;

namespace NinScan.Core.Tests
{
    public class RealTimeLoopTests
    {
        [Fact]
        public void BackgroundLoop_FinishesAGotoAndTracksInRealTime()
        {
            var dir = Path.Combine(Path.GetTempPath(), "ninscan-rt-" + Guid.NewGuid().ToString("N"));
            var settings = new MountSettings { ComPort = MountSettings.SimulatorPort, LatitudeDeg = 51.5, TrackingIntervalMs = 200 };
            using (var m = new AltAzMount(settings, SystemClock.Instance, NullLog.Instance, dir))
            {
                m.Connect();
                var target = AstroMath.AltAzToRaDec(new AltAz(AstroMath.ToRad(4), AstroMath.ToRad(3)), DateTime.UtcNow, m.Site);
                m.SlewToEquatorial(target);
                var sw = Stopwatch.StartNew();
                while (m.Slewing && sw.Elapsed < TimeSpan.FromSeconds(30)) Thread.Sleep(50);
                Assert.False(m.Slewing);
                Assert.True(m.Tracking);
                Thread.Sleep(2000);
                Assert.True(m.Tracking);
                Assert.Null(m.LastError);
            }
            try { Directory.Delete(dir, true); } catch { }
        }
    }
}
