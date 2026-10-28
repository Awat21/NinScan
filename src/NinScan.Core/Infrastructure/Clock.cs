using System;
using System.Threading;

namespace NinScan.Core.Infrastructure
{
    /// <summary>
    /// Time source for the mount engine. Everything that depends on "now" goes through this,
    /// so tests and the simulator can run on a manual clock.
    /// </summary>
    public interface IClock
    {
        DateTime UtcNow { get; }
        void Sleep(TimeSpan duration);
    }

    public sealed class SystemClock : IClock
    {
        public static readonly SystemClock Instance = new SystemClock();
        public DateTime UtcNow => DateTime.UtcNow;
        public void Sleep(TimeSpan duration)
        {
            if (duration > TimeSpan.Zero) Thread.Sleep(duration);
        }
    }

    /// <summary>A clock that only moves when told to. Sleep advances time instantly.</summary>
    public sealed class ManualClock : IClock
    {
        private readonly object _lock = new object();
        private DateTime _now;

        public ManualClock(DateTime startUtc)
        {
            _now = DateTime.SpecifyKind(startUtc, DateTimeKind.Utc);
        }

        public DateTime UtcNow { get { lock (_lock) return _now; } }

        public void Advance(TimeSpan duration)
        {
            lock (_lock) _now = _now + duration;
        }

        public void Sleep(TimeSpan duration)
        {
            if (duration > TimeSpan.Zero) Advance(duration);
        }
    }
}
