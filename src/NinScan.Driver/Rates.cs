using System;
using System.Collections;
using System.Runtime.InteropServices;
using ASCOM.DeviceInterface;

namespace NinScan.Driver
{
    [ComVisible(true)]
    [Guid("cf68003b-8e29-4674-81ef-8a9c0132423e")]
    [ClassInterface(ClassInterfaceType.None)]
    public sealed class Rate : IRate
    {
        public Rate(double minimum, double maximum)
        {
            Minimum = minimum;
            Maximum = maximum;
        }

        public double Maximum { get; set; }
        public double Minimum { get; set; }
        public void Dispose() { }
    }

    [ComVisible(true)]
    [Guid("1ddd1186-3106-427a-a6fb-4fca26a77309")]
    [ClassInterface(ClassInterfaceType.None)]
    public sealed class AxisRates : IAxisRates, IEnumerable
    {
        private readonly IRate[] _rates;

        public AxisRates(double maxDegPerSec)
        {
            _rates = maxDegPerSec > 0 ? new IRate[] { new Rate(0, maxDegPerSec) } : new IRate[0];
        }

        public int Count => _rates.Length;

        /// <summary>ASCOM collections are 1-based.</summary>
        public IRate this[int index]
        {
            get
            {
                if (index < 1 || index > _rates.Length) throw new ASCOM.InvalidValueException("AxisRates index", index.ToString(), "1 to " + _rates.Length);
                return _rates[index - 1];
            }
        }

        public IEnumerator GetEnumerator() => _rates.GetEnumerator();
        public void Dispose() { }
    }

    [ComVisible(true)]
    [Guid("e33c7dd1-648f-42ee-b9f8-ed2ae6d51ade")]
    [ClassInterface(ClassInterfaceType.None)]
    public sealed class TrackingRates : ITrackingRates, IEnumerable
    {
        private readonly DriveRates[] _rates = { DriveRates.driveSidereal, DriveRates.driveLunar, DriveRates.driveSolar };

        public int Count => _rates.Length;

        public DriveRates this[int index]
        {
            get
            {
                if (index < 1 || index > _rates.Length) throw new ASCOM.InvalidValueException("TrackingRates index", index.ToString(), "1 to " + _rates.Length);
                return _rates[index - 1];
            }
        }

        public IEnumerator GetEnumerator() => _rates.GetEnumerator();
        public void Dispose() { }
    }
}
