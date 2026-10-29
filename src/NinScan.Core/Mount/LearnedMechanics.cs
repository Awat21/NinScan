using System;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;

namespace NinScan.Core.Mount
{
    /// <summary>
    /// What the driver has learned about this mount's gears, kept across power cycles.
    /// The pointing model is rebuilt every session because the base moves; backlash and clutch
    /// behaviour belong to the mount, so they carry over.
    ///
    /// Each axis keeps a prior (all earlier sessions combined) and the current session's estimate.
    /// They are merged weighted by the number of syncs that taught them, so a single odd session
    /// cannot throw away what many good ones learned.
    /// </summary>
    [DataContract]
    public sealed class LearnedMechanics
    {
        /// <summary>Evidence from earlier sessions stops counting beyond this many syncs, so the mount can still adapt as it wears.</summary>
        public const double MaxPriorWeight = 40;

        [DataMember] public double[] PriorBacklash = new double[2];
        [DataMember] public double[] PriorWeight = new double[2];
        [DataMember] public double[] SessionBacklash = new double[2];
        [DataMember] public double[] SessionWeight = new double[2];
        /// <summary>Times a sync showed the tube moving differently from the motors, not explained by backlash.</summary>
        [DataMember] public int[] SlipEvents = new int[2];
        [DataMember] public DateTime UpdatedUtc;

        /// <summary>Best current backlash estimate for an axis (0 az, 1 alt), radians.</summary>
        public double Effective(int axis)
        {
            double w = PriorWeight[axis] + SessionWeight[axis];
            if (w <= 0) return 0;
            return (PriorBacklash[axis] * PriorWeight[axis] + SessionBacklash[axis] * SessionWeight[axis]) / w;
        }

        public double TotalWeight(int axis) => PriorWeight[axis] + SessionWeight[axis];

        public void SetSession(int axis, double backlash, double weight, DateTime utc)
        {
            SessionBacklash[axis] = backlash;
            SessionWeight[axis] = weight;
            UpdatedUtc = utc;
        }

        /// <summary>Called at a fresh power-on: folds the last session into the prior.</summary>
        public void RollSession()
        {
            for (int i = 0; i < 2; i++)
            {
                if (SessionWeight[i] <= 0) continue;
                PriorBacklash[i] = Effective(i);
                PriorWeight[i] = Math.Min(MaxPriorWeight, PriorWeight[i] + SessionWeight[i]);
                SessionBacklash[i] = 0;
                SessionWeight[i] = 0;
            }
        }

        public void Reset()
        {
            PriorBacklash = new double[2];
            PriorWeight = new double[2];
            SessionBacklash = new double[2];
            SessionWeight = new double[2];
            SlipEvents = new int[2];
        }

        public LearnedMechanics Clone()
        {
            return new LearnedMechanics
            {
                PriorBacklash = (double[])PriorBacklash.Clone(),
                PriorWeight = (double[])PriorWeight.Clone(),
                SessionBacklash = (double[])SessionBacklash.Clone(),
                SessionWeight = (double[])SessionWeight.Clone(),
                SlipEvents = (int[])SlipEvents.Clone(),
                UpdatedUtc = UpdatedUtc,
            };
        }

        public static LearnedMechanics Load(string path)
        {
            try
            {
                if (path == null || !File.Exists(path)) return new LearnedMechanics();
                using (var fs = File.OpenRead(path))
                {
                    var m = (LearnedMechanics)new DataContractJsonSerializer(typeof(LearnedMechanics)).ReadObject(fs);
                    return m != null && m.PriorBacklash?.Length == 2 ? m : new LearnedMechanics();
                }
            }
            catch
            {
                return new LearnedMechanics();
            }
        }

        public void Save(string path)
        {
            if (path == null) return;
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            using (var fs = File.Create(path))
            {
                new DataContractJsonSerializer(typeof(LearnedMechanics)).WriteObject(fs, this);
            }
        }
    }
}
