using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using NinScan.Core.Astronomy;

namespace NinScan.Core.Pointing
{
    /// <summary>One sync: where the sky really was and where the mount axes said they were.</summary>
    [DataContract]
    public sealed class SyncPoint
    {
        [DataMember] public double SkyAz;
        [DataMember] public double SkyAlt;
        [DataMember] public double MountAz;
        [DataMember] public double MountAlt;
        [DataMember] public DateTime TimeUtc;
    }

    /// <summary>Fitted model terms, radians.</summary>
    [DataContract]
    public sealed class ModelTerms
    {
        /// <summary>Azimuth index: where mount azimuth zero really points.</summary>
        [DataMember] public double IA;
        /// <summary>Altitude index.</summary>
        [DataMember] public double IE;
        /// <summary>Tilt of the azimuth axis towards north.</summary>
        [DataMember] public double TiltN;
        /// <summary>Tilt of the azimuth axis towards east.</summary>
        [DataMember] public double TiltE;
        /// <summary>Collimation (cone) error: the tube not square to the altitude axis.</summary>
        [DataMember] public double CA;
        /// <summary>Non-perpendicularity of the altitude and azimuth axes.</summary>
        [DataMember] public double NPAE;
        /// <summary>Tube flexure, proportional to cos(altitude).</summary>
        [DataMember] public double TF;

        internal const int Count = 7;

        internal double[] ToArray() => new[] { IA, IE, TiltN, TiltE, CA, NPAE, TF };

        internal static ModelTerms FromArray(double[] p) => new ModelTerms
        {
            IA = p[0], IE = p[1], TiltN = p[2], TiltE = p[3], CA = p[4], NPAE = p[5], TF = p[6],
        };

        public override string ToString()
        {
            string f(double r) => (AstroMath.ToDeg(r) * 3600).ToString("F0") + "\"";
            return "IA " + (AstroMath.ToDeg(IA)).ToString("F3") + "° IE " + (AstroMath.ToDeg(IE)).ToString("F3")
                + "° tiltN " + f(TiltN) + " tiltE " + f(TiltE) + " CA " + f(CA) + " NPAE " + f(NPAE) + " TF " + f(TF);
        }
    }

    /// <summary>
    /// Maps between where a star is in the sky (apparent alt/az) and the mount's own axis angles.
    ///
    /// The model is exact for a tilted base (an unlevel Dob is the biggest error source) and adds the
    /// usual small terms on top. Terms are switched on as syncs accumulate, so a single sync just fixes
    /// the zero points, two syncs also solve the base tilt, and more points add cone, axis
    /// non-perpendicularity and tube flexure. Every N.I.N.A. plate-solve sync becomes a point.
    /// </summary>
    public sealed class PointingModel
    {
        private readonly object _lock = new object();
        private readonly List<SyncPoint> _points = new List<SyncPoint>();
        private ModelTerms _terms = new ModelTerms();

        public int MaxPoints { get; set; } = 40;
        /// <summary>A new sync this close to an existing point replaces it, radians.</summary>
        public double ReplaceRadius { get; set; } = AstroMath.ToRad(3.0);
        /// <summary>
        /// When a sync disagrees with the model by more than this, the base has probably moved
        /// (bumped, clutch slipped); the model restarts from the new point. Only applied once the model
        /// has at least three well spread points. Radians; 0 disables.
        /// </summary>
        public double ResetThreshold { get; set; } = AstroMath.ToRad(2.0);

        public ModelTerms Terms { get { lock (_lock) return _terms; } }
        public int PointCount { get { lock (_lock) return _points.Count; } }
        public IReadOnlyList<SyncPoint> Points { get { lock (_lock) return _points.ToList(); } }

        /// <summary>RMS error of the fit on the sky, radians (0 with fewer than three points).</summary>
        public double RmsError { get; private set; }

        public event EventHandler Changed;

        public void Clear()
        {
            lock (_lock)
            {
                _points.Clear();
                _terms = new ModelTerms();
                RmsError = 0;
            }
            Changed?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>Adds a sync point and refits. Returns a description of what happened, for the log.</summary>
        public string AddPoint(AltAz sky, AltAz mount, DateTime utc)
        {
            string note;
            lock (_lock)
            {
                note = "";
                // Only judge a sync against the model once the model can describe the base tilt;
                // before that an unlevel base alone can put a correct sync degrees away.
                bool shapeKnown = ActiveTerms(_points.Count, SpreadLocked())[2];
                if (shapeKnown && _points.Count >= 3 && ResetThreshold > 0)
                {
                    var predicted = SkyToMountLocked(sky);
                    double err = Separation(predicted, mount);
                    if (err > ResetThreshold)
                    {
                        note = "Sync was " + AstroMath.ToDeg(err).ToString("F2") + "° from the model; assuming the base moved and restarting the model. ";
                        _points.Clear();
                    }
                }

                var p = new SyncPoint { SkyAz = sky.Az, SkyAlt = sky.Alt, MountAz = mount.Az, MountAlt = mount.Alt, TimeUtc = utc };
                int replaced = _points.RemoveAll(q => Separation(new AltAz(q.SkyAz, q.SkyAlt), sky) < ReplaceRadius);
                if (replaced > 0) note += "Replaced " + replaced + " nearby point(s). ";
                _points.Add(p);
                while (_points.Count > MaxPoints) _points.RemoveAt(0);
                FitLocked();
                note += "Model now has " + _points.Count + " point(s): " + _terms + (RmsError > 0 ? " rms " + (AstroMath.ToDeg(RmsError) * 3600).ToString("F0") + "\"" : "");
            }
            Changed?.Invoke(this, EventArgs.Empty);
            return note;
        }

        /// <summary>Sky (apparent) position to mount axis angles. Mount azimuth is wrapped to [0, 2π).</summary>
        public AltAz SkyToMount(AltAz sky)
        {
            lock (_lock) return SkyToMountLocked(sky);
        }

        /// <summary>Mount axis angles to sky (apparent) position.</summary>
        public AltAz MountToSky(AltAz mount)
        {
            lock (_lock)
            {
                var p = _terms.ToArray();
                // Newton iteration on the forward model, starting from the index-only guess.
                var sky = new AltAz(AstroMath.Wrap2Pi(mount.Az - p[0]), mount.Alt - p[1]);
                for (int i = 0; i < 20; i++)
                {
                    var f = Forward(p, sky);
                    double dAz = AstroMath.WrapPi(mount.Az - f.Az);
                    double dAlt = mount.Alt - f.Alt;
                    if (Math.Abs(dAz) < 1e-12 && Math.Abs(dAlt) < 1e-12) break;
                    const double h = 1e-7;
                    var fa = Forward(p, new AltAz(sky.Az + h, sky.Alt));
                    var fe = Forward(p, new AltAz(sky.Az, sky.Alt + h));
                    double j11 = AstroMath.WrapPi(fa.Az - f.Az) / h, j12 = AstroMath.WrapPi(fe.Az - f.Az) / h;
                    double j21 = (fa.Alt - f.Alt) / h, j22 = (fe.Alt - f.Alt) / h;
                    double det = j11 * j22 - j12 * j21;
                    if (Math.Abs(det) < 1e-12) break;
                    sky.Az = AstroMath.Wrap2Pi(sky.Az + (j22 * dAz - j12 * dAlt) / det);
                    sky.Alt += (-j21 * dAz + j11 * dAlt) / det;
                }
                return sky;
            }
        }

        /// <summary>Replaces all points (used when restoring a saved session) and refits.</summary>
        public void ReplacePoints(IEnumerable<SyncPoint> points)
        {
            lock (_lock)
            {
                _points.Clear();
                _points.AddRange(points);
                FitLocked();
            }
            Changed?.Invoke(this, EventArgs.Empty);
        }

        public void Save(string path)
        {
            List<SyncPoint> copy;
            lock (_lock) copy = _points.ToList();
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            using (var fs = File.Create(path))
            {
                new DataContractJsonSerializer(typeof(List<SyncPoint>)).WriteObject(fs, copy);
            }
        }

        public void Load(string path)
        {
            List<SyncPoint> loaded;
            using (var fs = File.OpenRead(path))
            {
                loaded = (List<SyncPoint>)new DataContractJsonSerializer(typeof(List<SyncPoint>)).ReadObject(fs);
            }
            lock (_lock)
            {
                _points.Clear();
                _points.AddRange(loaded ?? new List<SyncPoint>());
                FitLocked();
            }
            Changed?.Invoke(this, EventArgs.Empty);
        }

        // ----- model -----

        private AltAz SkyToMountLocked(AltAz sky)
        {
            var m = Forward(_terms.ToArray(), sky);
            return new AltAz(AstroMath.Wrap2Pi(m.Az), m.Alt);
        }

        /// <summary>
        /// Forward model. The sky vector is first expressed in the frame of the tilted azimuth axis
        /// (an exact rotation), then the small mechanical terms are added.
        /// </summary>
        internal static AltAz Forward(double[] p, AltAz sky)
        {
            double ce = Math.Cos(sky.Alt);
            double x = ce * Math.Cos(sky.Az), y = ce * Math.Sin(sky.Az), z = Math.Sin(sky.Alt);

            // Rotation vector that carries the tilted axis (tN, tE, 1) back to the zenith.
            double wx = p[3], wy = -p[2];
            double angle = Math.Sqrt(wx * wx + wy * wy);
            if (angle > 1e-15)
            {
                double kx = wx / angle, ky = wy / angle; // unit axis, kz = 0
                double c = Math.Cos(angle), s = Math.Sin(angle);
                double kDotV = kx * x + ky * y;
                // k × v with kz = 0
                double cx = ky * z, cy = -kx * z, cz = kx * y - ky * x;
                double nx = x * c + cx * s + kx * kDotV * (1 - c);
                double ny = y * c + cy * s + ky * kDotV * (1 - c);
                double nz = z * c + cz * s;
                x = nx; y = ny; z = nz;
            }

            double alt = Math.Asin(Math.Max(-1, Math.Min(1, z)));
            double az = Math.Atan2(y, x);
            double cosAlt = Math.Max(Math.Cos(alt), 1e-6);
            double tanAlt = Math.Tan(alt);
            az += p[0] + p[4] / cosAlt + p[5] * tanAlt;
            alt += p[1] + p[6] * Math.Cos(alt);
            return new AltAz(az, alt);
        }

        /// <summary>Which terms are fitted for a given number of points.</summary>
        /// <param name="points">Number of sync points.</param>
        /// <param name="spread">Largest separation between any two points, radians. Points bunched in one patch
        /// of sky cannot tell the shape terms apart, so those stay off until the points are spread out.</param>
        internal static bool[] ActiveTerms(int points, double spread)
        {
            bool spreadOk = spread >= AstroMath.ToRad(15);
            return new[]
            {
                points >= 1,                // IA
                points >= 1,                // IE
                points >= 2 && spreadOk,    // TiltN
                points >= 2 && spreadOk,    // TiltE
                points >= 4 && spreadOk,    // CA
                points >= 6 && spreadOk,    // NPAE
                points >= 8 && spreadOk,    // TF
            };
        }

        private double SpreadLocked()
        {
            double best = 0;
            for (int i = 0; i < _points.Count; i++)
                for (int j = i + 1; j < _points.Count; j++)
                    best = Math.Max(best, Separation(new AltAz(_points[i].SkyAz, _points[i].SkyAlt), new AltAz(_points[j].SkyAz, _points[j].SkyAlt)));
            return best;
        }

        private void FitLocked()
        {
            int n = _points.Count;
            if (n == 0)
            {
                _terms = new ModelTerms();
                RmsError = 0;
                return;
            }

            var active = ActiveTerms(n, SpreadLocked());
            var idx = Enumerable.Range(0, ModelTerms.Count).Where(i => active[i]).ToArray();

            // Starting point: zero points from the circular mean of the residuals.
            double sx = 0, sy = 0, se = 0;
            foreach (var q in _points)
            {
                double d = q.MountAz - q.SkyAz;
                sx += Math.Cos(d); sy += Math.Sin(d);
                se += q.MountAlt - q.SkyAlt;
            }
            var p = new double[ModelTerms.Count];
            p[0] = Math.Atan2(sy, sx);
            p[1] = se / n;

            // Levenberg–Marquardt on the residuals, az residuals weighted by cos(alt) so they are true sky angles.
            double lambda = 1e-3;
            double cost = Cost(p);
            bool converged = false;
            for (int iter = 0; iter < 50 && !converged; iter++)
            {
                int m = idx.Length;
                var jtj = new double[m, m];
                var jtr = new double[m];
                var r0 = Residuals(p);
                for (int k = 0; k < m; k++)
                {
                    var pk = (double[])p.Clone();
                    const double h = 1e-7;
                    pk[idx[k]] += h;
                    var rk = Residuals(pk);
                    var col = new double[r0.Length];
                    for (int i = 0; i < r0.Length; i++) col[i] = (rk[i] - r0[i]) / h;
                    for (int i = 0; i < r0.Length; i++) jtr[k] += col[i] * r0[i];
                    for (int l = 0; l <= k; l++)
                    {
                        var pl = (double[])p.Clone();
                        pl[idx[l]] += h;
                        var rl = l == k ? rk : Residuals(pl);
                        double sum = 0;
                        for (int i = 0; i < r0.Length; i++) sum += col[i] * (rl[i] - r0[i]) / h;
                        jtj[k, l] = sum;
                        jtj[l, k] = sum;
                    }
                }

                bool improved = false;
                for (int attempt = 0; attempt < 10 && !improved; attempt++)
                {
                    var a = new double[m, m];
                    for (int k = 0; k < m; k++)
                        for (int l = 0; l < m; l++)
                            a[k, l] = jtj[k, l] + (k == l ? lambda * (jtj[k, k] + 1e-12) : 0);
                    var b = new double[m];
                    for (int k = 0; k < m; k++) b[k] = -jtr[k];
                    var step = Solve(a, b);
                    if (step == null) { lambda *= 10; continue; }
                    var trial = (double[])p.Clone();
                    for (int k = 0; k < m; k++) trial[idx[k]] += step[k];
                    double c = Cost(trial);
                    if (c < cost)
                    {
                        double gain = cost - c;
                        p = trial;
                        cost = c;
                        lambda = Math.Max(lambda / 10, 1e-12);
                        improved = true;
                        if (gain < 1e-24) converged = true;
                    }
                    else
                    {
                        lambda *= 10;
                    }
                }
                if (!improved) break;
            }

            p[0] = AstroMath.WrapPi(p[0]);
            _terms = ModelTerms.FromArray(p);
            int dof = 2 * n - idx.Length;
            RmsError = dof > 0 ? Math.Sqrt(cost / n) : 0;
        }

        private double[] Residuals(double[] p)
        {
            var r = new double[_points.Count * 2];
            for (int i = 0; i < _points.Count; i++)
            {
                var q = _points[i];
                var f = Forward(p, new AltAz(q.SkyAz, q.SkyAlt));
                r[2 * i] = AstroMath.WrapPi(q.MountAz - f.Az) * Math.Cos(q.SkyAlt);
                r[2 * i + 1] = q.MountAlt - f.Alt;
            }
            return r;
        }

        private double Cost(double[] p)
        {
            double s = 0;
            foreach (var v in Residuals(p)) s += v * v;
            return s;
        }

        /// <summary>Gaussian elimination with partial pivoting. Returns null if singular.</summary>
        internal static double[] Solve(double[,] a, double[] b)
        {
            int n = b.Length;
            var m = (double[,])a.Clone();
            var x = (double[])b.Clone();
            for (int col = 0; col < n; col++)
            {
                int piv = col;
                for (int r = col + 1; r < n; r++)
                    if (Math.Abs(m[r, col]) > Math.Abs(m[piv, col])) piv = r;
                if (Math.Abs(m[piv, col]) < 1e-30) return null;
                if (piv != col)
                {
                    for (int c = 0; c < n; c++) { var t = m[col, c]; m[col, c] = m[piv, c]; m[piv, c] = t; }
                    var tb = x[col]; x[col] = x[piv]; x[piv] = tb;
                }
                for (int r = col + 1; r < n; r++)
                {
                    double f = m[r, col] / m[col, col];
                    for (int c = col; c < n; c++) m[r, c] -= f * m[col, c];
                    x[r] -= f * x[col];
                }
            }
            for (int r = n - 1; r >= 0; r--)
            {
                double s = x[r];
                for (int c = r + 1; c < n; c++) s -= m[r, c] * x[c];
                x[r] = s / m[r, r];
            }
            return x;
        }

        /// <summary>Great-circle separation of two horizontal positions, radians.</summary>
        public static double Separation(AltAz a, AltAz b)
        {
            double s = Math.Sin(a.Alt) * Math.Sin(b.Alt) + Math.Cos(a.Alt) * Math.Cos(b.Alt) * Math.Cos(a.Az - b.Az);
            return Math.Acos(Math.Max(-1, Math.Min(1, s)));
        }
    }
}
