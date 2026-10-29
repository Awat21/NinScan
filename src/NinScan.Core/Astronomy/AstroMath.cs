using System;

namespace NinScan.Core.Astronomy
{
    /// <summary>Horizontal coordinates in radians. Azimuth runs from north through east.</summary>
    public struct AltAz
    {
        public double Az;
        public double Alt;

        public AltAz(double az, double alt)
        {
            Az = az;
            Alt = alt;
        }

        public override string ToString() => "Az " + AstroMath.ToDeg(Az).ToString("F4") + "° Alt " + AstroMath.ToDeg(Alt).ToString("F4") + "°";
    }

    /// <summary>Equatorial coordinates: right ascension in hours, declination in degrees.</summary>
    public struct RaDec
    {
        public double RaHours;
        public double DecDeg;

        public RaDec(double raHours, double decDeg)
        {
            RaHours = raHours;
            DecDeg = decDeg;
        }

        public override string ToString() => "RA " + RaHours.ToString("F5") + "h Dec " + DecDeg.ToString("F4") + "°";
    }

    public sealed class Site
    {
        public double LatitudeDeg;
        /// <summary>East positive.</summary>
        public double LongitudeDeg;
        public double ElevationM;
        public double PressureHPa = 1010;
        public double TemperatureC = 10;
    }

    /// <summary>
    /// Positional astronomy needed by an alt-az mount. Coordinates are topocentric "of date" (JNow),
    /// which is what ASCOM clients send when the driver reports a topocentric equatorial system.
    /// </summary>
    public static class AstroMath
    {
        public const double SiderealRateArcsecPerSec = 15.0410671786691;
        public const double LunarRateArcsecPerSec = 14.685;
        public const double SolarRateArcsecPerSec = 15.0;
        /// <summary>Earth rotation, radians per SI second.</summary>
        public const double EarthRateRadPerSec = SiderealRateArcsecPerSec / 3600.0 * Math.PI / 180.0;

        public static double ToRad(double deg) => deg * Math.PI / 180.0;
        public static double ToDeg(double rad) => rad * 180.0 / Math.PI;

        /// <summary>Wraps to [0, 2π).</summary>
        public static double Wrap2Pi(double a)
        {
            a %= 2 * Math.PI;
            return a < 0 ? a + 2 * Math.PI : a;
        }

        /// <summary>Wraps to (-π, π].</summary>
        public static double WrapPi(double a)
        {
            a = Wrap2Pi(a);
            return a > Math.PI ? a - 2 * Math.PI : a;
        }

        public static double Wrap24(double hours)
        {
            hours %= 24.0;
            return hours < 0 ? hours + 24.0 : hours;
        }

        public static double JulianDate(DateTime utc)
        {
            if (utc.Kind == DateTimeKind.Local) utc = utc.ToUniversalTime();
            return 2440587.5 + (utc - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalDays;
        }

        /// <summary>Greenwich mean sidereal time in hours (IAU 1982 expression; UT1 taken as UTC).</summary>
        public static double GreenwichMeanSiderealHours(DateTime utc)
        {
            double jd = JulianDate(utc);
            double d = jd - 2451545.0;
            double t = d / 36525.0;
            double deg = 280.46061837 + 360.98564736629 * d + 0.000387933 * t * t - t * t * t / 38710000.0;
            return Wrap24(deg / 15.0);
        }

        public static double LocalSiderealHours(DateTime utc, double longitudeDeg)
        {
            return Wrap24(GreenwichMeanSiderealHours(utc) + longitudeDeg / 15.0);
        }

        /// <summary>Converts hour angle / declination (radians) to geometric altitude and azimuth.</summary>
        public static AltAz HaDecToAltAz(double ha, double dec, double lat)
        {
            double sinAlt = Math.Sin(lat) * Math.Sin(dec) + Math.Cos(lat) * Math.Cos(dec) * Math.Cos(ha);
            double alt = Math.Asin(Clamp(sinAlt));
            double y = -Math.Cos(dec) * Math.Sin(ha);
            double x = Math.Sin(dec) * Math.Cos(lat) - Math.Cos(dec) * Math.Sin(lat) * Math.Cos(ha);
            return new AltAz(Wrap2Pi(Math.Atan2(y, x)), alt);
        }

        /// <summary>Converts geometric altitude and azimuth to hour angle / declination (radians).</summary>
        public static void AltAzToHaDec(AltAz h, double lat, out double ha, out double dec)
        {
            double sinDec = Math.Sin(lat) * Math.Sin(h.Alt) + Math.Cos(lat) * Math.Cos(h.Alt) * Math.Cos(h.Az);
            dec = Math.Asin(Clamp(sinDec));
            double y = -Math.Sin(h.Az) * Math.Cos(h.Alt);
            double x = Math.Sin(h.Alt) * Math.Cos(lat) - Math.Cos(h.Alt) * Math.Sin(lat) * Math.Cos(h.Az);
            ha = WrapPi(Math.Atan2(y, x));
        }

        /// <summary>Geometric altitude/azimuth of an RA/Dec at a moment, before refraction.</summary>
        public static AltAz RaDecToAltAz(RaDec rd, DateTime utc, Site site)
        {
            double lst = LocalSiderealHours(utc, site.LongitudeDeg);
            double ha = ToRad((lst - rd.RaHours) * 15.0);
            return HaDecToAltAz(ha, ToRad(rd.DecDeg), ToRad(site.LatitudeDeg));
        }

        public static RaDec AltAzToRaDec(AltAz h, DateTime utc, Site site)
        {
            AltAzToHaDec(h, ToRad(site.LatitudeDeg), out double ha, out double dec);
            double lst = LocalSiderealHours(utc, site.LongitudeDeg);
            return new RaDec(Wrap24(lst - ToDeg(ha) / 15.0), ToDeg(dec));
        }

        /// <summary>
        /// Atmospheric refraction for a true (geometric) altitude, radians. Sæmundsson's formula
        /// scaled for pressure and temperature. Returns 0 well below the horizon.
        /// </summary>
        public static double RefractionFromTrue(double trueAlt, Site site)
        {
            double h = ToDeg(trueAlt);
            if (h < -1.9) return 0;
            double rArcmin = 1.02 / Math.Tan(ToRad(h + 10.3 / (h + 5.11)));
            rArcmin *= (site.PressureHPa / 1010.0) * (283.0 / (273.0 + site.TemperatureC));
            if (rArcmin < 0) rArcmin = 0;
            return ToRad(rArcmin / 60.0);
        }

        public static double TrueToApparent(double trueAlt, Site site) => trueAlt + RefractionFromTrue(trueAlt, site);

        /// <summary>Inverse of <see cref="TrueToApparent"/>, solved by iteration so the two round-trip exactly.</summary>
        public static double ApparentToTrue(double apparentAlt, Site site)
        {
            double t = apparentAlt;
            for (int i = 0; i < 30; i++)
            {
                t = apparentAlt - RefractionFromTrue(t, site);
            }
            return t;
        }

        /// <summary>
        /// Apparent position of the Sun, good to about 0.01° (the Astronomical Almanac's low-precision formulae).
        /// Plenty for keeping a telescope away from it.
        /// </summary>
        public static RaDec SunPosition(DateTime utc)
        {
            double n = JulianDate(utc) - 2451545.0;
            double l = 280.460 + 0.9856474 * n;
            double g = ToRad(357.528 + 0.9856003 * n);
            double lambda = ToRad(l + 1.915 * Math.Sin(g) + 0.020 * Math.Sin(2 * g));
            double eps = ToRad(23.439 - 0.0000004 * n);
            double ra = Math.Atan2(Math.Cos(eps) * Math.Sin(lambda), Math.Cos(lambda));
            double dec = Math.Asin(Math.Sin(eps) * Math.Sin(lambda));
            return new RaDec(Wrap24(ToDeg(ra) / 15.0), ToDeg(dec));
        }

        /// <summary>
        /// Rate of field rotation for an alt-az mount, radians per second.
        /// Kept here for the derotation work planned for v1.1.
        /// </summary>
        public static double FieldRotationRate(AltAz h, double lat)
        {
            double c = Math.Cos(h.Alt);
            if (Math.Abs(c) < 1e-9) return double.PositiveInfinity;
            return EarthRateRadPerSec * Math.Cos(lat) * Math.Cos(h.Az) / c;
        }

        private static double Clamp(double x) => x > 1 ? 1 : (x < -1 ? -1 : x);
    }
}
