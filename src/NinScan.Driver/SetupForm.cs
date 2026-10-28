using System;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.Text;
using System.Windows.Forms;
using NinScan.Core.Astronomy;
using NinScan.Core.Infrastructure;
using NinScan.Core.Mount;
using NinScan.Core.Protocol;

namespace NinScan.Driver
{
    /// <summary>Settings window, opened from the ASCOM chooser's Properties button or by running the exe.</summary>
    internal sealed class SetupForm : Form
    {
        private readonly ComboBox _port = new ComboBox { DropDownStyle = ComboBoxStyle.DropDown, Width = 140 };
        private readonly ComboBox _baud = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 140 };
        private readonly TextBox _lat = Num();
        private readonly TextBox _lon = Num();
        private readonly TextBox _elev = Num();
        private readonly CheckBox _refraction = new CheckBox { Text = "Apply atmospheric refraction", AutoSize = true };
        private readonly CheckBox _revAz = new CheckBox { Text = "Reverse azimuth", AutoSize = true };
        private readonly CheckBox _revAlt = new CheckBox { Text = "Reverse altitude", AutoSize = true };
        private readonly TextBox _minAlt = Num();
        private readonly TextBox _maxAlt = Num();
        private readonly TextBox _azTravel = Num();
        private readonly TextBox _manual = Num();
        private readonly TextBox _resetModel = Num();
        private readonly TextBox _aperture = Num();
        private readonly TextBox _focal = Num();
        private readonly ComboBox _aux = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 140 };
        private readonly CheckBox _debug = new CheckBox { Text = "Log every serial command (for troubleshooting)", AutoSize = true };
        private readonly Label _modelInfo = new Label { AutoSize = true, MaximumSize = new Size(420, 0) };
        private readonly Label _status = new Label { AutoSize = true, MaximumSize = new Size(420, 0) };
        private readonly MountSettings _settings;

        public SetupForm()
        {
            _settings = SharedMount.Engine.Settings;
            Text = "NinScan settings";
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = MinimizeBox = false;
            StartPosition = FormStartPosition.CenterScreen;
            AutoSize = true;
            AutoSizeMode = AutoSizeMode.GrowAndShrink;
            Padding = new Padding(10);
            Font = SystemFonts.MessageBoxFont;

            var grid = new TableLayoutPanel { ColumnCount = 2, AutoSize = true, Dock = DockStyle.Fill };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            Heading(grid, "Connection");
            var refresh = new Button { Text = "Refresh", AutoSize = true };
            refresh.Click += (s, e) => FillPorts();
            var test = new Button { Text = "Test connection", AutoSize = true };
            test.Click += (s, e) => TestConnection();
            Row(grid, "COM port", Flow(_port, refresh));
            _baud.Items.AddRange(new object[] { "Auto", "9600", "115200" });
            Row(grid, "Baud rate", Flow(_baud, test));

            Heading(grid, "Site");
            Row(grid, "Latitude (°, N positive)", _lat);
            Row(grid, "Longitude (°, E positive)", _lon);
            Row(grid, "Elevation (m)", _elev);
            Row(grid, "", _refraction);

            Heading(grid, "Mount");
            Row(grid, "", Flow(_revAz, _revAlt));
            Row(grid, "Lowest altitude (°)", _minAlt);
            Row(grid, "Highest altitude (°)", _maxAlt);
            Row(grid, "Azimuth travel either way (°)", _azTravel);
            _aux.Items.AddRange(new object[] { "Leave as is", "On", "Off" });
            Row(grid, "Freedom Find encoders", _aux);
            Row(grid, "Hand-push detection (°, 0 = off)", _manual);
            Row(grid, "Restart model if a sync is off by (°)", _resetModel);
            Row(grid, "Aperture / focal length (mm)", Flow(_aperture, _focal));

            Heading(grid, "Pointing model");
            var clear = new Button { Text = "Clear model", AutoSize = true };
            clear.Click += (s, e) => { SharedMount.Engine.ClearModel(); ShowModel(); };
            Row(grid, "", Flow(_modelInfo, clear));

            Heading(grid, "Troubleshooting");
            var logs = new Button { Text = "Open log folder", AutoSize = true };
            logs.Click += (s, e) =>
            {
                System.IO.Directory.CreateDirectory(SharedMount.LogDirectory);
                Process.Start("explorer.exe", SharedMount.LogDirectory);
            };
            Row(grid, "", Flow(_debug, logs));
            Row(grid, "", _status);

            var ok = new Button { Text = "OK", DialogResult = DialogResult.None, AutoSize = true };
            var cancel = new Button { Text = "Cancel", DialogResult = DialogResult.Cancel, AutoSize = true };
            ok.Click += (s, e) => Save();
            AcceptButton = ok;
            CancelButton = cancel;
            var buttons = new FlowLayoutPanel { FlowDirection = FlowDirection.RightToLeft, AutoSize = true, Dock = DockStyle.Fill };
            buttons.Controls.Add(cancel);
            buttons.Controls.Add(ok);
            grid.Controls.Add(buttons, 0, grid.RowCount);
            grid.SetColumnSpan(buttons, 2);

            Controls.Add(grid);
            Load += (s, e) => Fill();
        }

        private void Fill()
        {
            FillPorts();
            _port.Text = _settings.ComPort;
            _baud.SelectedIndex = _settings.BaudRate == 9600 ? 1 : _settings.BaudRate == 115200 ? 2 : 0;
            _lat.Text = F(_settings.LatitudeDeg);
            _lon.Text = F(_settings.LongitudeDeg);
            _elev.Text = F(_settings.ElevationM);
            _refraction.Checked = _settings.Refraction;
            _revAz.Checked = _settings.ReverseAzimuth;
            _revAlt.Checked = _settings.ReverseAltitude;
            _minAlt.Text = F(_settings.MinAltitudeDeg);
            _maxAlt.Text = F(_settings.MaxAltitudeDeg);
            _azTravel.Text = F(_settings.AzimuthTravelDeg);
            _aux.SelectedIndex = (int)_settings.AuxEncoders;
            _manual.Text = F(_settings.ManualMoveThresholdDeg);
            _resetModel.Text = F(_settings.ModelResetThresholdDeg);
            _aperture.Text = F(_settings.ApertureMm);
            _focal.Text = F(_settings.FocalLengthMm);
            _debug.Checked = _settings.DebugLogging;
            ShowModel();
            var e = SharedMount.Engine;
            _status.Text = e.Connected
                ? "Connected (" + e.ConnectionDescription + ", board " + e.BoardInfo?.VersionText + "). A new COM port takes effect on the next connect."
                : "Not connected.";
        }

        private void FillPorts()
        {
            string current = _port.Text;
            _port.Items.Clear();
            _port.Items.Add(MountSettings.SimulatorPort);
            _port.Items.AddRange(SerialTransport.AvailablePorts());
            _port.Text = current;
        }

        private void ShowModel()
        {
            var m = SharedMount.Engine.Model;
            _modelInfo.Text = m.PointCount == 0
                ? "No sync points yet. Each plate-solve sync adds one."
                : m.PointCount + " sync point(s)" + (m.RmsError > 0 ? ", fit " + (AstroMath.ToDeg(m.RmsError) * 3600).ToString("F0") + "″ rms" : "")
                    + "\n" + m.Terms;
        }

        private void Save()
        {
            try
            {
                var s = _settings.Clone();
                s.ComPort = _port.Text.Trim();
                s.BaudRate = _baud.SelectedIndex == 1 ? 9600 : _baud.SelectedIndex == 2 ? 115200 : 0;
                s.LatitudeDeg = P(_lat, "latitude");
                s.LongitudeDeg = P(_lon, "longitude");
                s.ElevationM = P(_elev, "elevation");
                s.Refraction = _refraction.Checked;
                s.ReverseAzimuth = _revAz.Checked;
                s.ReverseAltitude = _revAlt.Checked;
                s.MinAltitudeDeg = P(_minAlt, "lowest altitude");
                s.MaxAltitudeDeg = P(_maxAlt, "highest altitude");
                s.AzimuthTravelDeg = P(_azTravel, "azimuth travel");
                s.AuxEncoders = (AuxEncoderMode)Math.Max(0, _aux.SelectedIndex);
                s.ManualMoveThresholdDeg = P(_manual, "hand-push detection");
                s.ModelResetThresholdDeg = P(_resetModel, "model restart");
                s.ApertureMm = P(_aperture, "aperture");
                s.FocalLengthMm = P(_focal, "focal length");
                s.DebugLogging = _debug.Checked;
                if (string.IsNullOrEmpty(s.ComPort)) throw new ArgumentException("Choose a COM port, or SIMULATOR.");
                SharedMount.SaveSettings(s);
                DialogResult = DialogResult.OK;
                Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, "NinScan", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        /// <summary>Talks to the board directly and reports what it says, without starting the engine.</summary>
        private void TestConnection()
        {
            if (SharedMount.Engine.Connected)
            {
                _status.Text = "Already connected and working (" + SharedMount.Engine.ConnectionDescription + ").";
                return;
            }
            string port = _port.Text.Trim();
            if (string.Equals(port, MountSettings.SimulatorPort, StringComparison.OrdinalIgnoreCase))
            {
                _status.Text = "The simulator needs no test.";
                return;
            }
            Cursor = Cursors.WaitCursor;
            var sb = new StringBuilder();
            try
            {
                var bauds = _baud.SelectedIndex == 1 ? new[] { 9600 } : _baud.SelectedIndex == 2 ? new[] { 115200 } : new[] { 9600, 115200 };
                foreach (var baud in bauds)
                {
                    using (var t = new SerialTransport(port, baud, SharedMount.Log))
                    {
                        var reply = t.Transact(":e1\r");
                        if (reply == null || !reply.StartsWith("="))
                        {
                            sb.AppendLine(baud + " baud: no reply.");
                            continue;
                        }
                        var mc = new MotorController(t, SharedMount.Log);
                        var info = mc.InquireVersion();
                        var az = mc.InquireAxisParameters(Axis.Azimuth);
                        var alt = mc.InquireAxisParameters(Axis.Altitude);
                        sb.AppendLine("Mount found at " + baud + " baud: board " + info.VersionText + ", " + info.MountName + ".");
                        sb.AppendLine("Azimuth: " + az.CountsPerRevolution + " steps/rev, position " + mc.InquirePosition(Axis.Azimuth) + ", " + mc.InquireStatus(Axis.Azimuth) + ".");
                        sb.AppendLine("Altitude: " + alt.CountsPerRevolution + " steps/rev, position " + mc.InquirePosition(Axis.Altitude) + ", " + mc.InquireStatus(Axis.Altitude) + ".");
                        var f = mc.InquireFeatures(Axis.Azimuth);
                        sb.AppendLine("Extended features: " + (f.HasValue ? "0x" + f.Value.ToString("X6") : "not reported") + ".");
                        SharedMount.Log.Info("Connection test: " + sb.ToString().Replace(Environment.NewLine, " "));
                        break;
                    }
                }
            }
            catch (Exception ex)
            {
                sb.AppendLine("Failed: " + ex.Message);
            }
            finally
            {
                Cursor = Cursors.Default;
            }
            _status.Text = sb.ToString().Trim();
        }

        // ------------------------------------------------------------------ layout helpers

        private static TextBox Num() => new TextBox { Width = 90 };

        private static string F(double v) => v.ToString("0.######", CultureInfo.InvariantCulture);

        private static double P(TextBox box, string name)
        {
            var text = box.Text.Trim().Replace(',', '.');
            if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
                throw new ArgumentException("The " + name + " must be a number.");
            return v;
        }

        private static void Heading(TableLayoutPanel grid, string text)
        {
            var l = new Label { Text = text, AutoSize = true, Font = new Font(SystemFonts.MessageBoxFont, FontStyle.Bold), Margin = new Padding(0, 10, 0, 2) };
            grid.Controls.Add(l, 0, grid.RowCount);
            grid.SetColumnSpan(l, 2);
            grid.RowCount++;
        }

        private static void Row(TableLayoutPanel grid, string label, Control control)
        {
            grid.Controls.Add(new Label { Text = label, AutoSize = true, Anchor = AnchorStyles.Left, Margin = new Padding(3, 6, 12, 3) }, 0, grid.RowCount);
            grid.Controls.Add(control, 1, grid.RowCount);
            grid.RowCount++;
        }

        private static FlowLayoutPanel Flow(params Control[] controls)
        {
            var p = new FlowLayoutPanel { AutoSize = true, WrapContents = false, Margin = new Padding(0) };
            p.Controls.AddRange(controls);
            return p;
        }
    }
}
