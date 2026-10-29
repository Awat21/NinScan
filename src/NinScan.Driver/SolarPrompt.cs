using System;
using System.Drawing;
using System.Windows.Forms;
using NinScan.Core.Astronomy;

namespace NinScan.Driver
{
    /// <summary>
    /// Asked when the driver connects (or the site location changes) while the Sun is up.
    /// It does not block the client: Sun avoidance stays on until someone answers yes.
    /// </summary>
    internal sealed class SolarPrompt : Form
    {
        private static SolarPrompt _open;

        public static void ShowIfNeeded()
        {
            var engine = SharedMount.Engine;
            if (_open != null || !engine.Connected || !engine.ShouldOfferSolarViewing) return;
            _open = new SolarPrompt(AstroMath.ToDeg(engine.SunPosition.Alt), engine.Settings.SunAvoidanceDeg);
            _open.FormClosed += (s, e) => _open = null;
            _open.Show();
            _open.Activate();
        }

        private SolarPrompt(double sunAltDeg, double avoidanceDeg)
        {
            Text = "NinScan: the Sun is up";
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = MinimizeBox = false;
            StartPosition = FormStartPosition.CenterScreen;
            TopMost = true;
            AutoSize = true;
            AutoSizeMode = AutoSizeMode.GrowAndShrink;
            Padding = new Padding(14);
            Font = SystemFonts.MessageBoxFont;

            var text = new Label
            {
                AutoSize = true,
                MaximumSize = new Size(440, 0),
                Text = "The Sun is up (" + sunAltDeg.ToString("F0") + "° above the horizon).\n\n"
                    + "Solar viewing is OFF: NinScan will keep the telescope at least " + avoidanceDeg + "° from the Sun.\n\n"
                    + "Is solar viewing intended?\n\n"
                    + "Only choose Yes if a proper solar filter is fitted securely over the FRONT of the telescope, "
                    + "and the finder is capped or filtered too. Solar viewing switches off again the next time NinScan connects.",
            };
            var yes = new Button { Text = "Yes, filter fitted: allow solar viewing", AutoSize = true };
            var no = new Button { Text = "No, keep away from the Sun", AutoSize = true };
            yes.Click += (s, e) =>
            {
                SharedMount.Engine.SolarViewing = true;
                Close();
            };
            no.Click += (s, e) => Close();
            AcceptButton = no;
            CancelButton = no;

            var buttons = new FlowLayoutPanel { AutoSize = true, FlowDirection = FlowDirection.RightToLeft, Dock = DockStyle.Fill, Margin = new Padding(0, 12, 0, 0) };
            buttons.Controls.Add(no);
            buttons.Controls.Add(yes);
            var layout = new TableLayoutPanel { AutoSize = true, ColumnCount = 1, Dock = DockStyle.Fill };
            layout.Controls.Add(text);
            layout.Controls.Add(buttons);
            Controls.Add(layout);
        }
    }
}
