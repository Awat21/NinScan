using System;
using NinScan.Core.Infrastructure;
using NinScan.Core.Protocol;
using NinScan.Core.Simulation;
using Xunit;

namespace NinScan.Core.Tests
{
    public class ProtocolTests
    {
        [Theory]
        [InlineData(0x123456, "563412")]
        [InlineData(0x800000, "000080")]
        [InlineData(0, "000000")]
        [InlineData(0xFFFFFF, "FFFFFF")]
        public void Encode24_IsLittleEndianHex(int value, string expected)
        {
            Assert.Equal(expected, MotorCodec.Encode24(value));
            Assert.Equal(value, MotorCodec.Decode24(expected));
        }

        [Fact]
        public void Positions_AreOffsetAroundTheMiddleOfTheRange()
        {
            Assert.Equal("000080", MotorCodec.EncodePosition(0));
            Assert.Equal(-1, MotorCodec.DecodePosition("FFFF7F"));
            Assert.Equal(1000, MotorCodec.DecodePosition(MotorCodec.EncodePosition(1000)));
        }

        [Fact]
        public void BuildCommand_FramesWithColonAxisAndReturn()
        {
            Assert.Equal(":G220\r", MotorCodec.BuildCommand('G', Axis.Altitude, "20"));
            Assert.Equal(":F3\r", MotorCodec.BuildCommand('F', Axis.Both));
        }

        [Fact]
        public void ParseReply_ErrorsCarryTheCode()
        {
            Assert.Equal("101", MotorCodec.ParseReply(":f1\r", "=101\r"));
            var ex = Assert.Throws<MotorErrorException>(() => MotorCodec.ParseReply(":G10\r", "!2\r"));
            Assert.Equal(2, ex.Code);
            Assert.Throws<MotorTimeoutException>(() => MotorCodec.ParseReply(":j1\r", null));
        }

        [Fact]
        public void Controller_ReadsSimulatorParameters()
        {
            var clock = new ManualClock(new DateTime(2026, 1, 1, 22, 0, 0, DateTimeKind.Utc));
            var sim = new SimulatedMotorBoard(clock, countsPerRevolution: 2000000, timerFrequency: 64935, highSpeedRatio: 32, version: "0322A0");
            var mc = new MotorController(sim, null);
            var info = mc.InquireVersion();
            Assert.Equal(3, info.MajorVersion);
            Assert.Equal(0x22, info.MinorVersion);
            Assert.Equal(0xA0, info.MountCode);
            var p = mc.InquireAxisParameters(Axis.Azimuth);
            Assert.Equal(2000000, p.CountsPerRevolution);
            Assert.Equal(64935, p.TimerFrequency);
            Assert.Equal(32, p.HighSpeedRatio);
        }

        [Fact]
        public void Simulator_RejectsModeChangeWhileRunning()
        {
            var clock = new ManualClock(new DateTime(2026, 1, 1, 22, 0, 0, DateTimeKind.Utc));
            var sim = new SimulatedMotorBoard(clock);
            var mc = new MotorController(sim, null);
            mc.InitialisationDone();
            mc.SetMotionMode(Axis.Azimuth, MotionMode.SlewSlow, false);
            mc.SetStepPeriod(Axis.Azimuth, 1000);
            mc.Start(Axis.Azimuth);
            var ex = Assert.Throws<MotorErrorException>(() => mc.SetMotionMode(Axis.Azimuth, MotionMode.SlewSlow, true));
            Assert.Equal(2, ex.Code);
        }

        [Fact]
        public void Simulator_SlowSlewRunsAtTimerOverPeriod()
        {
            var clock = new ManualClock(new DateTime(2026, 1, 1, 22, 0, 0, DateTimeKind.Utc));
            var sim = new SimulatedMotorBoard(clock, timerFrequency: 50000);
            var mc = new MotorController(sim, null);
            mc.InitialisationDone();
            mc.SetMotionMode(Axis.Altitude, MotionMode.SlewSlow, true);
            mc.SetStepPeriod(Axis.Altitude, 500); // 100 steps/s
            mc.Start(Axis.Altitude);
            clock.Advance(TimeSpan.FromSeconds(10));
            Assert.Equal(-1000, mc.InquirePosition(Axis.Altitude));
            var st = mc.InquireStatus(Axis.Altitude);
            Assert.True(st.Running);
            Assert.True(st.SlewMode);
            Assert.True(st.Reverse);
        }

        [Fact]
        public void Simulator_GotoArrivesExactlyAndStops()
        {
            var clock = new ManualClock(new DateTime(2026, 1, 1, 22, 0, 0, DateTimeKind.Utc));
            var sim = new SimulatedMotorBoard(clock);
            var mc = new MotorController(sim, null);
            mc.InitialisationDone();
            mc.SetMotionMode(Axis.Azimuth, MotionMode.GotoFast, false);
            mc.SetGotoIncrement(Axis.Azimuth, 500000);
            mc.Start(Axis.Azimuth);
            clock.Advance(TimeSpan.FromSeconds(1));
            Assert.True(mc.InquireStatus(Axis.Azimuth).Running);
            clock.Advance(TimeSpan.FromSeconds(120));
            Assert.False(mc.InquireStatus(Axis.Azimuth).Running);
            Assert.Equal(500000, mc.InquirePosition(Axis.Azimuth));
        }
    }
}
