# NinScan

A standalone ASCOM driver for Sky-Watcher alt-az GoTo mounts, written for the **Flextube 250P GoTo Dobsonian** on the Sky-Watcher USB adapter. You don't need the SynScan App or the hand controller.

N.I.N.A., SharpCap and other ASCOM programs pick **"NinScan (Sky-Watcher Alt-Az)"** in the ASCOM chooser. Windows starts the driver by itself when a program connects and closes it when the last one disconnects, so there's nothing to open first.

## Why it exists

The motor board in the mount only understands step counts and step rates. It doesn't know about RA/Dec, star positions or tracking. Normally the hand controller or the SynScan App does that work, which is why the SynScan App has to stay open: its ASCOM driver just passes commands through to the app.

NinScan does that work itself:

- **Coordinate conversion**: RA/Dec ↔ alt/az for your site and time, with atmospheric refraction.
- **Tracking**: an alt-az mount has to change both axis speeds all the time. NinScan recalculates them twice a second and corrects any drift.
- **Pointing model**: every sync, including N.I.N.A.'s plate-solve syncs, adds a point. The model solves for an unlevel base exactly. As points build up it adds cone error, axis non-perpendicularity and tube flexure, so gotos improve the longer you image.
- **Safety**: altitude limits, an azimuth travel limit (so cables don't wind up), and motors stopped when the last program disconnects.
- **Hand-push detection**: if you push the tube while it's tracking, NinScan carries on tracking from the new spot instead of fighting you back. This relies on Freedom Find keeping the count.

## Status

**v0.1, not yet tested on real hardware.** Everything works against the built-in simulator and has 40 automated tests. The next step is first light on the 250P: see [docs/INSTALL.md](docs/INSTALL.md).

| Version | Scope |
|---|---|
| v1 (this) | Connect, goto, tracking (sidereal, lunar, solar), sync and pointing model, park, manual moves, limits, Freedom Find, simulator |
| v1.1 | Field derotation with the Wanderer Astro Rotator Mini V2 (the maths is already in `AstroMath.FieldRotationRate`) |
| v2 | Pulse guiding (PHD2) on alt-az: guide pulses converted using the current sky angle at the target |

## Layout

```
src/NinScan.Core     Protocol, simulator, astronomy, pointing model, mount engine (netstandard2.0, testable anywhere)
src/NinScan.Driver   ASCOM COM local server and settings window (Windows, .NET Framework 4.8)
tests/               xUnit tests, mostly end-to-end against the simulator
docs/                Install and first-light guide, design notes
```

Build and test with the .NET 8 SDK: `dotnet test tests/NinScan.Core.Tests`, then `dotnet build src/NinScan.Driver -c Release`. Every push is built on Windows by GitHub Actions, and the ready-to-install driver is attached to the run as the `NinScan-driver` artifact.

See [docs/DESIGN.md](docs/DESIGN.md) for how it works.
