# How NinScan works

## Pieces

```
N.I.N.A. / SharpCap ──COM──▶ Telescope (ASCOM ITelescopeV4)
                                  │   one per client, all sharing:
                                  ▼
                             AltAzMount engine ── background loop, twice a second
                              │        │
                 PointingModel│        │MotorController ── ITransport ── SerialTransport (USB adapter)
                              │        │                             └─ SimulatedMotorBoard
                         AstroMath ────┘
```

## Motor protocol

Sky-Watcher's motor controller command set: `:` + command letter + axis (`1` azimuth, `2` altitude, `3` both) + hex data + `\r`. 24-bit values are sent least-significant byte first. Positions are offset by `0x800000`. Replies are `=data` or `!code`.

| Command | Use |
|---|---|
| `e`, `a`, `b`, `g` | Board version, steps per revolution, timer frequency, high-speed ratio |
| `j` / `E` | Read / set the position |
| `f` | Status: running, goto or slew mode, direction, initialised |
| `G` | Motion mode: `0` fast goto, `1` slow slew (tracking), `2` slow goto, `3` fast slew; plus direction |
| `I` | Step period: speed = timer frequency ÷ period (× high-speed ratio in fast modes) |
| `H`, `M` | Goto distance and brake point |
| `J`, `K`, `L` | Start, decelerating stop, instant stop |
| `F` | Initialisation done (set once after power-on) |
| `q`, `d`, `W` | Extended features, aux encoder reading, Freedom Find on/off (logged, not relied on) |

## Coordinates

- Clients send topocentric RA/Dec of date (ASCOM `equTopocentric`).
- RA/Dec → geometric alt/az uses the IAU 1982 sidereal time. Refraction is added with Sæmundsson's formula, and inverted exactly by iteration.
- The **apparent sky** alt/az goes through the pointing model to get **mount axis angles**, which become step counts.
- Mount azimuth is kept *unwrapped* (it can be −250° or +300°). Each goto picks the shortest turn that stays inside the travel limit.

## Pointing model

The forward model (sky → mount) first rotates the sky vector into the frame of the tilted azimuth axis (exact, so a base 3° off level is handled correctly). It then adds the small terms:

- azimuth: `IA + CA / cos(alt) + NPAE · tan(alt)`
- altitude: `IE + TF · cos(alt)`

Terms switch on as points accumulate: 1 point gives the zero points, 2 well-spread points add the base tilt, 4 add cone, 6 add NPAE, 8 add flexure. It is fitted by Levenberg–Marquardt, and the inverse is Newton iteration.

A sync more than 2° from a model that already has three spread-out points means the base moved, so the model restarts from that sync. Syncs within 3° of an existing point replace it.

## Tracking

Every cycle (500 ms by default) the engine reads both positions. It then works out where the target will be one cycle ahead and sets each axis rate to land exactly there. The control is feed-forward and error correction in one, so drift can't build up.

The rate becomes a step period (`I`) and is updated on the fly. Direction changes and switches between slow and fast mode stop the axis first, since the board refuses `G` while running.

## Gotos

The engine aims where the target will be when the slew ends. One fast goto, and if needed a second, gets within ¼°. The tracking loop then pulls in the last arcseconds, and `Slewing` only turns false once it's within 20″.

## Session memory

The motor board keeps counting as long as it's powered, so NinScan stores the pointing model and park state in `%LOCALAPPDATA%\NinScan\state.json`. On connect, the board's "initialised" flag tells a fresh power-on (start from the start position, clear the model) from a reconnect (restore both).
