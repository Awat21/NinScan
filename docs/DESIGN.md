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

## Backlash and slip

**Gear play.** For each axis the engine tracks the play: motor angle minus tube angle, between −B/2 and +B/2. Moving one way pushes it to that end, and the tube only moves once it gets there. Everything the model and clients see is the *tube* angle: the motor angle less the play.

**Learning.** Each sync stores the motor angles and the gear history: the direction of the last move longer than 1°, followed by every move since. Replaying that history through the play model gives the play at the sync for any backlash value B. So B is just another term in the pointing model fit, where the residual is `motor − play(B) − model(sky)`.
- B is fitted once there are at least two syncs with the gears loaded each way.
- It is pulled towards what earlier sessions learned, weighted by how many syncs taught it (capped at 40, so the estimate can follow wear).
- It is stored in `learned.json` and survives power cycles. The pointing model doesn't, because the base moves between sessions.

**Compensation.**
- A goto aims the motor at `tube target + play after the move`.
- In tracking, when an axis must reverse, the play is taken up quickly instead of the tube stalling. For example, 0.3° of play at a 10″/s tracking rate would otherwise stall the tube for about 100 s.
- Reversals under 2″ are ignored, so the gears don't rattle at turning points.

**Approach direction.** If the final move would come from the wrong side, the goto first stops at least `max(1°, 2B)` past the target on the far side. A second leg then comes in from the approach side.
- The default approach is the direction the axis will move when tracking, so tracking starts with the gears already loaded.
- With the right approach, even an unlearned backlash only shifts the zero point, which the first sync absorbs.

**Slip detection.** At each sync the engine compares the motor movement since the previous sync, less the change in play, with the tube movement measured from the two plate solves.
- If the gear loading was the same at both syncs (checked by replaying both histories for several values of B), backlash can't explain any difference. Anything over `max(3′, 3 × fit rms)` counts as a clutch slip.
- If the loading changed and B isn't learned yet, the difference is reported as possible backlash and not counted.

**Tracking loop stability.** Each cycle the loop corrects 60% of the remaining error on top of the target's own motion. Corrections are capped at 0.2°/s. The board changes speed instantly at those rates, so the loop can't overshoot and ring. In testing, correcting the full error each cycle became unstable once backlash take-up was added.

## Sun safety

The Sun's position comes from the Astronomical Almanac's low-precision formulae, accurate to about 0.01°. With solar viewing off, the following checks apply whenever the Sun is less than the avoidance radius below the horizon:

- **Goto targets** inside the avoidance radius (default 15°) are refused, with a message saying why.
- **Goto paths** are sampled assuming both axes move at the same speed. If the direct path passes the Sun, the engine tries, in order:
  1. altitude first, then azimuth;
  2. azimuth first, then altitude;
  3. down low and across;
  4. up high and across.

  It uses the first clear route as waypoints, or refuses the goto if none is clear.
- **While slewing**, a goto that gets within 60% of the radius and closer than it started is stopped. This is a safety net in case the path assumption is wrong on the real mount.
- **Manual moves** stop at the edge of the zone, and can't start towards the Sun from inside it.
- **Tracking** stops if the target comes within the radius.

All of these only block moves that get *closer* to the Sun. If solar viewing is switched off while pointing at the Sun, you can still slew away or park.

Solar viewing is a per-connection flag, never saved. When the driver connects, or a client sets the site latitude or longitude, and the Sun is up, a non-blocking window asks whether solar viewing is intended. Clients can also use the `NinScan:SolarViewing` action with `on` or `off`.

## Session memory

The motor board keeps counting as long as it's powered, so NinScan stores the pointing model, gear state and park state in `%LOCALAPPDATA%\NinScan\state.json`. On connect, the board's "initialised" flag tells a fresh power-on (start from the start position, clear the model) from a reconnect (restore both).
