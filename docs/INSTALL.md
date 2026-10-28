# Installing NinScan and first light

## You need

- Windows 10 or 11 with the [ASCOM Platform](https://ascom-standards.org/) 7 (N.I.N.A. users usually have it already).
- The Sky-Watcher USB adapter plugged into the mount's hand-controller port. Windows shows it as a COM port in Device Manager → Ports.
- The SynScan App **closed**. Only one program can use the COM port at a time.

## Install

1. Get the `NinScan-driver` artifact from the latest GitHub Actions run (Actions tab → Build → the newest run → Artifacts), or build it yourself.
2. Unzip it somewhere permanent, for example `C:\Program Files\NinScan`. Windows remembers this path, so don't move the folder afterwards.
3. Double-click `Register.cmd` and accept the administrator prompt. You should see "NinScan is registered".
4. Double-click `NinScan.Driver.exe` to open the settings:
   - Pick the COM port, and press **Test connection** with the mount switched on. It should report the motor board version and the steps per revolution of each axis. **Please send me that text.**
   - Enter your latitude and longitude (east positive, so the UK and Europe west of Greenwich are negative).
   - Press OK.

To remove the driver, run `Unregister.cmd`.

## First light, step by step

These checks confirm the parts the simulator can't: motor directions, speeds and the Freedom Find behaviour. Logs are in `%LOCALAPPDATA%\NinScan\logs`; the settings window has a button that opens the folder. For the first sessions, tick "Log every serial command".

1. **Start position.** Like SynScan, NinScan assumes that when you switch the mount on, the tube is **level and pointing north**. Set it there, then switch the mount on.
2. **Connect** from N.I.N.A. (Equipment → Telescope → NinScan) or SharpCap.
3. **Directions.** Use the manual arrows at a slow speed:
   - "Increase azimuth" (east) should turn the tube clockwise seen from above (north → east). If it goes the other way, tick **Reverse azimuth** in the settings.
   - "Up" should raise the tube. If not, tick **Reverse altitude**.
4. **First goto.** Slew to a bright star. It won't be centred yet, since the start position was only roughly right.
5. **Sync.** Plate-solve and sync in N.I.N.A., or centre the star by hand and sync. Do this at two or three stars in different parts of the sky. The settings window shows the model terms and fit.
6. **Track** for ten minutes and watch for drift. Then send me the log for that period.
7. **Freedom Find.** Push the tube a little by hand while tracking. The log should say "Tube moved … pushed by hand?" and tracking should carry on from the new spot. If the position doesn't change when you push, try setting Freedom Find encoders to **On**.
8. **Park** puts the tube back at the start position (level, facing north), ready for next time.

If the mount stays switched on between sessions, NinScan remembers its position and the pointing model. After a power cycle it starts fresh from the start position.

## Things to report back

- The Test connection text (board version, steps per revolution).
- Whether either axis needed reversing.
- Any "Mount rejected … error 2 (motor not stopped)" lines in the log. That would mean the board doesn't accept speed changes while running, and I'd change how tracking updates are sent.
- How far off the first goto was, and how far off after one and after three syncs.
