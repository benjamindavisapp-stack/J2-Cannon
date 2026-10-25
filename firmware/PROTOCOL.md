# RedBoard serial protocol

115200 baud, ASCII lines ending in `\n`. The Pi sends commands; the RedBoard
answers `OK <CMD>` or `ERR <CMD> <REASON>`, sends a status line every 100 ms
and events when something happens. Opening the port resets the board, which
then sends `E BOOT`.

Any line from the Pi counts as a heartbeat. If nothing arrives for 1 s, the
board sends `E LINKLOST`, stops auto pressurising (FILL/HOLD) and stops any
auto move. The joystick keeps working.

## Commands

| Command | Meaning | Errors |
|---|---|---|
| `HB` | heartbeat, no reply | |
| `MOVE <yaw> <pitch>` | move to absolute angles (deg) | `NOZERO` `LIMIT` `JOY` `DISABLED` `ARGS` |
| `JOG <y> <p>` | jog at -1..1 of jog speed; repeat within 300 ms or it stops. No `OK` reply | `JOY` `DISABLED` |
| `STOP` | decelerate both axes and hold | |
| `ZERO Y\|P\|B` | current position becomes 0 for yaw, pitch or both | `MOVING` |
| `PRESS <psi>` | fill to psi, then hold there (1..75) | `FAULT` `RANGE` |
| `PSTOP` | compressor off, pressure state IDLE | |
| `CLEAR` | clear a latched fault | `ACTIVE` (cause still present) |
| `ENABLE 0\|1` | driver enable; disabling clears both zeros | |
| `SET <key> <value>` | change a setting (RAM) | `KEY` `RANGE` |
| `SAVE` | write settings to EEPROM | `MOVING` |
| `GET` | dump settings as `CFG` lines | |
| `VER` | `OK VER <version>` | |

Setting keys: `Y.SPD` `Y.VMAX` `Y.ACC` `Y.JOG` `Y.MIN` `Y.MAX` `Y.INV` (same
with `P.` for pitch), `G.HYST` `G.PCOMP` `G.UZJOG` `G.JXINV` `G.JYINV`
`G.STALL` `G.MAXRUN`. Units: steps/deg, deg/s, deg/s², deg, psi, seconds.
`MIN`/`MAX` are clamped to the hard limits in `config.h` (yaw ±90, pitch
-45..+90).

## Status (every 100 ms)

```
S ya=12.34 pa=5.00 yt=12.34 pt=5.00 md=PP mv=0 z=3 en=1 psi=40.1 tgt=40.0 pst=HOLD c=0 f=0 j=0 adc=512
```

| Field | Meaning |
|---|---|
| `ya` `pa` | yaw / pitch now, deg |
| `yt` `pt` | yaw / pitch target, deg (= position unless moving to a target) |
| `md` | mode per axis: `H` hold, `P` position move, `V` velocity jog |
| `mv` | moving bits: 1 yaw, 2 pitch |
| `z` | zeroed bits: 1 yaw, 2 pitch |
| `en` | drivers enabled |
| `psi` `tgt` | chamber pressure and set pressure |
| `pst` | `IDLE` `FILL` `HOLD` `MAN` (joystick button) |
| `c` | compressor on |
| `f` | fault code (0 none) |
| `j` | joystick in control |
| `adc` | raw pressure sensor reading 0..1023 |

## Events

| Event | When |
|---|---|
| `E BOOT fw=<ver> cfg=eeprom\|defaults lcd=<addr>` | after reset |
| `E SHOT pre=<psi> post=<psi> ya=<deg> pa=<deg>` | pressure fell 6+ psi within 0.5 s from 8+ psi |
| `E FAULT <code> <name>` | 1 `OVERPRESSURE` (≥78 psi), 2 `SENSOR_LOW`, 3 `SENSOR_HIGH`, 4 `FILL_STALL`, 5 `FILL_TIMEOUT` |
| `E FAULTCLR` | fault cleared from the joystick |
| `E JOY` | joystick took over from the Pi |
| `E ZERO B` | zero set from the joystick |
| `E LINKLOST` | no line from the Pi for 1 s |
