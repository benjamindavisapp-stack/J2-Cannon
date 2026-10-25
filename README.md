# J2 Cannon

Control software for a pneumatic launcher that aims itself. Type in
coordinates or click the Limelight video. The cannon works out yaw, pitch
and pressure, turns, and fills the chamber. You fire with your own trigger
circuit.

```
 Limelight 3A ──USB──┐
                     │      web page (Pi screen, or any browser on the network)
 Raspberry Pi 3B+ ───┤      Python: video, clicks, ballistics, calibration
                     │
                     └─USB serial──► SparkFun RedBoard
                                     firmware: steppers, limits, compressor,
                                     pressure cutoff, joystick, LEDs, LCD
```

The RedBoard owns everything safety- or timing-critical. It enforces the
angle limits and pressure cutoff itself, so a Pi crash, a bad setting or a
bad command can't drive past them. Firing is not connected to either board.

| Folder | What |
|---|---|
| `firmware/j2_cannon/` | RedBoard sketch (Arduino IDE, no extra libraries) |
| `firmware/PROTOCOL.md` | serial commands between Pi and RedBoard |
| `firmware/test/` | runs the firmware on a PC for tests and as a virtual RedBoard |
| `pi/` | Raspberry Pi app (`python3 -m j2cannon`) and `config.toml` |

---

## Wiring (RedBoard)

| Pin | Connection | Notes |
|---|---|---|
| 5, 6 | LED pair | pin → resistor → LEDs wired anti-parallel → pin. Red = 5 high / 6 low. If the colours are swapped, set `LED_RED_IS_A 0` in `config.h`. |
| 7 | SSR input + | HIGH = compressor on. Put a 10 kΩ pull-down from pin 7 to GND so the SSR stays off while the board resets. |
| 8, 9, 10 | Pitch (worm) TB6600 ENA+, DIR+, PUL+ | ENA−, DIR−, PUL− to GND |
| 11, 12, 13 | Turret TB6600 ENA+, DIR+, PUL+ | ENA−, DIR−, PUL− to GND |
| A0 | Pressure sensor signal | sensor + to 5 V, − to GND (0.5–4.5 V = 0–80 psi) |
| A1, A2 | Joystick VRx, VRy | joystick + to 5 V |
| A3 | Joystick SW | the internal pull-up is used |
| SDA, SCL | LCD I2C backpack | address is detected automatically |

Notes:
- **ENA:** with this wiring a TB6600 is *enabled* while ENA is LOW. The firmware
  holds ENA LOW, so the motors hold position. "Disable" in the app lets the
  turret turn freely and clears the zero.
- **Pin 13** also drives the on-board LED, so it blinks with turret steps. At reset
  the bootloader flashes it three times, which is three microsteps at most.
- **SSR:** use a DC-controlled AC-output SSR rated for the compressor's
  starting current. SSRs usually fail *stuck on*. Your relief valve and e-stop
  are what protect you then, not this code.

## Flash the RedBoard

1. Open `firmware/j2_cannon/j2_cannon.ino` in the Arduino IDE.
2. Board: **Arduino Uno** (the RedBoard is Uno-compatible). Pick its port.
3. Upload. No libraries to install: Wire and EEPROM come with the IDE.

Stop the Pi app first if it is running (`sudo systemctl stop j2cannon`), or the
port will be busy. From the Pi you can also use `arduino-cli`:

```
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno firmware/j2_cannon
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno firmware/j2_cannon
```

The LCD should show `J2 CANNON 1.0.0`, then the angles and pressure.

## Set up the Pi

Raspberry Pi OS Bookworm (Python 3.11):

```
sudo apt install python3-flask python3-serial
sudo usermod -aG dialout $USER     # serial port access; log out and back in
git clone https://github.com/benjamindavisapp-stack/J2-Cannon.git
cd J2-Cannon/pi
python3 -m j2cannon
```

Open **http://localhost:8080** on the Pi, or `http://<pi-address>:8080` from your
laptop. To start it at boot, see the instructions in `pi/j2cannon.service`.

On older Pi OS (Python 3.9), also run `pip install tomli`.

The Limelight 3A appears over USB at 172.29.0.1. Check it at
http://172.29.0.1:5801 (its own web UI). If the video doesn't show in the app,
fix `stream_url` in `config.toml`.

**No hardware handy?** `python3 -m j2cannon --sim` runs everything against a
simulated RedBoard and Limelight. It works on a laptop too:
`pip install flask pyserial`. Shots fired in the simulator are logged
separately (`pi/data/sim/`).

---

## First power-up (no projectile, chamber unpressurised)

Do these once, with the barrel empty and the compressor hose disconnected.

1. **Fill in `pi/config.toml`.** Set `microsteps` to match each TB6600's DIP
   switches. Set `gear_ratio` to your best tooth-count estimate. Also fill in the
   barrel, chamber pipes, projectile, pivot height and Limelight position. Restart the app.
2. **Directions.** On the Manual tab, hold the → arrow: yaw should count up *and*
   the turret should turn **right** (seen from behind the cannon). Hold ↑: pitch
   should count up and the barrel should **rise**. If either goes the wrong way,
   set `invert = true` for that axis and restart.
3. **Joystick.** Push the stick right and up. If the turret goes left or the
   barrel goes down, set `invert_x` / `invert_y` under `[joystick]`.
4. **Zero.** Jog to straight ahead and level (a phone level on the barrel works),
   then double-click the joystick or press **Zero both**.
5. **Limits.** Jog toward each end. The turret should stop at ±90° and the
   pitch at −45° / +90°. If something physical is hit first, narrow `min_deg` /
   `max_deg` in `config.toml`.
6. **Gear ratios** (Calibrate tab): see below.
7. **Pressure.** Reconnect the compressor and **Fill to 20 psi** on the Manual
   tab. The red LED should flash while filling, the compressor should stop at 20,
   and the green LED should come on. Check the reading against a gauge.

## Every session

1. Power up and open the page. Every reset loses the zero (the chip line says
   **Not zeroed**).
2. Jog straight ahead and level, then double-click the joystick (or **Zero both**).
3. Aim:
   - **Coordinates:** forward / right / height in feet from the turret base, then **Aim**.
   - **Click the video.** "Click does" picks how distance is found:
     - *Flat ground*: the click is on the ground (or at "Target height").
     - *Click + distance*: you type the distance.
     - *AprilTag*: the Limelight measures it (needs an AprilTag pipeline with 3D enabled).
   - **Tracking:** follows whatever the Limelight's active pipeline detects, a few
     times a second, ranging by flat ground or AprilTag.
4. The cannon turns, then fills. When it says **READY: fire when clear** (and
   the green LED is on), fire with your trigger.
5. After a shot the compressor stays off. **Re-arm** refills for the same target,
   or aim somewhere new.

**STOP** (or the space bar) stops motion and the compressor. Moving the
joystick at any time takes over from the Pi.

### How it picks angle and pressure

- **Lowest pressure** (default): the gentlest shot that reaches the target, usually
  around 35–45°. **High arc** instead lobs at `lob_angle_deg` (60°).
- **Fixed pressure:** always fills to the given psi and finds the angle (low or
  high arc).
- There is no vent valve. If the chamber already holds *more* than a new
  target needs, the cannon re-aims for the pressure it has instead of waiting.
  Turn off "Use pressure already in the chamber" to stop this.
- Once filled, it re-checks the angle against the *measured* pressure and nudges
  the pitch ("Fine-tune angle").
- Shots under `min_psi` (8) are raised to 8 psi, because very weak shots aren't
  repeatable.

The camera is on the barrel. After aiming for a lob the camera points at the
sky, so use **Look at target** to bring the target back into view. For
continuous tracking where the camera keeps the target in view, use Fixed
pressure + Low arc.

## Calibration (Calibrate tab)

**Gear ratios.** Pick an axis and press **Test move** (30° by default).
Measure how far it really turned, using a protractor or tape marks for the
turret and a phone level for the pitch. Enter that and press **Apply**. The
corrected ratio is saved and sent to the RedBoard. Repeat until a test move
comes out right.

**Camera boresight.** Put a marker at a measured spot, e.g. 50 ft forward on
the ground. Enter its coordinates, set "Click does" to *Boresight*, and click the
marker in the video. This sets the camera's angle relative to the barrel.

**Shot model.** Every shot is logged automatically, with angle and pre-shot
pressure. For each one, measure the horizontal distance from the turret
centre to where it first landed and type it into *Measured ft*. Then:

- 2–3 shots: **Fit efficiency**.
- 5+ shots covering at least two clearly different angles (e.g. 15° and 45°)
  and a few pressures: **Fit efficiency + drag**.

Check the error column, then **Save this model**. Untick *Use* for any bad
shot (misfire, wind gust). Calibration lives in `pi/data/calibration.json`, so
back it up. Delete it to go back to `config.toml`.

## Joystick, LEDs and LCD

| Joystick | Does |
|---|---|
| X / Y | jog turret / pitch (overrides the Pi) |
| hold button | run the compressor while held, stopping at the set pressure (75 psi if none) |
| double-click | zero both axes here |
| click | switch LCD page (main / diagnostics) |
| hold 3 s | clear a pressure fault, if its cause is gone |

| LED | Means |
|---|---|
| off | idle |
| red flashing | filling |
| green | at the set pressure |
| red solid | pressure fault: compressor locked out |

LCD line 1: `Y+12.3 P+ 5.0 NZ`, the angles plus a flag: `NZ` not zeroed, `DS`
motors disabled, `JS` joystick, `PI` Pi connected. Line 2: `psi/set psi state`,
or the fault.

## What the firmware enforces

- Turret ±90°, pitch −45° to +90° from your zero. These hard limits are in
  `config.h`. The Pi can narrow them, never widen them. Moves are refused until
  both axes are zeroed. Before zeroing, jogging is slow and unlimited.
- Maximum set pressure 75 psi. At 78 psi the compressor cuts out and latches a
  fault. The sensor tops out at 80 psi, so the code stays below that.
- A sensor reading near 0 V or 5 V (unplugged or shorted) is a fault.
- If the compressor runs 20 s without gaining 0.5 psi, or 300 s in total, that is
  a fault. Both are adjustable in `config.toml`.
- The compressor won't restart within 3 s of stopping.
- After a shot, the compressor never refills on its own.
- If the Pi goes quiet for 1 s, auto moves and auto filling stop.
- A watchdog resets the board if the firmware hangs. A reset turns the compressor off.

Your relief valves, e-stops and two-button trigger are the real safety system.
This code is a layer on top of them.

## Tests

```
make -C firmware/test test        # 26 firmware scenarios on a simulated RedBoard
make -C firmware/test avr         # compile for the ATmega328P (needs avr-gcc + Arduino core)
make -C firmware/test fw_host     # builds the virtual RedBoard used below
cd pi && python3 -m unittest discover -s tests
```

The Python system tests run the app against the simulator. If `fw_host` has
been built, they also run against the real firmware code behind a virtual
serial port.

## Troubleshooting

| Symptom | Check |
|---|---|
| "RedBoard offline" | USB cable; `ls /dev/ttyUSB*`; user in `dialout` group; Arduino IDE serial monitor closed |
| Move refused: set zero | zero after every power-up or reset |
| Turret goes the wrong way | `invert` for that axis (app arrows) or `invert_x/y` (joystick) |
| Moves are the wrong size | calibrate the gear ratio; check `microsteps` matches the DIP switches |
| Motor stalls or skips | lower `max_speed_dps` / `accel_dps2`, check TB6600 current setting |
| FILL_STALL fault | leak, hose off, or compressor not running; raise `fill_stall_s` for a slow compressor |
| Limelight offline | http://172.29.0.1:5801 opens? Try another USB port |
| Clicks land in the wrong place | camera position in `config.toml`, then boresight |
