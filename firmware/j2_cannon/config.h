// J2 Cannon - RedBoard firmware configuration.
//
// Everything in this file is compile-time. Values under "Defaults" can also be
// changed from the Pi (SET command) and are stored in EEPROM. The hard limits
// in "Safety" cannot be widened from the Pi - change them here and reflash.
#pragma once

#define FW_VERSION "1.0.0"
#define SERIAL_BAUD 115200

// ---------------------------------------------------------------- Pins
// The step pulse pins are driven directly from the timer ISR through PORTB
// (D8..D13 = PB0..PB5). If you move PUL pins, update the masks in stepper.cpp.
#define PIN_LED_A 5          // LED pair, wired anti-parallel between pins 5 and 6
#define PIN_LED_B 6
#define PIN_COMPRESSOR 7     // SSR input (+). HIGH = compressor on.
#define PIN_PITCH_ENA 8      // worm gear (pitch) driver
#define PIN_PITCH_DIR 9
#define PIN_PITCH_PUL 10     // PB2
#define PIN_YAW_ENA 11       // turret (yaw) driver
#define PIN_YAW_DIR 12
#define PIN_YAW_PUL 13       // PB5
#define PIN_PRESSURE A0
#define PIN_JOY_X A1         // jogs the turret
#define PIN_JOY_Y A2         // jogs the pitch
#define PIN_JOY_SW A3        // joystick button, active LOW

// 1 = red lights when pin 5 is HIGH and pin 6 is LOW. Set to 0 if the
// colours come out swapped.
#define LED_RED_IS_A 1

// TB6600 wired PUL+/DIR+/ENA+ to the pins and the minus terminals to GND.
// Driving ENA puts a TB6600 offline, so the driver is enabled when ENA is LOW.
#define DRIVER_ENABLE_LEVEL LOW

// ---------------------------------------------------------------- Safety
// Angle limits relative to the zero you set by jogging. The Pi can narrow
// these but never widen them.
#define YAW_HARD_MIN_DEG   -90.0f
#define YAW_HARD_MAX_DEG    90.0f
#define PITCH_HARD_MIN_DEG -45.0f
#define PITCH_HARD_MAX_DEG  90.0f

// Pressure sensor: 0-80 psi, 0.5-4.5 V on a 5 V supply (ratiometric).
#define SENSOR_PSI_FULL_SCALE 80.0f
#define SENSOR_ZERO_FRAC 0.10f       // 0.5 V / 5 V
#define SENSOR_SPAN_FRAC 0.80f       // (4.5 V - 0.5 V) / 5 V
#define SENSOR_FAULT_LOW_ADC 51      // below ~0.25 V: unplugged or broken wire
#define SENSOR_FAULT_HIGH_ADC 1000   // above ~4.9 V: shorted to 5 V

// The sensor saturates at 80 psi, so stay clear of the top of its range.
#define PSI_MAX_TARGET 75.0f   // highest pressure the Pi or joystick can request
#define PSI_CUTOFF 78.0f       // at or above this: compressor off + latched fault

// Compressor protection
#define COMPRESSOR_MIN_OFF_MS 3000UL   // no restart within 3 s of stopping

// Shot detection: a drop of SHOT_DROP_PSI from the peak of the last 0.5 s.
#define SHOT_DROP_PSI 6.0f
#define SHOT_MIN_PSI 8.0f

// Pi link: auto pressurising and auto moves stop if the Pi goes quiet.
#define PI_LINK_TIMEOUT_MS 1000UL
#define PI_JOG_TIMEOUT_MS 300UL

// ---------------------------------------------------------------- Defaults
// steps/deg = 200 full steps * microsteps * gear ratio / 360. These are
// placeholders until you calibrate; the Pi pushes its values on connect.
#define DEF_YAW_STEPS_PER_DEG   (200.0f * 8.0f * 12.0f / 360.0f)
#define DEF_PITCH_STEPS_PER_DEG (200.0f * 8.0f * 40.0f / 360.0f)
#define DEF_YAW_VMAX_DPS     20.0f   // auto move speed, deg/s
#define DEF_PITCH_VMAX_DPS   10.0f
#define DEF_YAW_ACCEL_DPS2   30.0f   // deg/s^2
#define DEF_PITCH_ACCEL_DPS2 20.0f
#define DEF_YAW_JOG_DPS      15.0f   // full-stick jog speed, deg/s
#define DEF_PITCH_JOG_DPS     8.0f
#define DEF_UNZEROED_JOG_FRAC 0.4f   // jog speed fraction before zeroing
#define DEF_PSI_HYST 1.0f            // top off when this far below target
#define DEF_PSI_OVERSHOOT 0.0f       // stop filling this far below target
#define DEF_FILL_STALL_S 20          // fault if no +0.5 psi in this long
#define DEF_FILL_MAX_RUN_S 300       // fault if the compressor runs this long
