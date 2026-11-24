# One-Axis Stabilizing Gimbal — CMPE 146 Final Project

Keeps a small platform level on one axis while the base is tilted by hand, and
lets a joystick shift the angle it holds. Runs on an LP-MSPM0G3507 LaunchPad
under FreeRTOS.

Target device: **MSPM0G3507** (Cortex-M0+, 32 MHz) · SDK **MSPM0-SDK 2.11.0.07**
· compiler **TI Arm Clang 5.1.1.LTS**

---

## Status

Bring-up completed on hardware **2026-11-23**. Demo is **Thursday, December 3**.

### Verified on the LaunchPad

| Subsystem | Evidence |
|---|---|
| Boot and scheduler | Banner on UART, all five tasks created, telemetry streaming |
| I2C bus (I2C1, PB2/PB3) | BMI160 answers every transaction at 100 kHz |
| BMI160 **accelerometer + gyroscope** | `BMI160 ready` prints only after chip ID reads `0xD1` **and** `PMU_STATUS` confirms both sensors in normal power mode |
| Complementary filter | ±0.02 deg of noise at rest, no drift; tracks smoothly through ±90 deg |
| Joystick -> ADC -> offset | Stick deflection shifts the held angle in real time |
| Servo driver + PWM | `servo` tracks as the exact negation of `angle` across the full range, clamps safely at the limits |
| Counter-rotation direction | Tilt the base right, the arm goes left |
| Servo scale | Measured and calibrated - see "Servo calibration" below |
| Task communication | Three length-1 queues; angle, offset and status all cross task boundaries through them |

### Still to do

- Final mechanical assembly: IMU and pan-tilt base rigidly on one handle
- Re-confirm counter-rotation direction **after** mounting (orientation can flip
  the sign; the fix is `invert_servo = true`)
- Re-seat the servo horn at boot centre so travel is symmetric
- Standalone boot with no debugger attached
- Two-to-three minute reliability run
- Logic analyzer capture of the I2C traffic
- Slides

### Known weakness

The I2C wait loops have no timeout. If a wire to the IMU comes loose mid-run,
the sensor task blocks and the angle estimate freezes at exactly `0.00` (the
driver's hold-last-good-sample fallback), while the heartbeat keeps blinking -
so it *looks* alive. This was hit once during bring-up and cleared on restart.
Bounded timeouts plus a visible telemetry error and automatic bus recovery are
the planned fix.

---

## Parts

| Part | Notes |
|---|---|
| LP-MSPM0G3507 LaunchPad | Cortex-M0+, 128 KB flash, 32 KB SRAM |
| BMI160 6-axis IMU breakout | From Lab 4. **3.3 V only** — do not power from 5 V |
| Pan-tilt kit, 2 micro servos (SG90 class) | Only one servo is used; mount on its side so the servo tilts the platform |
| Dual-axis joystick module | Two 10 kΩ pots; powered from **3.3 V**, not 5 V |
| 4×AA battery holder + 4 AA cells | Servo power only, never from LaunchPad pins |
| Jumper wires | |

> **Battery note.** Four fresh alkaline cells read 6.0–6.4 V, which is at or
> just over an SG90's 6 V ceiling. Four NiMH cells (4.8 V) are gentler. If a
> servo buzzes, jitters, or browns out, suspect supply voltage first.

---

## Wiring

```
                    LP-MSPM0G3507
                  +-----------------+
   BMI160         |                 |
  +--------+      |                 |
  | VCC o--+------o 3V3  (BP pin 1) |
  | GND o--+---+--o GND  (BP pin 20)|
  | SCL o--+---|--o PB2  (BP pin 9) |
  | SDA o--+---|--o PB3  (BP pin 10)|
  +--------+   |  |                 |
               |  |                 |
   Joystick    |  |                 |
  +--------+   |  |                 |
  | +5V o--+---|--o 3V3  (BP pin 1) |   <-- 3.3 V despite the "+5V" label
  | GND o--+---+  |                 |
  | VRx o--+------o PA25 (BP pin 2) |
  | VRy    |      |                 |   (unused, one axis is graded)
  | SW     |      |                 |
  +--------+   |  |                 |
               |  | PB15 (BP pin 17)o---------+
               |  +-----------------+         |
               |                              | signal (orange)
               |     Servo                    |
               |   +---------+                |
               |   | signal o+----------------+
               |   | V+     o+------ battery + (4xAA)
               +---+ GND    o+------ battery -
                   +---------+            |
                                          |
   Battery minus MUST also tie to LaunchPad GND  <-- shared ground
```

| Signal | MCU pin | IOMUX slot | Peripheral | BoosterPack |
|---|---|---|---|---|
| I2C SCL → BMI160 | PB2 | PINCM15, func 4 | I2C1 | pin 9 |
| I2C SDA → BMI160 | PB3 | PINCM16, func 4 | I2C1 | pin 10 |
| Servo signal | PB15 | PINCM32, func 5 | TIMG8_CCP0 | pin 17 |
| Joystick VRx | PA25 | PINCM55, analog | ADC0 channel 2 | pin 2 |
| Telemetry TX | PA10 | PINCM21, func 2 | UART0 | to XDS110 |
| Heartbeat LED | PB27 | PINCM58, GPIO | LED2 green, active HIGH | on-board |

**Two power domains, one ground.** The servo draws stall currents the
LaunchPad's 3V3 regulator cannot supply, so it runs from the AA pack. The pack's
negative terminal must be bonded to LaunchPad GND or the PWM signal has no
reference and the servo will twitch or ignore commands.

**IMU goes on the base or handle, not the moving platform.** The firmware
measures how the base is tilted and turns the servo the opposite way. Mounting
the IMU on the platform closes a positive feedback loop and it will oscillate.

---

## Architecture

Three layers, each in its own directory, with dependencies pointing only
downward:

```
gimbal/          application + tasks   main.cpp, gimbal_tasks.{hpp,cpp}
   |
drivers/         device + platform     bmi160.hpp, servo.hpp,
   |                                   mspm0_{i2c,timer,adc,gpio,uart}.hpp
hal/             class interfaces      accelerometer, gyroscope, i2c, adc,
                                       timer (steady_clock + pwm), gpio
```

### Portable vs platform drivers

**Portable device drivers** — written only against the `hal/` interfaces, with
zero microcontroller register access, zero DriverLib calls, and zero RTOS
calls. Recompile them for another MCU and nothing in the file changes:

- `drivers/bmi160.hpp` — accelerometer **and** gyroscope, over `lab4::i2c`,
  using `lab2::steady_clock` for the datasheet start-up delays
- `drivers/servo.hpp` — angle in degrees → pulse width → duty cycle, over
  `lab2::pwm`

**Platform drivers** — implement the `hal/` interfaces against MSPM0G3507
hardware. Register access belongs here and only here:

| File | Implements | Hardware |
|---|---|---|
| `mspm0_i2c.hpp` | `lab4::i2c` | I2C1, PB2/PB3 (from Lab 4) |
| `mspm0_timer.hpp` | `lab2::steady_clock`, `lab2::pwm` | TIMG12 clock, TIMG8 PWM (from Lab 2) |
| `mspm0_adc.hpp` | `lab5::adc16_stream` | ADC0 ch2 + DMA (from Lab 5) |
| `mspm0_gpio.hpp` | `lab1::output_pin`, `lab1::input_pin` | GPIO (from Lab 1) |
| `mspm0_uart.hpp` | — (diagnostic only) | UART0 TX telemetry |

### Why the BMI160 exposes two objects instead of inheriting two interfaces

`lab4::accelerometer` and `lab4::gyroscope` each declare a private
`driver_read()` whose return type differs. C++ cannot overload on return type,
so one class cannot override both. The driver therefore holds two small
adapter members and hands them out:

```cpp
sensor::bmi160 imu(bus, clock);
lab4::accelerometer& accel = imu.acceleration();
lab4::gyroscope&     gyro  = imu.angular_velocity();
```

The tasks only ever see the interfaces, never `bmi160` itself.

---

## Tasks and communication

```
  sensor_task (100 Hz, prio 3)          input_task (50 Hz, prio 1)
  ----------------------------          --------------------------
  accelerometer.read()                  joystick.read(16 samples)
  gyroscope.read()                      average -> -1..+1
  accel angle = atan2(...)              deadzone + rescale
  complementary filter                  scale to degrees
          |                                      |
   tilt_measurement                      target_adjustment
          |                                      |
   angle_mailbox                          offset_mailbox
   queue, length 1                        queue, length 1
   xQueueOverwrite                        xQueueOverwrite
          |                                      |
          +----------> actuator_task <-----------+
                       (prio 2)
                       xQueueReceive(angle, block forever)
                       xQueuePeek(offset, no wait)
                       servo.angle(trim + offset - tilt)
                       xQueueOverwrite(status_mailbox)
                                |
                                v
                       telemetry_task (5 Hz, prio 1)
                       xQueuePeek(status) -> UART

  heartbeat_task (1 Hz, prio 1) -- toggles LED2 green
```

**Why length-1 mailboxes.** `xQueueOverwrite` replaces the pending value
instead of queueing behind it, so a producer never blocks and the actuator
always acts on the newest sample rather than draining a backlog of stale
angles. For a control loop, a late-but-fresh sample beats an on-time stale one.

**Why the actuator blocks on the angle mailbox.** It makes the sensor task the
clock for the whole control path: one new angle estimate, one servo update.
The joystick offset is *peeked* rather than received, so the latest value stays
available across cycles even though the input task runs at half the rate.

**No unprotected globals.** Every value that crosses a task boundary goes
through a FreeRTOS queue. The driver references the tasks share are written
once by `gimbal::start()` before the scheduler starts, are never written again,
and are handed to each task as its task parameter.

All tasks, stacks, and queues are **statically allocated** — the build cannot
fail to start for want of heap.

---

## Complementary filter

Run at 100 Hz in `sensor_task`:

```
angle = 0.98 * (angle + gyro_rate * dt) + 0.02 * accel_angle
```

- The gyroscope is accurate short-term but its integral drifts.
- The accelerometer has no drift but is noisy and is corrupted by any
  acceleration that is not gravity.
- The weighted blend takes the best of each.

Two details beyond the formula:

1. **`dt` is measured, not assumed.** `sensor_task` reads `steady_clock` each
   cycle, so if a cycle runs late the gyro integration is still correct.
2. **The filter is seeded from the accelerometer** at startup, so the estimate
   begins at the true angle instead of ramping to it from zero.

Roll uses `atan2(ay, az)` and gyro X. Pitch uses `atan2(-ax, hypot(ay, az))`
and gyro Y. Yaw is deliberately unsupported: the accelerometer cannot observe
rotation about the gravity vector, so a yaw estimate would drift without bound.

---

## Servo calibration (measured, not assumed)

The nominal hobby-servo mapping is 1000–2000 µs across 180° of travel. **The
servo on this build does not follow it**, and assuming it did made the platform
under-correct — it leaned the right way but never reached level.

Measured on hardware: rotating the IMU **90°** moved the arm only **45°**, a
clean 2× shortfall. So this servo sweeps roughly **90° per 1000 µs**, half the
nominal rate.

The fix is in `gimbal/main.cpp`: widen the pulse range symmetrically about the
1500 µs neutral point, which increases microseconds-per-commanded-degree
without shifting the zero position.

```
             span      µs/degree    travel at full command
1000-2000 us  1000 us    5.56         ~45 deg   (under-corrects 2x)
 600-2400 us  1800 us   10.00         ~81 deg   (correct in practice)
```

Centring on 1500 µs matters: an asymmetric range (say 500–2400) would move
neutral to 1450 µs and tilt the platform at rest.

> Widening the pulse range is only safe while the servo can physically reach
> the commanded positions. A servo being driven past its internal end stop
> **buzzes or hums and warms up**, draws 700 mA+ continuously, and can strip
> its gears. A healthy servo at the end of its travel is silent. If it buzzes,
> narrow the range back toward 1000/2000.

The alternative fix — keeping 1000–2000 µs and setting the angle limits to ±45
— gives exact 1:1 correction with no stall risk at all, at the cost of halving
the maximum correctable tilt.

---

## Build and run

The workspace holds two CCS projects: `freertos_kernel` (the RTOS, sources
linked from the SDK) and `gimbal` (the application). `gimbal` declares a
project dependency on `freertos_kernel`, so building `gimbal` builds both.

In CCS: open this folder as the workspace, then build the `gimbal` project.

Current footprint: **FLASH 19 %** (24.8 KB of 128 KB), **SRAM 49 %**
(16.2 KB of 32 KB).

### Telemetry

Open the XDS110 virtual COM port at **115200 8N1**. On boot:

```
=== CMPE 146 gimbal ===
BMI160 ready (accelerometer + gyroscope)
starting scheduler
angle=-1.24 offset=0.00 servo=1.24
```

`angle` is the filtered base tilt, `offset` the joystick contribution, `servo`
the commanded angle after trim and clamping.

> No `std::printf` anywhere in this project. On this toolchain it reaches the
> host through debugger semihosting, which **blocks forever when no debugger is
> attached** — which would break the standalone-boot requirement. Telemetry
> goes through a real UART peripheral instead, which is safe with nothing
> listening.

### Start-up failure signal

A **5 Hz blink** on LED2 green (clearly faster than the 1 Hz heartbeat) means
start-up failed — the IMU never answered after 5 attempts, or a task could not
be created. The reason is printed on the telemetry UART. A steady **1 Hz**
blink means the scheduler is running normally.

---

## Bring-up procedure

Do these in order; each step isolates one thing.

1. **I2C and the IMU.** Wire only the BMI160. Power up and watch telemetry.
   `BMI160 ready` means the chip ID read back `0xD1` and both sensors reached
   normal power mode. `BMI160 init FAILED` means check 3V3, GND, and that SDA
   and SCL are not swapped.
2. **Angle sign and magnitude.** Hold the board level — `angle` should sit near
   0. Tilt it 90°; `angle` should track smoothly to about ±90 and come back.
   If it reads the negative of the tilt, set `invert_servo` (step 4) rather
   than editing the filter.
3. **Servo, with the IMU unplugged from the equation.** Connect the servo
   signal to BP pin 17 and the AA pack, **ground bonded**. It should park at
   centre on boot. If it buzzes at the end stops, narrow
   `min/max_pulse_microseconds` in `actuator::servo::settings`.
4. **Closed loop.** Tilt the base. The platform should stay level. If the servo
   drives the *wrong* way and makes the tilt worse, set
   `tuning::invert_servo = true` in `main.cpp`.
5. **Trim.** If the platform holds level but a few degrees off, set
   `tuning::servo_trim_degrees`.
6. **Joystick.** Centre the stick: `offset` should read 0.00. Push it fully:
   `offset` should reach about ±30. If it creeps while centred, raise
   `tuning::joystick_deadzone`.
7. **Standalone boot.** Unplug the debugger. Power the LaunchPad from USB (or
   a phone charger) and the servo from the AA pack. It must come up and
   stabilize with no debugger and no reflash.
8. **Reliability.** Let it run 3 minutes while tilting, confirming the
   heartbeat never stops and the loop never needs a reset.

### Everything tunable in one place

`gimbal::tuning` in `gimbal/gimbal_tasks.hpp`, set from `main.cpp`:

| Field | Default | Purpose |
|---|---|---|
| `stabilized_axis` | `roll` | Which axis is held level |
| `filter_weight` | `0.98` | Gyro share of the complementary filter |
| `sensor_rate_hz` | `100` | Control loop rate |
| `input_rate_hz` | `50` | Joystick sample rate |
| `joystick_range_degrees` | `30` | Offset at full stick deflection |
| `joystick_deadzone` | `0.08` | Dead band either side of centre |
| `servo_trim_degrees` | `0` | Mechanical squaring-up |
| `invert_servo` | `false` | Flip servo direction |

---

## Logic analyzer capture

Probe **PB2 (SCL, BP pin 9)** and **PB3 (SDA, BP pin 10)**, ground to BP pin
20. The bus runs at **100 kHz** — the rate Lab 4 proved on this hardware.

`sensor_task` produces two transactions every 10 ms:

1. **Accelerometer read** — write `0x12` (`ACC_X_L`), then read 6 bytes
2. **Gyroscope read** — write `0x0C` (`GYR_X_L`), then read 6 bytes

Each is a write-then-read: start, address `0x68` + W, register byte, repeated
start, address `0x68` + R, six data bytes, stop. For the slide, point out the
7-bit address `0x68`, the R/W bit, the register pointer, the ACK after each
byte, and the 10 ms gap that confirms the 100 Hz sensor rate.

---

## Rubric coverage

| Requirement | Where |
|---|---|
| Sensor task, ~100 Hz, complementary filter | `gimbal_tasks.cpp` → `sensor_task` |
| Input task, joystick via ADC | `gimbal_tasks.cpp` → `input_task` |
| Actuator task, drives servo | `gimbal_tasks.cpp` → `actuator_task` |
| Heartbeat LED | `gimbal_tasks.cpp` → `heartbeat_task` |
| FreeRTOS primitives, no unprotected globals | three length-1 queues |
| BMI160 accelerometer **and** gyroscope | `drivers/bmi160.hpp` |
| Servo driver | `drivers/servo.hpp` |
| Written to class interfaces, no register access in drivers | `hal/` + the two portable drivers |
| Drivers / tasks / application in separate modules | `drivers/` · `gimbal_tasks.*` · `main.cpp` |
| Standalone boot | no semihosting anywhere |
