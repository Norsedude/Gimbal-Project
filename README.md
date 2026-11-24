# One-Axis Stabilizing Gimbal — CMPE 146 Final Project

Keeps a small platform level on one axis while the base is tilted by hand, and
lets a joystick shift the angle it holds. Runs on an LP-MSPM0G3507 LaunchPad
under FreeRTOS.

Target device: **MSPM0G3507** (Cortex-M0+, 32 MHz) · SDK **MSPM0-SDK 2.11.0.07**
· compiler **TI Arm Clang 5.1.1.LTS**

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

Three details beyond the formula:

1. **Gyroscope bias is calibrated at start-up.** For the first 2 seconds
   `sensor_task` averages 200 gyro readings while the gimbal sits still and
   subtracts that offset from every later reading. A window is only accepted
   if the readings stay within 3 deg/s of each other; if the gimbal was moving
   it is retried, up to 5 times, then skipped rather than trusting a bias
   measured during motion.
2. **`dt` is measured, not assumed.** `sensor_task` reads `steady_clock` each
   cycle, so if a cycle runs late the gyro integration is still correct.
3. **The filter is seeded from the accelerometer** at startup, so the estimate
   begins at the true angle instead of ramping to it from zero.

Roll uses `atan2(ay, az)` and gyro X. Pitch uses `atan2(-ax, hypot(ay, az))`
and gyro Y. Yaw is deliberately unsupported: the accelerometer cannot observe
rotation about the gravity vector, so a yaw estimate would drift without bound.

---

## Build and run

The workspace holds two CCS projects: `freertos_kernel` (the RTOS, sources
linked from the SDK) and `gimbal` (the application). `gimbal` declares a
project dependency on `freertos_kernel`, so building `gimbal` builds both.

In CCS: open this folder as the workspace, then build the `gimbal` project.

Current footprint: **FLASH 19 %** (24.7 KB of 128 KB), **SRAM 46 %**
(14.6 KB of 32 KB).

The project compiles with `-fno-threadsafe-statics`. The C++ guard that would
otherwise protect each function-local `static` in `main()` has an error path
that prints through `vfprintf`, which drags the whole semihosting host-I/O
library into the image. Nothing runs concurrently before the scheduler starts,
so the guard protects nothing; dropping it removes `printf`, file I/O, and
`malloc` from the build entirely.

### Telemetry

Open the XDS110 virtual COM port at **115200 8N1**. On boot:

```
=== CMPE 146 gimbal ===
BMI160 ready (accelerometer + gyroscope)
starting scheduler
angle=-1.24 offset=0.00 servo=1.24 bias=-0.37
```

Nothing prints for the first ~2 seconds after `starting scheduler`: that is the
gyroscope calibration (keep the gimbal still). `angle` is the filtered base
tilt, `offset` the joystick contribution, `servo` the commanded angle after
trim and clamping, and `bias` the gyroscope offset measured at start-up and
subtracted from every reading (exactly `0.00` means the gimbal moved during
calibration and it was skipped - power-cycle and hold it still).

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
| `gyro_calibration_samples` | `200` | Start-up gyro bias average (2 s); 0 disables |
| `gyro_calibration_max_spread_dps` | `3.0` | Max motion allowed during calibration |
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

Each read is **two separate I2C transfers**, each with its own START and STOP:

```
START | 0x68 + W | ACK | register | ACK | STOP
START | 0x68 + R | ACK | d0 ACK d1 ACK ... d5 NACK | STOP
```

The first transfer sets the BMI160's register pointer; the second reads six
bytes starting there (X low, X high, Y low, Y high, Z low, Z high). The sensor
keeps the pointer across the STOP, so this reads the same data a repeated-start
transfer would. On the capture you will see a **STOP between the register byte
and the read** - that is expected, not a fault.

For the slide, point out: the 7-bit address `0x68` and the R/W bit after it
(`0xD0` write, `0xD1` read on the wire), the register pointer byte, the ACK
after every byte, the controller's NACK on the last data byte (telling the
sensor to stop sending), and the **10 ms gap between bursts** that confirms
the 100 Hz sensor rate.

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
