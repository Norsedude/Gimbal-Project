# What To Do Next

Firmware is written and builds clean. **Nothing has been tested on hardware yet.**
This is the order to do that in. Each step isolates one thing, so when something
misbehaves you know exactly what caused it.

Demo is **Thursday, December 3**.

---

## Step 0 — Move the repo off iCloud Drive (do this first, 5 minutes)

During development iCloud renamed `gimbal/main.cpp` to `gimbal/main 2.cpp` in the
middle of a build. It will do it again. Cloud sync also locks files and syncs
half-written build output.

```
git clone <this repo> C:\dev\Gimbal-Project
```

Then open `C:\dev\Gimbal-Project` as your CCS workspace and work there. Git keeps
all your history; iCloud is adding nothing but risk 11 days before a graded demo.

If you hit a weird build error later, **check for `* 2.*` files before debugging
anything else**:

```
dir /s /b "* 2.*"
```

---

## Step 1 — Identify your parts (10 minutes)

Look at the actual hardware and confirm three things:

1. **Servo model** — read the label. `SG90` (plastic gear) or `MG90S` (metal gear).
   Either is fine, both are 50 Hz. If it says something else, tell me the pulse
   range from its datasheet.
2. **BMI160 breakout** — confirm it has a `3V3` or `VCC` pin and a `GND` pin.
   **Do not connect it to 5 V.** Most of these breakouts are 3.3 V only and 5 V
   will damage them. Also note whether `SDO`/`SA0` is tied low (address `0x68`,
   what the firmware expects) — your Lab 4 worked at `0x68`, so it almost
   certainly is.
3. **Batteries** — 4×AA. Alkaline gives 6.0–6.4 V fresh, which is at or slightly
   over an SG90's 6 V ceiling. NiMH (4.8 V) is gentler. Alkaline usually works;
   if the servo buzzes or resets the board, this is the first suspect.

---

## Step 2 — Mechanical assembly

### 2a. Orient the pan-tilt bracket on its side

As sold, the **bottom servo pans** — it rotates around the vertical axis (yaw).
An accelerometer cannot measure yaw, because rotating around gravity doesn't
change what gravity looks like. A yaw estimate would drift forever.

**Mount the whole assembly on its side** so the servo you drive **tilts** the
platform up and down. You only need one servo. Leave the second one unplugged.

### 2b. Centre the servo *before* bolting on the horn

This matters and it's easy to get wrong.

1. Wire up the servo (Step 3c) and power the board **first**.
2. On boot the firmware parks the servo at its centre (1500 µs) before anything
   else happens.
3. **Now** attach the horn and platform so the platform sits **level** while the
   servo is at that centre position.

If you bolt the horn on first and then power up, the servo jumps to centre and
your platform starts off crooked, costing you half your travel in one direction.
Small residual error is fine — that's what `servo_trim_degrees` is for.

### 2c. Mount the IMU on the BASE, not the platform

Put the BMI160 on the base or the handle — the part **you** tilt. Not on the
moving platform.

The firmware measures how the base is tilted and turns the servo the opposite
way. If the IMU rides on the platform, the servo's own correction changes what
the IMU reads, which closes a positive feedback loop and it will oscillate.

Mount it flat and firmly (double-sided tape is fine). A loose IMU adds vibration
noise straight into the accelerometer.

### 2d. Keep the load light

These are 9 g micro servos. A small piece of cardboard or a plastic plate is
fine. They cannot hold a phone.

---

## Step 3 — Electrical assembly

Build this up **one block at a time** and power down between changes.

### Full connection table

| From | To | BoosterPack pin |
|---|---|---|
| BMI160 `VCC` | LaunchPad `3V3` | pin 1 |
| BMI160 `GND` | LaunchPad `GND` | pin 20 |
| BMI160 `SCL` | `PB2` | pin 9 |
| BMI160 `SDA` | `PB3` | pin 10 |
| Joystick `+5V` (yes, to 3.3 V) | LaunchPad `3V3` | pin 1 |
| Joystick `GND` | LaunchPad `GND` | pin 20 |
| Joystick `VRx` | `PA25` | pin 2 |
| Servo signal (orange/yellow) | `PB15` | pin 17 |
| Servo `V+` (red) | **battery +** | — |
| Servo `GND` (brown/black) | **battery −** | — |
| **battery −** | **LaunchPad `GND`** | pin 20 or 22 |

### 3a. IMU only

Wire the four BMI160 lines. Nothing else yet. Double-check `SDA` and `SCL` are
not swapped — that is the single most common wiring mistake here.

### 3b. Joystick

Three wires. The module is silkscreened `+5V` but it is just two 10 kΩ
potentiometers — **power it from 3.3 V**. That keeps its output inside the ADC's
VDDA reference so full stick travel maps to full ADC range. On 5 V you'd clip
and could stress the pin.

Only `VRx` is used. `VRy` and `SW` stay unconnected.

### 3c. Servo and battery

> ⚠️ **The servo's red wire goes to the battery, never to a LaunchPad pin.**
> A micro servo can pull ~700 mA stalled. The LaunchPad's 3V3 regulator cannot
> supply that and you will brown out the MCU mid-demo.

Three connections:
- Servo signal → BP pin 17
- Servo red → battery +
- Servo brown → battery −
- **And battery − → LaunchPad GND**

That last wire is not optional. PWM is a voltage referenced to ground. Without a
shared ground the servo has no reference for the signal and will twitch randomly
or ignore it entirely. If your servo behaves erratically, check this wire first.

### Before first power-on

With the battery **disconnected**, check with a multimeter:
- No short between `3V3` and `GND`
- No short between battery + and battery −
- Continuity from battery − to LaunchPad `GND`

---

## Step 4 — Build and flash

1. Open `C:\dev\Gimbal-Project` as the CCS workspace.
2. Build the **`gimbal`** project (it builds `freertos_kernel` automatically).
   It should build with no compiler warnings (CCS may note that it substituted
   SDK 2.11.0.07 / SysConfig 1.28.0 for the older versions the project names -
   that is harmless).
3. Flash and run.
4. Open the **XDS110 virtual COM port at 115200 8N1**.

Expected on boot:

```
=== CMPE 146 gimbal ===
BMI160 ready (accelerometer + gyroscope)
starting scheduler
angle=-0.42 offset=0.00 servo=0.42 bias=-0.37
```

**Keep the gimbal still for the first 2 seconds after power-up** - that is the
gyroscope bias calibration, and telemetry starts once it finishes. If `bias`
reads exactly `0.00`, it moved during calibration; power-cycle and hold still.

### LED meaning

| LED2 green | Meaning |
|---|---|
| **1 Hz** steady blink | Normal — scheduler running |
| **5 Hz** fast blink | Start-up failed. Read the serial output for why |
| Off / solid | Never reached `main`, or power problem |

---

## Step 5 — Bring-up tests, in this order

### Test 1 — Heartbeat and telemetry

Power on with only USB. You should get the banner and a 1 Hz blink.

*If nothing on serial:* confirm you picked the XDS110 COM port (not another
device) and 115200 8N1.

### Test 2 — IMU responds

Look for `BMI160 ready`. That line means the chip answered, returned chip ID
`0xD1`, and reported **both** accelerometer and gyroscope in normal power mode.

*If you get `BMI160 init FAILED`:* power (3.3 V not 5 V), ground, and `SDA`/`SCL`
not swapped. The firmware already retries 5 times before giving up.

### Test 3 — Angle tracks correctly

Watch the `angle=` field while you tilt the base by hand.

- Level → near `0.00`
- Tilt ~90° one way → tracks smoothly toward `90`
- Tilt back → returns to `0`
- Hold still → should be steady, not drifting or jittering

*If angle drifts while held still:* check `bias` - if it is `0.00`, calibration
was skipped because the board moved; power-cycle and keep it still for 2 s. If
`bias` is non-zero and it still drifts, the IMU is probably loose on its mount.

*If it reads the negative of the tilt:* don't edit the filter — that's what
`invert_servo` is for in Test 5.

*If the wrong motion changes the angle* (you tilt one way and nothing happens,
but the other axis responds): your IMU is mounted rotated. Either rotate it 90°,
or change `stabilized_axis` from `roll` to `pitch` in `main.cpp`.

### Test 4 — Servo responds

Connect the servo and battery. On boot it should snap to centre and hold.

*If it buzzes or grinds at the extremes:* narrow the pulse range in
`actuator::servo::settings` (`drivers/servo.hpp`) — try 1100–1900 µs.

*If it twitches randomly:* the shared ground wire is missing or loose.

### Test 5 — Closed loop (the 25-point test)

Tilt the base. **The platform should stay level.**

*If the servo drives the wrong way and makes the tilt worse* — this is the most
likely first result, and it's a one-line fix. In `gimbal/main.cpp`:

```cpp
gimbal::tuning const parameters{ .invert_servo = true };
```

*If it oscillates / shakes:* check the IMU is on the base and not the platform
(Step 2c). If it's correct and still oscillates, lower `filter_weight` to `0.95`.

*If it holds level but a few degrees off:* set `servo_trim_degrees` to the offset.

### Test 6 — Joystick (10 points)

- Stick centred → `offset=0.00`
- Push fully → `offset` reaches about `±30`
- The platform should shift its held angle in real time as you move the stick

*If `offset` creeps while centred:* raise `joystick_deadzone` (try `0.12`).
*If it feels laggy:* tell me — the input task is at 50 Hz and can go faster.
*If it moves backwards:* negate `joystick_range_degrees` (use `-30.0F`).

### Test 7 — Standalone boot (5 points)

**Unplug the debugger entirely.** Power the LaunchPad from a USB charger or
power bank, servo from the AA pack. It must come up and stabilize with no
debugger and no reflash.

This should already work — the firmware deliberately contains no `printf`,
because on this toolchain `printf` goes through debugger semihosting and blocks
forever when no debugger is attached.

### Test 8 — Reliability (5 points)

Run it for a full 3 minutes while tilting the base around. The heartbeat must
not stop and it must not need a reset.

**Fault-recovery test (worth showing the TA):** while it runs, pull the IMU's
SDA jumper for a couple of seconds, then push it back in. Expected:
- While unplugged: the platform freezes at its last angle (no runaway drift),
  and the heartbeat keeps blinking.
- After reconnecting: within about half a second it stabilizes again, with no
  reset.

How that works: every I2C wait is bounded (~5 ms), so a missing sensor makes a
read fail instead of hanging. After a failure the driver bit-bangs nine SCL
pulses plus a STOP to free a stuck bus and resets the I2C peripheral. After
20 failed reads in a row (~100 ms) the BMI160 driver re-runs its start-up
sequence. A failed gyro read reports zero rotation, so the angle cannot drift
during an outage.

---

## Step 6 — Tuning reference

Everything lives in `gimbal::tuning`, set from `gimbal/main.cpp`:

| Field | Default | Change it when |
|---|---|---|
| `stabilized_axis` | `roll` | Wrong axis responds to your tilt |
| `gyro_calibration_max_spread_dps` | `3.0` | `bias` keeps reading 0.00 even when held still → raise to 5.0 |
| `filter_weight` | `0.98` | Lower (0.95) if it oscillates; raise (0.99) if noisy |
| `joystick_range_degrees` | `30` | Want more/less joystick authority; negate to reverse |
| `joystick_deadzone` | `0.08` | Stick creeps at centre → raise it |
| `servo_trim_degrees` | `0` | Platform holds level but offset by a few degrees |
| `invert_servo` | `false` | **Servo fights the tilt instead of correcting it** |

---

## Step 7 — Logic analyzer capture (4 slide points)

Probe `PB2` (SCL, pin 9) and `PB3` (SDA, pin 10), ground to pin 20. Bus runs at
**100 kHz**.

Every 10 ms the sensor task reads both sensors. Each read is two separate
I2C transfers, each with its own START and STOP:
1. `START, 0x68+W, 0x12 (ACC_X_L), STOP` then `START, 0x68+R, 6 bytes, STOP` - accelerometer
2. `START, 0x68+W, 0x0C (GYR_X_L), STOP` then `START, 0x68+R, 6 bytes, STOP` - gyroscope

There is a STOP between the register byte and the read - that is how this
driver works, not a fault. The BMI160 keeps its register pointer across it.

For the slide, label: the 7-bit address `0x68` and the R/W bit, the register
pointer byte, the ACK after each byte, the NACK on the last data byte, and the
**10 ms gap between bursts that proves the 100 Hz rate**.

---

## Step 8 — Slides (25 points)

The `README.md` has source material for all six required slides:

| Slide | Points | Where to get it |
|---|---|---|
| Overview | 4 | README intro |
| Wiring diagram + parts list | 5 | README "Wiring" — redraw it cleanly, show **separate servo power and shared ground** |
| Tasks | 4 | README task diagram — names must match the code exactly |
| Task communication | 4 | The three length-1 mailboxes and why `xQueueOverwrite` |
| Drivers | 4 | BMI160, servo, joystick — say which are **portable** vs platform |
| Logic analyzer capture | 4 | Step 7 — must be **explained**, not just pasted |

Be ready to defend, since the TA will ask:
- **Why two objects instead of inheriting both sensor interfaces?** Because
  `lab4::accelerometer` and `lab4::gyroscope` each declare a private
  `driver_read()` differing only in return type, and C++ can't overload on
  return type.
- **Why length-1 queues with `xQueueOverwrite`?** A control loop wants the
  freshest sample, not a backlog of stale ones. Producers never block.
- **Why is the IMU on the base?** Avoids the positive feedback loop.
- **Why no `printf`?** Semihosting blocks with no debugger attached, which would
  break standalone boot.
- **Point at any register line and find it in the TRM.** Each driver names its
  TRM chapter at the top.

---

## Suggested schedule

| When | What |
|---|---|
| Now | Step 0 (move off iCloud), Step 1 (identify parts) |
| Next session | Steps 2–3 assembly, Step 4 flash, Tests 1–2 |
| Following | Tests 3–6, tuning |
| ~Dec 1 | Tests 7–8, logic analyzer capture |
| Dec 2 | Slides, full dress rehearsal, record backup video |
| **Dec 3** | **Demo** |

Record the backup video once it works. A video-only demo earns half credit on
stabilization and joystick, so it's a floor, not a plan.

---

## When you're ready

Plug in the LaunchPad and tell me where you are. I can flash it, watch the
telemetry over the serial MCP, and work through the tuning with you.
