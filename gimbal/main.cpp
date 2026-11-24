// CMPE 146 final project: one-axis stabilizing gimbal.
//
// This file is the application layer. It owns no control logic - it only
// constructs the platform drivers, wires them to the portable device drivers,
// hands the resulting class interfaces to the task set, and starts the
// scheduler.
//
// Layering:
//   hal/      - class interfaces (accelerometer, gyroscope, i2c, adc, pwm,
//               steady_clock, output_pin)
//   drivers/  - mspm0_* platform drivers implement those interfaces against
//               MSPM0G3507 hardware; bmi160 and servo are portable device
//               drivers written only against the interfaces
//   gimbal/   - this application plus the task set in gimbal_tasks.cpp
//
// Wiring (LP-MSPM0G3507):
//   BMI160 SCL   -> PB2  (BoosterPack pin 9)
//   BMI160 SDA   -> PB3  (BoosterPack pin 10)
//   BMI160 VCC   -> 3V3  (BoosterPack pin 1)   -- 3.3 V only
//   Servo signal -> PB15 (BoosterPack pin 17)
//   Joystick VRx -> PA25 (BoosterPack pin 2)
//   Joystick VCC -> 3V3  (BoosterPack pin 1)   -- not 5 V, keeps ADC in range
//   Heartbeat    -> PB27 (on-board LED2 green, active HIGH)
//   Servo power  -> 4xAA pack; pack ground tied to LaunchPad GND
//
// Deliberately no std::printf anywhere: on this toolchain it reaches the host
// through debugger semihosting, which blocks forever when no debugger is
// attached. The project has to boot and run standalone.

#include <cstddef>

#include "FreeRTOS.h"
#include "task.h"

#include "../drivers/bmi160.hpp"
#include "../drivers/mspm0_adc.hpp"
#include "../drivers/mspm0_gpio.hpp"
#include "../drivers/mspm0_i2c.hpp"
#include "../drivers/mspm0_timer.hpp"
#include "../drivers/mspm0_uart.hpp"
#include "../drivers/servo.hpp"
#include "gimbal_tasks.hpp"

namespace {

/// Standard hobby-servo frame rate.
constexpr std::uint32_t servo_frequency_hz = 50;

/// On-board LED2 green: PB27, IOMUX slot 58, active HIGH.
constexpr unsigned heartbeat_pin = 27;
constexpr unsigned heartbeat_slot = 58;

/**
 * @brief Blink fast forever to signal a start-up failure.
 *
 * Reached only if the IMU never answers or a task could not be created. A
 * 5 Hz blink is clearly different from the 1 Hz heartbeat, so a glance at the
 * board tells you the firmware never got as far as running the control loop.
 */
[[noreturn]] void halt_blinking(lab1::output_pin& p_led,
                                lab2::steady_clock& p_clock)
{
  bool illuminated = false;
  while (true) {
    illuminated = !illuminated;
    p_led.level(illuminated);
    mspm0::delay_ms(p_clock, 100);
  }
}

// --- Telemetry --------------------------------------------------------------
// Lowest-priority task that prints the control-loop snapshot to the debug
// probe virtual COM port at 5 Hz. Diagnostic only: it reads the status
// mailbox and touches nothing the control tasks rely on. This is how the
// filter sign, joystick travel, and servo command get checked on the bench.

constexpr std::uint32_t telemetry_stack_words = 384;
constexpr std::uint32_t telemetry_period_ms = 200;

StaticTask_t s_telemetry_task_control;
StackType_t s_telemetry_task_stack[telemetry_stack_words];

void telemetry_task(void* p_parameters)
{
  auto& uart = *static_cast<mspm0::telemetry_uart*>(p_parameters);

  while (true) {
    gimbal::status snapshot{};
    if (gimbal::latest_status(snapshot)) {
      char line[96];
      std::size_t at = 0;
      mspm0::append_text(line, sizeof(line), at, "angle=");
      mspm0::append_float(line, sizeof(line), at, snapshot.angle_degrees);
      mspm0::append_text(line, sizeof(line), at, " offset=");
      mspm0::append_float(line, sizeof(line), at, snapshot.offset_degrees);
      mspm0::append_text(line, sizeof(line), at, " servo=");
      mspm0::append_float(line, sizeof(line), at, snapshot.servo_degrees);
      mspm0::append_text(line, sizeof(line), at, " bias=");
      mspm0::append_float(line, sizeof(line), at, snapshot.gyro_bias_dps);
      mspm0::append_text(line, sizeof(line), at, "\r\n");
      uart.write(line);
    }

    vTaskDelay(pdMS_TO_TICKS(telemetry_period_ms));
  }
}

}  // namespace

/// ADC0 completion interrupt, routed to the ADC driver that owns it.
extern "C" void ADC0_IRQHandler()
{
  mspm0::adc::interrupt();
}

int main()
{
  // Static storage, not locals: once vTaskStartScheduler() runs, the stack
  // main() was using is handed out to tasks, so anything the tasks keep
  // referencing has to outlive this frame.
  // Telemetry first, so there is a way to see what happened if any later
  // constructor misbehaves.
  static mspm0::telemetry_uart telemetry;
  telemetry.write("\r\n=== CMPE 146 gimbal ===\r\n");

  static mspm0::steady_clock clock;
  static mspm0::i2c sensor_bus;
  static mspm0::pwm servo_pwm(mspm0::pwm::pb15_timer_g8_channel_0,
                              servo_frequency_hz);
  static mspm0::adc joystick;
  static mspm0::output_pin heartbeat_led(
    mspm0::gpiob, heartbeat_pin, heartbeat_slot);

  static sensor::bmi160 imu(sensor_bus, clock);
  // SG90/MG90S class: about 500-2500 us spans -90..+90 degrees. The generic
  // 1000-2000 us range only gives about +-45 degrees of real travel, which
  // made the platform under-correct by roughly half. Narrow these by ~50 us
  // per side if the servo buzzes or strains at a hard tilt.
  static actuator::servo tilt_servo(servo_pwm,
                                    actuator::servo::settings{
                                      .min_angle = -90.0F,
                                      .max_angle = 90.0F,
                                      .min_pulse_microseconds = 500,
                                      .max_pulse_microseconds = 2500,
                                    });

  // Every driver above configured its hardware in its own constructor, so by
  // this point the only thing left to check is whether the external sensor
  // answered.
  if (!imu.initialized()) {
    telemetry.write("BMI160 init FAILED - check 3V3, GND, SDA/SCL\r\n");
    halt_blinking(heartbeat_led, clock);
  }
  telemetry.write("BMI160 ready (accelerometer + gyroscope)\r\n");

  gimbal::hardware const devices{
    .accelerometer = imu.acceleration(),
    .gyroscope = imu.angular_velocity(),
    .joystick = joystick,
    .servo = tilt_servo,
    .heartbeat_led = heartbeat_led,
    .clock = clock,
  };

  gimbal::tuning const parameters{};

  if (!gimbal::start(devices, parameters)) {
    telemetry.write("task creation FAILED\r\n");
    halt_blinking(heartbeat_led, clock);
  }

  if (xTaskCreateStatic(telemetry_task,
                        "telemetry_task",
                        telemetry_stack_words,
                        &telemetry,
                        tskIDLE_PRIORITY + 1,
                        s_telemetry_task_stack,
                        &s_telemetry_task_control) == nullptr) {
    telemetry.write("telemetry task creation FAILED\r\n");
    halt_blinking(heartbeat_led, clock);
  }

  telemetry.write("starting scheduler\r\n");
  vTaskStartScheduler();

  // Only reached if the scheduler itself fails to start. The idle and timer
  // task memory callbacks that configSUPPORT_STATIC_ALLOCATION requires are
  // supplied by the freertos_kernel project (dpl/StaticAllocs_freertos.c), so
  // they are deliberately not defined here.
  while (true) {
  }
}
