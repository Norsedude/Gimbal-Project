// Gimbal task set. See gimbal_tasks.hpp for the data-flow diagram.

#include "gimbal_tasks.hpp"

#include <array>
#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace gimbal {
namespace {

// ===========================================================================
// Messages passed between tasks
// ===========================================================================

/// sensor_task -> actuator_task
struct tilt_measurement
{
  /// Filtered tilt of the base on the stabilized axis, in degrees.
  float angle_degrees;
  /// Gyroscope bias the filter is correcting for, in degrees per second.
  float gyro_bias_dps;
};

/// input_task -> actuator_task
struct target_adjustment
{
  /// Offset the operator wants added to the held angle, in degrees.
  float offset_degrees;
};

// ===========================================================================
// Task context
// ===========================================================================

/// Drivers and tuning, as pointers so the struct can be statically created
/// before start() fills it in. Written once by start() before the scheduler
/// runs and only read afterwards, and passed to each task explicitly as its
/// task parameter rather than reached as a global.
struct task_context
{
  lab4::accelerometer* accelerometer;
  lab4::gyroscope* gyroscope;
  lab5::adc16_stream* joystick;
  actuator::servo* servo;
  lab1::output_pin* heartbeat_led;
  lab2::steady_clock* clock;
  tuning parameters;
};

task_context s_context{};

// ===========================================================================
// Static storage: queues and tasks
// ===========================================================================

constexpr std::uint32_t control_stack_words = 512;
constexpr std::uint32_t heartbeat_stack_words = 192;

// Length-1 mailboxes: the newest sample replaces the previous one.
constexpr UBaseType_t mailbox_length = 1;

StaticQueue_t s_angle_mailbox_control;
std::uint8_t s_angle_mailbox_storage[mailbox_length * sizeof(tilt_measurement)];
QueueHandle_t s_angle_mailbox = nullptr;

StaticQueue_t s_offset_mailbox_control;
std::uint8_t
  s_offset_mailbox_storage[mailbox_length * sizeof(target_adjustment)];
QueueHandle_t s_offset_mailbox = nullptr;

// Published by actuator_task for telemetry. Peeked, never received, so
// readers cannot steal it from one another.
StaticQueue_t s_status_mailbox_control;
std::uint8_t s_status_mailbox_storage[mailbox_length * sizeof(status)];
QueueHandle_t s_status_mailbox = nullptr;

StaticTask_t s_sensor_task_control;
StackType_t s_sensor_task_stack[control_stack_words];

StaticTask_t s_input_task_control;
StackType_t s_input_task_stack[control_stack_words];

StaticTask_t s_actuator_task_control;
StackType_t s_actuator_task_stack[control_stack_words];

StaticTask_t s_heartbeat_task_control;
StackType_t s_heartbeat_task_stack[heartbeat_stack_words];

// ===========================================================================
// Angle maths
// ===========================================================================

constexpr float pi = 3.14159265358979323846F;
constexpr float radians_to_degrees = 180.0F / pi;

/// Tilt of the gravity vector on the chosen axis, in degrees.
///
/// Roll is rotation about X, so it is seen in the Y/Z plane. Pitch is
/// rotation about Y, and uses the magnitude of the Y/Z pair as its reference
/// so that it stays correct while the board is also rolled.
float acceleration_angle(lab4::accelerometer::read_t const& p_acceleration,
                         axis p_axis)
{
  if (p_axis == axis::roll) {
    return std::atan2(p_acceleration.y, p_acceleration.z) * radians_to_degrees;
  }

  auto const reference = std::sqrt((p_acceleration.y * p_acceleration.y) +
                                   (p_acceleration.z * p_acceleration.z));
  return std::atan2(-p_acceleration.x, reference) * radians_to_degrees;
}

/// Angular rate about the chosen axis, in degrees per second.
float angular_rate(lab4::gyroscope::read_t const& p_angular_velocity,
                   axis p_axis)
{
  return (p_axis == axis::roll) ? p_angular_velocity.x : p_angular_velocity.y;
}

/// Calibration windows tried before settling for the steadiest one. Three
/// windows bound the start-up delay to 6 seconds.
constexpr int gyro_calibration_attempts = 3;

/**
 * @brief Measure the gyroscope zero-rate offset on the stabilized axis.
 *
 * A MEMS gyroscope at rest does not read exactly 0 - the BMI160 datasheet
 * allows a few degrees per second of offset. The complementary filter
 * integrates the rate, so an uncorrected offset becomes a steady angle error
 * that the 2 % accelerometer share has to keep fighting. Averaging the rate
 * while the gimbal is still and subtracting it removes that error at source.
 *
 * A window whose samples stay within a narrow spread was taken at rest and is
 * used straight away. Otherwise a few more windows are tried, and the
 * steadiest of them is used if it is no worse than hand tremor: tremor
 * oscillates around the true offset, so its average is still a good
 * estimate. If every window shows deliberate motion, the filter runs
 * uncalibrated rather than trusting a bias measured while being tilted.
 *
 * @return the offset in degrees per second, or 0 if it could not be measured
 */
float measure_gyro_bias(task_context const& p_context,
                        TickType_t p_period_ticks)
{
  auto const& parameters = p_context.parameters;
  auto const samples = parameters.gyro_calibration_samples;
  if (samples == 0) {
    return 0.0F;
  }

  float steadiest_spread = parameters.gyro_calibration_handheld_spread_dps;
  float steadiest_bias = 0.0F;

  for (int attempt = 0; attempt < gyro_calibration_attempts; attempt++) {
    float total = 0.0F;
    float lowest = 0.0F;
    float highest = 0.0F;
    auto wake_time = xTaskGetTickCount();

    for (std::uint32_t index = 0; index < samples; index++) {
      xTaskDelayUntil(&wake_time, p_period_ticks);
      auto const rate = angular_rate(p_context.gyroscope->read(),
                                     parameters.stabilized_axis);
      if (index == 0 || rate < lowest) {
        lowest = rate;
      }
      if (index == 0 || rate > highest) {
        highest = rate;
      }
      total += rate;
    }

    auto const spread = highest - lowest;
    auto const average = total / static_cast<float>(samples);
    if (spread <= parameters.gyro_calibration_max_spread_dps) {
      return average;  // at rest: as good as it gets
    }
    if (spread <= steadiest_spread) {
      steadiest_spread = spread;
      steadiest_bias = average;
    }
  }
  return steadiest_bias;
}

// ===========================================================================
// sensor_task - BMI160 accelerometer + gyroscope, complementary filter
// ===========================================================================

void sensor_task(void* p_parameters)
{
  auto const& context = *static_cast<task_context const*>(p_parameters);
  auto const& parameters = context.parameters;

  auto const nominal_period =
    1.0F / ((parameters.sensor_rate_hz > 0.0F) ? parameters.sensor_rate_hz
                                               : 100.0F);
  auto const period_ticks = pdMS_TO_TICKS(
    static_cast<std::uint32_t>((nominal_period * 1000.0F) + 0.5F));

  // Measure the gyroscope offset first, while the gimbal is still at
  // power-up. The actuator task holds the servo at centre until the first
  // angle is published below.
  auto const gyro_bias = measure_gyro_bias(context, period_ticks);

  // Seed the filter from the accelerometer so the estimate starts at the real
  // angle instead of walking up to it from zero over the first few seconds.
  auto angle = acceleration_angle(context.accelerometer->read(),
                                  parameters.stabilized_axis);

  auto last_uptime = context.clock->uptime();
  auto const clock_frequency = static_cast<float>(context.clock->frequency());
  auto wake_time = xTaskGetTickCount();

  while (true) {
    xTaskDelayUntil(&wake_time, period_ticks);

    // Measure the real elapsed time rather than assuming the nominal period,
    // so the gyroscope integration stays correct even if a cycle runs late.
    auto const now = context.clock->uptime();
    auto elapsed_seconds =
      static_cast<float>(now - last_uptime) / clock_frequency;
    last_uptime = now;
    if (elapsed_seconds <= 0.0F || elapsed_seconds > 0.5F) {
      elapsed_seconds = nominal_period;  // clock glitch or first pass
    }

    auto const acceleration = context.accelerometer->read();
    auto const angular_velocity = context.gyroscope->read();

    auto const measured_angle =
      acceleration_angle(acceleration, parameters.stabilized_axis);
    auto const measured_rate =
      angular_rate(angular_velocity, parameters.stabilized_axis) - gyro_bias;

    // Complementary filter: trust the gyroscope over short intervals, and let
    // the accelerometer slowly correct its drift.
    auto const weight = parameters.filter_weight;
    angle = (weight * (angle + (measured_rate * elapsed_seconds))) +
            ((1.0F - weight) * measured_angle);

    tilt_measurement const message{ angle, gyro_bias };
    (void) xQueueOverwrite(s_angle_mailbox, &message);
  }
}

// ===========================================================================
// input_task - joystick ADC -> target angle offset
// ===========================================================================

void input_task(void* p_parameters)
{
  auto const& context = *static_cast<task_context const*>(p_parameters);
  auto const& parameters = context.parameters;

  auto const period_ticks = pdMS_TO_TICKS(static_cast<std::uint32_t>(
    (1000.0F / ((parameters.input_rate_hz > 0.0F) ? parameters.input_rate_hz
                                                  : 50.0F)) +
    0.5F));

  // Averaging a short burst costs nothing extra - the ADC driver fills the
  // whole buffer in one DMA transfer - and it takes the jitter out of a
  // cheap potentiometer.
  std::array<std::uint16_t, 16> samples{};

  while (true) {
    context.joystick->read(samples);

    std::uint32_t total = 0;
    for (auto const sample : samples) {
      total += sample;
    }
    auto const average =
      static_cast<float>(total) / static_cast<float>(samples.size());

    // 0 .. 65535 becomes -1 .. +1, centred on the stick resting position.
    auto deflection = ((average / 65535.0F) - 0.5F) * 2.0F;

    auto const deadzone = parameters.joystick_deadzone;
    auto const magnitude = std::fabs(deflection);
    if (magnitude <= deadzone) {
      deflection = 0.0F;
    } else if (deadzone < 1.0F) {
      // Rescale what is left so the response still reaches full travel and
      // has no step at the edge of the deadzone.
      auto const scaled = (magnitude - deadzone) / (1.0F - deadzone);
      deflection = (deflection < 0.0F) ? -scaled : scaled;
    }

    target_adjustment const message{ deflection *
                                     parameters.joystick_range_degrees };
    (void) xQueueOverwrite(s_offset_mailbox, &message);

    vTaskDelay(period_ticks);
  }
}

// ===========================================================================
// actuator_task - combine tilt and offset, drive the servo
// ===========================================================================

void actuator_task(void* p_parameters)
{
  auto const& context = *static_cast<task_context const*>(p_parameters);
  auto const& parameters = context.parameters;

  tilt_measurement tilt{ 0.0F, 0.0F };
  target_adjustment adjustment{ 0.0F };

  while (true) {
    // Paced by the sensor task: every new angle estimate wakes this task.
    if (xQueueReceive(s_angle_mailbox, &tilt, portMAX_DELAY) != pdPASS) {
      continue;
    }

    // Peek, so the latest joystick offset stays available for the next cycle
    // even though the input task runs at half the sensor rate.
    (void) xQueuePeek(s_offset_mailbox, &adjustment, 0);

    // Turn the servo opposite to the way the base is tilted, which is what
    // holds the platform level.
    auto const correction = parameters.invert_servo ? tilt.angle_degrees
                                                    : -tilt.angle_degrees;

    context.servo->angle(parameters.servo_trim_degrees +
                         adjustment.offset_degrees + correction);

    // Publish what just happened for the telemetry task. servo->angle()
    // reports the value after clamping, which is what is worth watching.
    status const snapshot{
      tilt.angle_degrees,
      adjustment.offset_degrees,
      context.servo->angle(),
      tilt.gyro_bias_dps,
    };
    (void) xQueueOverwrite(s_status_mailbox, &snapshot);
  }
}

// ===========================================================================
// heartbeat_task - proof that the scheduler is alive
// ===========================================================================

void heartbeat_task(void* p_parameters)
{
  auto const& context = *static_cast<task_context const*>(p_parameters);

  auto const half_period = pdMS_TO_TICKS(500);
  bool illuminated = false;

  while (true) {
    illuminated = !illuminated;
    context.heartbeat_led->level(illuminated);
    vTaskDelay(half_period);
  }
}

}  // namespace

// ===========================================================================
// Telemetry accessor
// ===========================================================================

bool latest_status(status& p_status)
{
  if (s_status_mailbox == nullptr) {
    return false;
  }
  return xQueuePeek(s_status_mailbox, &p_status, 0) == pdPASS;
}

// ===========================================================================
// start
// ===========================================================================

bool start(hardware const& p_hardware, tuning const& p_tuning)
{
  s_context = task_context{
    .accelerometer = &p_hardware.accelerometer,
    .gyroscope = &p_hardware.gyroscope,
    .joystick = &p_hardware.joystick,
    .servo = &p_hardware.servo,
    .heartbeat_led = &p_hardware.heartbeat_led,
    .clock = &p_hardware.clock,
    .parameters = p_tuning,
  };

  s_angle_mailbox = xQueueCreateStatic(mailbox_length,
                                       sizeof(tilt_measurement),
                                       s_angle_mailbox_storage,
                                       &s_angle_mailbox_control);
  s_offset_mailbox = xQueueCreateStatic(mailbox_length,
                                        sizeof(target_adjustment),
                                        s_offset_mailbox_storage,
                                        &s_offset_mailbox_control);
  s_status_mailbox = xQueueCreateStatic(mailbox_length,
                                        sizeof(status),
                                        s_status_mailbox_storage,
                                        &s_status_mailbox_control);
  if (s_angle_mailbox == nullptr || s_offset_mailbox == nullptr ||
      s_status_mailbox == nullptr) {
    return false;
  }

  // Seed the offset mailbox so the actuator task has something to peek at
  // before the input task has produced its first sample.
  target_adjustment const centred{ 0.0F };
  (void) xQueueOverwrite(s_offset_mailbox, &centred);

  // sensor_task is highest so its 100 Hz cadence sets the control rate;
  // actuator_task runs immediately after each new sample; input_task and the
  // heartbeat fill in the time that is left.
  auto const sensor_handle = xTaskCreateStatic(sensor_task,
                                               "sensor_task",
                                               control_stack_words,
                                               &s_context,
                                               tskIDLE_PRIORITY + 3,
                                               s_sensor_task_stack,
                                               &s_sensor_task_control);
  auto const actuator_handle = xTaskCreateStatic(actuator_task,
                                                 "actuator_task",
                                                 control_stack_words,
                                                 &s_context,
                                                 tskIDLE_PRIORITY + 2,
                                                 s_actuator_task_stack,
                                                 &s_actuator_task_control);
  auto const input_handle = xTaskCreateStatic(input_task,
                                              "input_task",
                                              control_stack_words,
                                              &s_context,
                                              tskIDLE_PRIORITY + 1,
                                              s_input_task_stack,
                                              &s_input_task_control);
  auto const heartbeat_handle = xTaskCreateStatic(heartbeat_task,
                                                  "heartbeat_task",
                                                  heartbeat_stack_words,
                                                  &s_context,
                                                  tskIDLE_PRIORITY + 1,
                                                  s_heartbeat_task_stack,
                                                  &s_heartbeat_task_control);

  return sensor_handle != nullptr && actuator_handle != nullptr &&
         input_handle != nullptr && heartbeat_handle != nullptr;
}

}  // namespace gimbal
