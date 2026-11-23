#pragma once

// Gimbal task set.
//
// Three tasks plus a heartbeat, all talking through FreeRTOS queues:
//
//        sensor_task (100 Hz)            input_task (50 Hz)
//        reads BMI160 accel+gyro         reads joystick ADC
//        complementary filter            deadzone + scaling
//              |                                |
//        tilt_measurement                 target_adjustment
//              |                                |
//        angle_mailbox                    offset_mailbox
//        (queue, length 1)                (queue, length 1)
//              |                                |
//              +--------> actuator_task <-------+
//                         blocks on angle_mailbox,
//                         peeks offset_mailbox,
//                         drives the servo
//
//        heartbeat_task (1 Hz) -- toggles an LED, proving the scheduler runs
//
// Both mailboxes are length-1 queues written with xQueueOverwrite, so a
// producer never blocks and the actuator always acts on the freshest sample
// rather than draining a backlog of stale ones. No application state is
// shared through mutable globals.

#include "../drivers/servo.hpp"
#include "../hal/accelerometer.hpp"
#include "../hal/adc.hpp"
#include "../hal/gpio.hpp"
#include "../hal/gyroscope.hpp"
#include "../hal/timer.hpp"

namespace gimbal {

/// Which axis the gimbal holds level. Yaw is deliberately absent: the
/// accelerometer cannot observe rotation about the gravity vector, so a yaw
/// estimate would drift without bound.
enum class axis
{
  /// Rotation about the sensor X axis, estimated from accel Y and Z.
  roll,
  /// Rotation about the sensor Y axis, estimated from accel X, Y and Z.
  pitch,
};

/// Everything that can be tuned at the bench without touching task logic.
struct tuning
{
  /// Axis to stabilize.
  axis stabilized_axis = axis::roll;

  /// Complementary filter weight given to the gyroscope-integrated angle.
  /// The accelerometer gets the remaining (1 - weight).
  float filter_weight = 0.98F;

  /// Sensor task rate. The complementary filter is designed around 100 Hz.
  float sensor_rate_hz = 100.0F;

  /// Input task rate.
  float input_rate_hz = 50.0F;

  /// Joystick travel at full deflection, in degrees of target offset.
  float joystick_range_degrees = 30.0F;

  /// Fraction of joystick travel either side of centre treated as zero, so a
  /// self-centring stick that does not rest exactly at mid-scale does not
  /// creep. Rescaled past the deadzone so there is no step at its edge.
  float joystick_deadzone = 0.08F;

  /// Added to every servo command. Use this to square up the platform when
  /// the servo horn cannot be remounted finely enough.
  float servo_trim_degrees = 0.0F;

  /// By default the servo is driven against the measured tilt, which is what
  /// keeps the platform level. Set true if the servo turns the wrong way
  /// because of how the horn or the IMU is mounted.
  bool invert_servo = false;
};

/// The drivers the tasks operate. All are class interfaces, so none of the
/// task code depends on the microcontroller.
struct hardware
{
  lab4::accelerometer& accelerometer;
  lab4::gyroscope& gyroscope;
  lab5::adc16_stream& joystick;
  actuator::servo& servo;
  lab1::output_pin& heartbeat_led;
  lab2::steady_clock& clock;
};

/// A snapshot of the control loop, published by actuator_task after every
/// servo update. Used for telemetry and bench tuning only - nothing in the
/// control path reads it.
struct status
{
  /// Filtered tilt of the base on the stabilized axis, in degrees.
  float angle_degrees;
  /// Operator offset currently applied, in degrees.
  float offset_degrees;
  /// Angle commanded to the servo, after trim and clamping, in degrees.
  float servo_degrees;
};

/**
 * @brief Read the most recent control-loop snapshot.
 *
 * Reads a length-1 queue, so it neither blocks nor disturbs the control
 * tasks, and needs no lock of its own.
 *
 * @param p_status - filled in with the latest snapshot
 * @return true - a snapshot was available
 * @return false - the control loop has not produced one yet
 */
bool latest_status(status& p_status);

/**
 * @brief Create the mailboxes and the four tasks.
 *
 * Everything is statically allocated, so this cannot fail for want of heap.
 * Call it before vTaskStartScheduler().
 *
 * @param p_hardware - drivers the tasks will use
 * @param p_tuning - tunable constants
 * @return true - every queue and task was created
 * @return false - a queue or task could not be created
 */
bool start(hardware const& p_hardware, tuning const& p_tuning = tuning{});

}  // namespace gimbal
