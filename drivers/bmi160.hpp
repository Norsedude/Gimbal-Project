#pragma once

// Bosch BMI160 6-axis IMU driver.
//
// This is a portable device driver: it is written entirely against the class
// interfaces (lab4::i2c for the bus and lab2::steady_clock for start-up
// timing) and contains no microcontroller register access, no DriverLib
// calls, and no RTOS calls. Moving it to another microcontroller means
// supplying different implementations of those two interfaces - nothing in
// this file changes.
//
// The BMI160 registers named below are the sensor's own registers, reached
// over I2C through the bus interface. Register numbers and timings come from
// the BMI160 datasheet (BST-BMI160-DS000), section 2.11 unless noted.
//
// The chip senses two different physical quantities, so it exposes one
// interface object per quantity: acceleration() returns a lab4::accelerometer
// and angular_velocity() returns a lab4::gyroscope. They cannot be inherited
// by this one class because lab4::accelerometer and lab4::gyroscope each
// declare a private driver_read() whose return type differs, and C++ cannot
// overload on return type alone.

#include <array>
#include <cstdint>

#include "../hal/accelerometer.hpp"
#include "../hal/gyroscope.hpp"
#include "../hal/i2c.hpp"
#include "../hal/timer.hpp"
#include "../third_party/span/span.hpp"

namespace sensor {

/**
 * @brief Driver for the Bosch BMI160 accelerometer + gyroscope.
 *
 * The constructor performs the complete sensor start-up - soft reset, chip ID
 * check, both sensors into normal power mode, ranges and output data rate -
 * retrying a few times in case the sensor is still settling after power-on.
 * There is no second initialization step; ask initialized() whether it
 * worked.
 */
class bmi160
{
public:
  /// Accelerometer full-scale range. The enumeration value is the value
  /// written to the ACC_RANGE register (datasheet 2.11.6).
  enum class acceleration_range : std::uint8_t
  {
    plus_minus_2g = 0x03,
    plus_minus_4g = 0x05,
    plus_minus_8g = 0x08,
    plus_minus_16g = 0x0C,
  };

  /// Gyroscope full-scale range. The enumeration value is the value written
  /// to the GYR_RANGE register (datasheet 2.11.8).
  enum class angular_velocity_range : std::uint8_t
  {
    plus_minus_2000dps = 0x00,
    plus_minus_1000dps = 0x01,
    plus_minus_500dps = 0x02,
    plus_minus_250dps = 0x03,
    plus_minus_125dps = 0x04,
  };

  struct settings
  {
    /// 7-bit I2C address. 0x68 when the sensor SDO pin is tied low, 0x69
    /// when it is tied high.
    std::uint8_t address = 0x68;

    /// +-2g keeps the best resolution for tilt sensing, where the only
    /// acceleration of interest is the 1 g gravity vector.
    acceleration_range acceleration = acceleration_range::plus_minus_2g;

    /// +-500 dps is a good compromise for a hand-tilted gimbal: finer
    /// resolution than the +-2000 dps default, with enough headroom for a
    /// brisk wrist movement.
    angular_velocity_range angular_velocity =
      angular_velocity_range::plus_minus_500dps;
  };

  /**
   * @param p_bus - I2C bus the sensor is attached to
   * @param p_clock - steady clock used for the datasheet start-up delays
   * @param p_settings - address and full-scale ranges
   */
  bmi160(lab4::i2c& p_bus,
         lab2::steady_clock& p_clock,
         settings const& p_settings)
    : m_bus(p_bus)
    , m_clock(p_clock)
    , m_settings(p_settings)
    , m_acceleration_scale(acceleration_full_scale(p_settings.acceleration) /
                           32768.0F)
    , m_angular_velocity_scale(
        angular_velocity_full_scale(p_settings.angular_velocity) / 32768.0F)
  {
    m_initialized = configure();
  }

  /**
   * @brief Construct with the default address and ranges.
   *
   * Defined after the class body because a `= {}` default argument cannot
   * see the default member initializers of the nested `settings` struct
   * while the enclosing class is still being defined.
   */
  bmi160(lab4::i2c& p_bus, lab2::steady_clock& p_clock);

  /**
   * @brief Whether the sensor answered and reached normal power mode.
   *
   * False means the sensor did not acknowledge, reported a chip ID other
   * than 0xD1, or never left suspend - normally a wiring or supply fault.
   */
  [[nodiscard]] bool initialized() const
  {
    return m_initialized;
  }

  /// Times the sensor stopped answering and was re-initialised after start-up.
  [[nodiscard]] std::uint32_t recovery_count() const
  {
    return m_recoveries;
  }

  /// Acceleration in standard gravities, as a lab4::accelerometer.
  [[nodiscard]] lab4::accelerometer& acceleration()
  {
    return m_acceleration_channel;
  }

  /// Angular velocity in degrees per second, as a lab4::gyroscope.
  [[nodiscard]] lab4::gyroscope& angular_velocity()
  {
    return m_angular_velocity_channel;
  }

private:
  // --- Sensor register map ------------------------------------------------
  static constexpr std::uint8_t register_chip_id = 0x00;               // 2.11.1
  static constexpr std::uint8_t register_power_status = 0x03;          // 2.11.3
  static constexpr std::uint8_t register_angular_velocity_data = 0x0C;  // 2.11.4
  static constexpr std::uint8_t register_acceleration_data = 0x12;      // 2.11.4
  static constexpr std::uint8_t register_acceleration_config = 0x40;    // 2.11.5
  static constexpr std::uint8_t register_acceleration_range = 0x41;     // 2.11.6
  static constexpr std::uint8_t register_angular_velocity_config = 0x42;  // 2.11.7
  static constexpr std::uint8_t register_angular_velocity_range = 0x43;   // 2.11.8
  static constexpr std::uint8_t register_command = 0x7E;               // 2.11.38

  static constexpr std::uint8_t expected_chip_id = 0xD1;
  static constexpr std::uint8_t command_soft_reset = 0xB6;
  static constexpr std::uint8_t command_acceleration_normal = 0x11;
  static constexpr std::uint8_t command_angular_velocity_normal = 0x15;

  /// ACC_CONF / GYR_CONF: bandwidth field = normal filter (0b010, bits 6:4),
  /// output data rate field = 100 Hz (0b1000, bits 3:0).
  static constexpr std::uint8_t config_bandwidth_normal = 2U << 4;
  static constexpr std::uint8_t config_output_rate_100hz = 8U << 0;
  static constexpr std::uint8_t config_100hz_normal = config_bandwidth_normal |
                                                      config_output_rate_100hz;

  /// PMU_STATUS: acc_pmu_status in bits 5:4, gyr_pmu_status in bits 3:2,
  /// both reading 0b01 when that sensor is in normal mode (datasheet 2.11.3).
  static constexpr std::uint8_t power_mode_normal = 0x01;
  static constexpr unsigned acceleration_mode_shift = 4;
  static constexpr unsigned angular_velocity_mode_shift = 2;
  static constexpr std::uint8_t power_mode_mask = 0x03;

  // Datasheet start-up timings, rounded up for margin. The gyroscope takes
  // up to 80 ms to come out of suspend - by far the longest wait here, and a
  // common cause of all-zero gyro readings when it is skipped.
  static constexpr std::uint32_t reset_delay_ms = 100;
  static constexpr std::uint32_t acceleration_start_up_ms = 10;
  static constexpr std::uint32_t angular_velocity_start_up_ms = 100;

  /// A sensor still settling after power-on may not answer the first try.
  static constexpr int start_up_attempts = 5;
  static constexpr std::uint32_t retry_delay_ms = 200;

  /// Six data bytes per sensor: X, Y, Z, each low byte then high byte.
  static constexpr std::size_t axis_data_bytes = 6;

  static constexpr float acceleration_full_scale(acceleration_range p_range)
  {
    switch (p_range) {
      case acceleration_range::plus_minus_2g:
        return 2.0F;
      case acceleration_range::plus_minus_4g:
        return 4.0F;
      case acceleration_range::plus_minus_8g:
        return 8.0F;
      case acceleration_range::plus_minus_16g:
        return 16.0F;
    }
    return 2.0F;
  }

  static constexpr float angular_velocity_full_scale(
    angular_velocity_range p_range)
  {
    switch (p_range) {
      case angular_velocity_range::plus_minus_2000dps:
        return 2000.0F;
      case angular_velocity_range::plus_minus_1000dps:
        return 1000.0F;
      case angular_velocity_range::plus_minus_500dps:
        return 500.0F;
      case angular_velocity_range::plus_minus_250dps:
        return 250.0F;
      case angular_velocity_range::plus_minus_125dps:
        return 125.0F;
    }
    return 2000.0F;
  }

  /// Each axis arrives as a little-endian two's-complement 16-bit value.
  static std::int16_t to_signed(std::uint8_t p_low, std::uint8_t p_high)
  {
    auto const unsigned_value = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(p_high) << 8) | p_low);
    return static_cast<std::int16_t>(unsigned_value);
  }

  bool write_register(std::uint8_t p_register, std::uint8_t p_value)
  {
    std::array<std::uint8_t, 2> const payload{ p_register, p_value };
    return m_bus.transaction(m_settings.address, payload, {});
  }

  bool read_registers(std::uint8_t p_register, std::span<std::uint8_t> p_into)
  {
    std::array<std::uint8_t, 1> const address{ p_register };
    return m_bus.transaction(m_settings.address, address, p_into);
  }

  /// Busy-wait using only the steady clock interface, so this driver needs no
  /// platform delay function of its own.
  void wait_milliseconds(std::uint32_t p_milliseconds)
  {
    auto const wait_ticks =
      (static_cast<std::uint64_t>(m_clock.frequency()) * p_milliseconds) / 1000;
    auto const start = m_clock.uptime();
    while (m_clock.uptime() - start < wait_ticks) {
    }
  }

  /// Retry the start-up sequence a few times before giving up.
  bool configure()
  {
    for (int attempt = 0; attempt < start_up_attempts; attempt++) {
      if (attempt_configure()) {
        return true;
      }
      wait_milliseconds(retry_delay_ms);
    }
    return false;
  }

  bool attempt_configure()
  {
    // A soft reset puts every register back to a known state, so a warm
    // restart behaves the same as a cold boot.
    if (!write_register(register_command, command_soft_reset)) {
      return false;
    }
    wait_milliseconds(reset_delay_ms);

    std::array<std::uint8_t, 1> chip_id{};
    if (!read_registers(register_chip_id, chip_id) ||
        chip_id[0] != expected_chip_id) {
      return false;
    }

    // Both sensors boot in suspend mode and must be commanded into normal
    // mode. The gyroscope needs by far the longer start-up window.
    if (!write_register(register_command, command_acceleration_normal)) {
      return false;
    }
    wait_milliseconds(acceleration_start_up_ms);

    if (!write_register(register_command, command_angular_velocity_normal)) {
      return false;
    }
    wait_milliseconds(angular_velocity_start_up_ms);

    // 100 Hz output data rate on both sensors, matching the rate the
    // application samples them at.
    if (!write_register(register_acceleration_config, config_100hz_normal) ||
        !write_register(register_acceleration_range,
                        static_cast<std::uint8_t>(m_settings.acceleration)) ||
        !write_register(register_angular_velocity_config,
                        config_100hz_normal) ||
        !write_register(
          register_angular_velocity_range,
          static_cast<std::uint8_t>(m_settings.angular_velocity))) {
      return false;
    }

    // Confirm the chip really is in normal mode rather than trusting that
    // the commands were accepted.
    std::array<std::uint8_t, 1> power_status{};
    if (!read_registers(register_power_status, power_status)) {
      return false;
    }
    auto const acceleration_mode =
      (power_status[0] >> acceleration_mode_shift) & power_mode_mask;
    auto const angular_velocity_mode =
      (power_status[0] >> angular_velocity_mode_shift) & power_mode_mask;

    return acceleration_mode == power_mode_normal &&
           angular_velocity_mode == power_mode_normal;
  }

  /// Consecutive failed reads that trigger a full sensor re-initialisation.
  /// The application reads twice per 10 ms cycle, so 20 is about 100 ms of
  /// silence - long enough to ignore a one-off glitch, short enough that a
  /// re-seated wire is picked up again almost immediately.
  static constexpr std::uint32_t failures_before_recovery = 20;

  /// Reads skipped without touching the bus after a re-initialisation fails.
  /// At two reads per 10 ms cycle this is about 250 ms between attempts.
  static constexpr std::uint32_t reads_skipped_while_absent = 50;

  /**
   * @brief Track read health and re-initialise the sensor if it went away.
   *
   * A loose jumper or a supply dip can reset the BMI160 back into suspend
   * mode, after which it answers with stale or zero data forever. Re-running
   * the start-up sequence brings it back without a board reset. One attempt
   * per trigger (not the five used at power-on) bounds the time spent here.
   */
  void note_read(bool p_succeeded)
  {
    if (p_succeeded) {
      m_consecutive_failures = 0;
      return;
    }
    if (++m_consecutive_failures < failures_before_recovery) {
      return;
    }
    m_consecutive_failures = 0;
    m_recoveries++;
    m_initialized = attempt_configure();
    if (!m_initialized) {
      m_reads_to_skip = reads_skipped_while_absent;
    }
  }

  /**
   * @brief Whether to skip the bus entirely because the sensor is absent.
   *
   * Every read of a missing sensor costs a bus timeout, and the caller is
   * typically the highest-priority task, so hammering a disconnected bus
   * would starve every other task - the heartbeat included. While absent,
   * reads return immediately without bus traffic, and one re-initialisation
   * is attempted each time the skip window runs out.
   */
  bool sensor_absent()
  {
    if (m_reads_to_skip == 0) {
      return false;
    }
    if (--m_reads_to_skip == 0) {
      m_recoveries++;
      m_initialized = attempt_configure();
      if (!m_initialized) {
        m_reads_to_skip = reads_skipped_while_absent;
      }
    }
    return !m_initialized;
  }

  // On a bus error the last good acceleration sample is returned rather than
  // zeros: 0 g on every axis would read as a wild tilt and kick the
  // complementary filter.
  lab4::accelerometer::read_t read_acceleration()
  {
    if (sensor_absent()) {
      return m_last_acceleration;
    }
    std::array<std::uint8_t, axis_data_bytes> raw{};
    bool const succeeded = read_registers(register_acceleration_data, raw);
    note_read(succeeded);
    if (!succeeded) {
      return m_last_acceleration;
    }

    m_last_acceleration = {
      static_cast<float>(to_signed(raw[0], raw[1])) * m_acceleration_scale,
      static_cast<float>(to_signed(raw[2], raw[3])) * m_acceleration_scale,
      static_cast<float>(to_signed(raw[4], raw[5])) * m_acceleration_scale,
    };
    return m_last_acceleration;
  }

  // On a bus error the gyroscope reports zero rotation, not the last good
  // rate. The filter integrates this value, so repeating a non-zero rate for
  // the length of an outage would walk the angle away without bound; zero
  // lets the estimate settle on the last good accelerometer angle instead.
  lab4::gyroscope::read_t read_angular_velocity()
  {
    if (sensor_absent()) {
      return { 0.0F, 0.0F, 0.0F };
    }
    std::array<std::uint8_t, axis_data_bytes> raw{};
    bool const succeeded = read_registers(register_angular_velocity_data, raw);
    note_read(succeeded);
    if (!succeeded) {
      return { 0.0F, 0.0F, 0.0F };
    }

    m_last_angular_velocity = {
      static_cast<float>(to_signed(raw[0], raw[1])) * m_angular_velocity_scale,
      static_cast<float>(to_signed(raw[2], raw[3])) * m_angular_velocity_scale,
      static_cast<float>(to_signed(raw[4], raw[5])) * m_angular_velocity_scale,
    };
    return m_last_angular_velocity;
  }

  /// Presents this sensor's acceleration output as a lab4::accelerometer.
  class acceleration_channel final : public lab4::accelerometer
  {
  public:
    explicit acceleration_channel(bmi160& p_parent)
      : m_parent(&p_parent)
    {
    }

  private:
    read_t driver_read() override
    {
      return m_parent->read_acceleration();
    }

    bmi160* m_parent;
  };

  /// Presents this sensor's angular velocity output as a lab4::gyroscope.
  class angular_velocity_channel final : public lab4::gyroscope
  {
  public:
    explicit angular_velocity_channel(bmi160& p_parent)
      : m_parent(&p_parent)
    {
    }

  private:
    read_t driver_read() override
    {
      return m_parent->read_angular_velocity();
    }

    bmi160* m_parent;
  };

  lab4::i2c& m_bus;
  lab2::steady_clock& m_clock;
  settings m_settings;
  float m_acceleration_scale;
  float m_angular_velocity_scale;
  bool m_initialized = false;
  std::uint32_t m_consecutive_failures = 0;
  std::uint32_t m_recoveries = 0;
  std::uint32_t m_reads_to_skip = 0;
  lab4::accelerometer::read_t m_last_acceleration{ 0.0F, 0.0F, 1.0F };
  lab4::gyroscope::read_t m_last_angular_velocity{ 0.0F, 0.0F, 0.0F };
  acceleration_channel m_acceleration_channel{ *this };
  angular_velocity_channel m_angular_velocity_channel{ *this };
};

inline bmi160::bmi160(lab4::i2c& p_bus, lab2::steady_clock& p_clock)
  : bmi160(p_bus, p_clock, settings{})
{
}

}  // namespace sensor
