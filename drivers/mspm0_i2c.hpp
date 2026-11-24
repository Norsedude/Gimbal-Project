#pragma once

// MSPM0G3507 I2C controller driver, implementing the lab4::i2c interface
// (hal/i2c.hpp). Carried over from lab 4, where it was verified against the
// BMI160 on real hardware, with the bounded waits described below added for
// the gimbal.
//
// I2C1 on PB2 (SCL) / PB3 (SDA), which are BoosterPack header pins 9 and 10.
// The MSPM0 pins are driven in HiZ mode to emulate open-drain, so the bus
// relies on external pull-up resistors - the ones on the BMI160 breakout.
//
// Bounded waits
// -------------
// Every poll loop here is bounded by a deadline taken from a steady clock.
// The lab 4 version spun forever, which is harmless on a bench where the
// sensor always answers, but in the gimbal one loose jumper parked the sensor
// task inside this file permanently. The heartbeat kept blinking from its own
// task, so the board looked alive while the angle estimate sat frozen at its
// last value. A bounded wait turns that into a failed transaction, which the
// lab4::i2c contract already provides for, and the caller keeps running.
//
// A failed transaction also resets the I2C peripheral, so a re-seated jumper
// starts working again without a power cycle.

#include <cstdint>

#include <ti/driverlib/driverlib.h>

#include "../hal/i2c.hpp"
#include "../hal/timer.hpp"
#include "../third_party/span/span.hpp"

namespace mspm0 {

/// Default SYSOSC/BUSCLK rate at reset, which clocks I2C1.
constexpr std::uint32_t i2c_bus_clock_hz = 32000000;

// I2C1 on PB2 (SCL) / PB3 (SDA) - BoosterPack header pins 9/10.
constexpr std::uint32_t i2c_scl_pincm = IOMUX_PINCM15;
constexpr std::uint32_t i2c_scl_function = IOMUX_PINCM15_PF_I2C1_SCL;
constexpr std::uint32_t i2c_sda_pincm = IOMUX_PINCM16;
constexpr std::uint32_t i2c_sda_function = IOMUX_PINCM16_PF_I2C1_SDA;

/**
 * @brief Blocking I2C controller built on I2C1.
 *
 * Every transaction polls the controller status register until the transfer
 * finishes, so calls block the calling task for the duration of the transfer.
 * At 100 kHz a 1-byte write followed by a 6-byte read takes roughly 0.8 ms,
 * which fits comfortably inside the gimbal sensor task 10 ms period.
 *
 * No transaction can block longer than the configured timeout, so a silent
 * sensor costs one control cycle rather than the whole run.
 */
class i2c final : public lab4::i2c
{
public:
  /**
   * @brief How long a single transfer may stall before it is declared failed.
   *
   * The longest legitimate transfer on this bus is a 1-byte write followed by
   * a 6-byte read, about 0.8 ms at 100 kHz. 5 ms is roughly six times that,
   * while still being short enough that even two timed-out transactions in
   * one pass cost a single 10 ms control cycle instead of the run.
   */
  static constexpr std::uint32_t default_timeout_milliseconds = 5;

  /**
   * @param p_clock - steady clock used to bound how long a stalled transfer
   * can block the caller. Must outlive this object.
   * @param p_settings - bus settings; defaults to 100 kHz, the rate verified
   * against the BMI160 breakout in lab 4.
   * @param p_timeout_milliseconds - per-transfer stall budget.
   */
  explicit i2c(lab2::steady_clock& p_clock,
               settings const& p_settings = {},
               std::uint32_t p_timeout_milliseconds =
                 default_timeout_milliseconds)
    : m_clock(&p_clock)
    , m_timeout_ticks(timeout_to_ticks(p_clock, p_timeout_milliseconds))
  {
    driver_configure(p_settings);
  }

  /**
   * @brief Transactions that failed or timed out since boot.
   *
   * Zero on a healthy bus. A climbing count means the sensor wiring is
   * intermittent, which is worth seeing on the telemetry line because the
   * device driver above deliberately hides a single dropped sample by
   * reporting the last good one.
   */
  [[nodiscard]] std::uint32_t fault_count() const
  {
    return m_fault_count;
  }

  /// Whether the most recent transaction completed successfully.
  [[nodiscard]] bool healthy() const
  {
    return m_healthy;
  }

private:
  /// Converts a millisecond budget into steady-clock ticks, never zero.
  static std::uint64_t timeout_to_ticks(lab2::steady_clock& p_clock,
                                        std::uint32_t p_milliseconds)
  {
    auto const ticks =
      (static_cast<std::uint64_t>(p_clock.frequency()) * p_milliseconds) /
      1000;
    return (ticks == 0) ? 1 : ticks;
  }

  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.clock_rate == 0) {
      return false;
    }

    // Kept so a fault can re-apply the same bit rate during recovery.
    m_settings = p_settings;

    configure_pins();
    return configure_peripheral();
  }

  /**
   * @brief Route PB2/PB3 to I2C1 as open-drain.
   *
   * Separate from configure_peripheral() because this resets all of GPIO
   * port B, which also carries the servo PWM output (PB15) and the heartbeat
   * LED (PB27). It must run once at construction, before those drivers are
   * built, and never again - recovery touches the I2C peripheral only.
   */
  void configure_pins()
  {
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOB);

    DL_GPIO_initPeripheralInputFunctionFeatures(i2c_scl_pincm,
                                                i2c_scl_function,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(i2c_sda_pincm,
                                                i2c_sda_function,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_enableHiZ(i2c_scl_pincm);  // emulate open drain
    DL_GPIO_enableHiZ(i2c_sda_pincm);  // emulate open drain
  }

  /// Brings I2C1 itself up from reset. Safe to re-run at any time.
  bool configure_peripheral()
  {
    DL_I2C_reset(I2C1);
    DL_I2C_enablePower(I2C1);
    delay_cycles(10);

    // Run I2C1 off BUSCLK (32 MHz) undivided.
    DL_I2C_ClockConfig clock_config{ DL_I2C_CLOCK_BUSCLK,
                                     DL_I2C_CLOCK_DIVIDE_1 };
    DL_I2C_setClockConfig(I2C1, &clock_config);
    DL_I2C_disableAnalogGlitchFilter(I2C1);
    DL_I2C_resetControllerTransfer(I2C1);

    // SCL period = (1 + TPR) * 10 * bus_clock_period, so TPR sets the bit
    // rate. 100 kHz gives TPR = 31; 400 kHz gives TPR = 7.
    auto const period = static_cast<std::uint8_t>(
      i2c_bus_clock_hz / (m_settings.clock_rate * 10) - 1);
    DL_I2C_setTimerPeriod(I2C1, period);

    DL_I2C_setControllerTXFIFOThreshold(I2C1, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C1, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C1);
    DL_I2C_enableController(I2C1);

    return true;
  }

  bool driver_transaction(std::uint8_t p_address,
                          std::span<std::uint8_t const> p_data_out,
                          std::span<std::uint8_t> p_data_in) override
  {
    auto const succeeded =
      attempt_transaction(p_address, p_data_out, p_data_in);

    m_healthy = succeeded;
    if (!succeeded) {
      m_fault_count++;
      recover_bus();
    }

    return succeeded;
  }

  bool attempt_transaction(std::uint8_t p_address,
                           std::span<std::uint8_t const> p_data_out,
                           std::span<std::uint8_t> p_data_in)
  {
    if (!p_data_out.empty() && !write(p_address, p_data_out)) {
      return false;
    }

    if (!p_data_in.empty() && !read(p_address, p_data_in)) {
      return false;
    }

    return true;
  }

  /**
   * @brief Put the controller back into a known-good state after a fault.
   *
   * A timed-out transfer can leave the state machine part-way through a
   * transaction and the FIFOs holding stale bytes. Clearing the transfer,
   * flushing both FIFOs and re-initializing the peripheral means the next
   * transaction starts clean, so re-seating a jumper is enough to resume
   * without a power cycle.
   *
   * Only I2C1 is touched - not the GPIO port, which the servo and LED share.
   *
   * This does not rescue a bus wedged by the sensor itself holding SDA low,
   * which needs nine manual clock pulses to shift out. That has not been seen
   * on this hardware; a loose jumper shows up as failed transactions instead.
   */
  void recover_bus()
  {
    DL_I2C_resetControllerTransfer(I2C1);
    DL_I2C_flushControllerTXFIFO(I2C1);
    DL_I2C_flushControllerRXFIFO(I2C1);
    (void) configure_peripheral();
  }

  bool write(std::uint8_t p_address, std::span<std::uint8_t const> p_data)
  {
    if (!wait_idle()) {
      return false;
    }

    DL_I2C_fillControllerTXFIFO(
      I2C1, p_data.data(), static_cast<std::uint16_t>(p_data.size()));
    DL_I2C_startControllerTransfer(I2C1,
                                   p_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   static_cast<std::uint16_t>(p_data.size()));
    // errata I2C_ERR_13: let the transfer start before polling status
    delay_cycles(30);
    return wait_done();
  }

  bool read(std::uint8_t p_address, std::span<std::uint8_t> p_data)
  {
    if (!wait_idle()) {
      return false;
    }

    DL_I2C_startControllerTransfer(I2C1,
                                   p_address,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   static_cast<std::uint16_t>(p_data.size()));
    delay_cycles(30);

    for (auto& byte : p_data) {
      // Each byte gets its own budget, so the deadline also guards against
      // the sensor going quiet part-way through a burst.
      auto const byte_deadline = deadline();
      while (DL_I2C_isControllerRXFIFOEmpty(I2C1)) {
        if (DL_I2C_getControllerStatus(I2C1) &
            DL_I2C_CONTROLLER_STATUS_ERROR) {
          return false;
        }
        if (expired(byte_deadline)) {
          return false;
        }
      }
      byte = DL_I2C_receiveControllerData(I2C1);
    }

    return wait_done();
  }

  /// Waits for the controller to go idle before starting a new transfer.
  bool wait_idle()
  {
    auto const idle_deadline = deadline();
    while (!(DL_I2C_getControllerStatus(I2C1) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
      if (expired(idle_deadline)) {
        return false;
      }
    }
    return true;
  }

  /// Waits for the transfer in flight to finish, then reports bus errors.
  bool wait_done()
  {
    auto const done_deadline = deadline();
    while (DL_I2C_getControllerStatus(I2C1) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
      if (expired(done_deadline)) {
        return false;
      }
    }
    return !(DL_I2C_getControllerStatus(I2C1) &
             DL_I2C_CONTROLLER_STATUS_ERROR);
  }

  /// Absolute clock reading at which the current wait gives up. The steady
  /// clock is 64-bit and monotonic, so this needs no wrap handling.
  [[nodiscard]] std::uint64_t deadline()
  {
    return m_clock->uptime() + m_timeout_ticks;
  }

  [[nodiscard]] bool expired(std::uint64_t p_deadline)
  {
    return m_clock->uptime() >= p_deadline;
  }

  lab2::steady_clock* m_clock;
  std::uint64_t m_timeout_ticks;
  settings m_settings{};
  std::uint32_t m_fault_count = 0;
  bool m_healthy = true;
};

}  // namespace mspm0
