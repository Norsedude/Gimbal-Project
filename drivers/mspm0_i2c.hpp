#pragma once

// MSPM0G3507 I2C controller driver, implementing the lab4::i2c interface
// (hal/i2c.hpp). Register sequence carried over from lab 4, where it was
// verified against the BMI160 on real hardware, with two additions for the
// final project:
//
//   1. Every wait is bounded. A missing ACK, a loose wire, or a sensor that
//      stalls the bus makes the transaction return false within a few
//      milliseconds instead of hanging the calling task forever.
//   2. After a failed transaction the controller is reset and the bus is
//      un-stuck (nine SCL pulses, I2C spec UM10204 section 3.1.16), so the
//      next transaction starts from a clean state.
//
// I2C1 on PB2 (SCL) / PB3 (SDA), which are BoosterPack header pins 9 and 10.
// The MSPM0 pins are driven in HiZ mode to emulate open-drain, so the bus
// relies on external pull-up resistors - the ones on the BMI160 breakout.
//
// Transaction shape: a write-then-read is two complete I2C transfers, each
// with its own START and STOP:
//   START | 0x68+W | register | STOP   START | 0x68+R | data... NACK | STOP
// The BMI160 keeps its register pointer across the STOP, so this reads the
// same data a repeated-start transfer would.

#include <cstdint>

#include <ti/driverlib/driverlib.h>

#include "../hal/i2c.hpp"
#include "../third_party/span/span.hpp"

namespace mspm0 {

/// Default SYSOSC/BUSCLK rate at reset, which clocks I2C1.
constexpr std::uint32_t i2c_bus_clock_hz = 32000000;

// I2C1 on PB2 (SCL) / PB3 (SDA) - BoosterPack header pins 9/10.
constexpr std::uint32_t i2c_scl_pincm = IOMUX_PINCM15;
constexpr std::uint32_t i2c_scl_function = IOMUX_PINCM15_PF_I2C1_SCL;
constexpr std::uint32_t i2c_scl_pin = DL_GPIO_PIN_2;
constexpr std::uint32_t i2c_sda_pincm = IOMUX_PINCM16;
constexpr std::uint32_t i2c_sda_function = IOMUX_PINCM16_PF_I2C1_SDA;
constexpr std::uint32_t i2c_sda_pin = DL_GPIO_PIN_3;

/**
 * @brief Blocking I2C controller built on I2C1.
 *
 * Every transaction polls the controller status register until the transfer
 * finishes, so calls block the calling task for the duration of the transfer.
 * At 100 kHz a 1-byte write followed by a 6-byte read takes roughly 0.8 ms,
 * which fits comfortably inside the gimbal sensor task 10 ms period.
 */
class i2c final : public lab4::i2c
{
public:
  /**
   * @param p_settings - bus settings; defaults to 100 kHz, the rate verified
   * against the BMI160 breakout in lab 4.
   */
  explicit i2c(settings const& p_settings = {})
  {
    driver_configure(p_settings);
  }

  /// Transactions that failed (NACK, bus error, or timeout) since power-up.
  [[nodiscard]] std::uint32_t failure_count() const
  {
    return m_failures;
  }

private:
  /// Upper bound on status-register polls for any one wait. One poll costs
  /// roughly 8-15 bus cycles, so this is about 2-4 ms at 32 MHz. The longest
  /// legitimate single wait here is about 0.3 ms (a 2-byte write at 100 kHz),
  /// so this leaves a wide margin while keeping a missing sensor from
  /// monopolising the CPU: the caller is the highest-priority task.
  static constexpr std::uint32_t max_polls = 8000;

  /// Half an SCL period at 100 kHz, used while bit-banging bus recovery.
  static constexpr std::uint32_t recovery_half_period_cycles =
    i2c_bus_clock_hz / 200000;

  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.clock_rate == 0) {
      return false;
    }
    m_settings = p_settings;

    // Port B is only powered here, never reset: the servo PWM pin (PB15) and
    // the heartbeat LED (PB27) live on the same port, and resetting it would
    // undo their configuration whenever this driver re-initialises.
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(16);

    route_pins_to_controller();
    configure_controller();
    return true;
  }

  bool driver_transaction(std::uint8_t p_address,
                          std::span<std::uint8_t const> p_data_out,
                          std::span<std::uint8_t> p_data_in) override
  {
    bool ok = true;
    if (!p_data_out.empty()) {
      ok = write(p_address, p_data_out);
    }
    if (ok && !p_data_in.empty()) {
      ok = read(p_address, p_data_in);
    }

    if (!ok) {
      m_failures++;
      recover();
    }
    return ok;
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
      std::uint32_t polls = 0;
      while (DL_I2C_isControllerRXFIFOEmpty(I2C1)) {
        if ((DL_I2C_getControllerStatus(I2C1) &
             DL_I2C_CONTROLLER_STATUS_ERROR) ||
            ++polls >= max_polls) {
          return false;
        }
      }
      byte = DL_I2C_receiveControllerData(I2C1);
    }

    return wait_done();
  }

  bool wait_idle()
  {
    for (std::uint32_t polls = 0; polls < max_polls; polls++) {
      if (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_IDLE) {
        return true;
      }
    }
    return false;
  }

  bool wait_done()
  {
    for (std::uint32_t polls = 0; polls < max_polls; polls++) {
      auto const status = DL_I2C_getControllerStatus(I2C1);
      if (!(status & DL_I2C_CONTROLLER_STATUS_BUSY)) {
        return !(status & DL_I2C_CONTROLLER_STATUS_ERROR);
      }
    }
    return false;
  }

  /**
   * @brief Return the bus and the controller to a known-good state.
   *
   * If the sensor was interrupted mid-byte (a wire bumped, or the sensor
   * browned out) it may still be holding SDA low waiting for clocks. Nine
   * SCL pulses let it finish that byte and release SDA; a STOP then returns
   * the bus to idle. The I2C1 peripheral is then fully reset and
   * re-configured so no stale FIFO contents or error flags survive.
   */
  void recover()
  {
    // Take the pins away from I2C1 as plain open-drain GPIO, released high.
    DL_GPIO_initDigitalOutput(i2c_scl_pincm);
    DL_GPIO_initDigitalOutput(i2c_sda_pincm);
    DL_GPIO_enableHiZ(i2c_scl_pincm);
    DL_GPIO_enableHiZ(i2c_sda_pincm);
    DL_GPIO_setPins(GPIOB, i2c_scl_pin | i2c_sda_pin);
    DL_GPIO_enableOutput(GPIOB, i2c_scl_pin | i2c_sda_pin);
    delay_cycles(recovery_half_period_cycles);

    for (int pulse = 0; pulse < 9; pulse++) {
      DL_GPIO_clearPins(GPIOB, i2c_scl_pin);
      delay_cycles(recovery_half_period_cycles);
      DL_GPIO_setPins(GPIOB, i2c_scl_pin);
      delay_cycles(recovery_half_period_cycles);
    }

    // STOP condition: SDA rises while SCL is high.
    DL_GPIO_clearPins(GPIOB, i2c_sda_pin);
    delay_cycles(recovery_half_period_cycles);
    DL_GPIO_setPins(GPIOB, i2c_sda_pin);
    delay_cycles(recovery_half_period_cycles);
    DL_GPIO_disableOutput(GPIOB, i2c_scl_pin | i2c_sda_pin);

    route_pins_to_controller();
    configure_controller();
  }

  void route_pins_to_controller()
  {
    // Route PB2/PB3 to I2C1, open-drain (HiZ) as the I2C bus requires.
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

  void configure_controller()
  {
    DL_I2C_reset(I2C1);
    DL_I2C_enablePower(I2C1);
    delay_cycles(16);

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
  }

  settings m_settings{};
  std::uint32_t m_failures = 0;
};

}  // namespace mspm0
