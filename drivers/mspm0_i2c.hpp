#pragma once

// MSPM0G3507 I2C controller driver, implementing the lab4::i2c interface
// (hal/i2c.hpp). Carried over unchanged in behaviour from lab 4, where it was
// verified against the BMI160 on real hardware.
//
// I2C1 on PB2 (SCL) / PB3 (SDA), which are BoosterPack header pins 9 and 10.
// The MSPM0 pins are driven in HiZ mode to emulate open-drain, so the bus
// relies on external pull-up resistors - the ones on the BMI160 breakout.

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
constexpr std::uint32_t i2c_sda_pincm = IOMUX_PINCM16;
constexpr std::uint32_t i2c_sda_function = IOMUX_PINCM16_PF_I2C1_SDA;

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

private:
  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.clock_rate == 0) {
      return false;
    }

    // Power on the GPIO port and the I2C1 peripheral.
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOB);
    DL_I2C_reset(I2C1);
    DL_I2C_enablePower(I2C1);
    delay_cycles(10);

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

    // Run I2C1 off BUSCLK (32 MHz) undivided.
    DL_I2C_ClockConfig clock_config{ DL_I2C_CLOCK_BUSCLK,
                                     DL_I2C_CLOCK_DIVIDE_1 };
    DL_I2C_setClockConfig(I2C1, &clock_config);
    DL_I2C_disableAnalogGlitchFilter(I2C1);
    DL_I2C_resetControllerTransfer(I2C1);

    // SCL period = (1 + TPR) * 10 * bus_clock_period, so TPR sets the bit
    // rate. 100 kHz gives TPR = 31; 400 kHz gives TPR = 7.
    auto const period = static_cast<std::uint8_t>(
      i2c_bus_clock_hz / (p_settings.clock_rate * 10) - 1);
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
    if (!p_data_out.empty() && !write(p_address, p_data_out)) {
      return false;
    }

    if (!p_data_in.empty() && !read(p_address, p_data_in)) {
      return false;
    }

    return true;
  }

  bool write(std::uint8_t p_address, std::span<std::uint8_t const> p_data)
  {
    wait_idle();
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
    wait_idle();
    DL_I2C_startControllerTransfer(I2C1,
                                   p_address,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   static_cast<std::uint16_t>(p_data.size()));
    delay_cycles(30);

    for (auto& byte : p_data) {
      while (DL_I2C_isControllerRXFIFOEmpty(I2C1)) {
        if (DL_I2C_getControllerStatus(I2C1) &
            DL_I2C_CONTROLLER_STATUS_ERROR) {
          return false;
        }
      }
      byte = DL_I2C_receiveControllerData(I2C1);
    }

    return wait_done();
  }

  void wait_idle()
  {
    while (!(DL_I2C_getControllerStatus(I2C1) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
    }
  }

  bool wait_done()
  {
    while (DL_I2C_getControllerStatus(I2C1) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
    }
    return !(DL_I2C_getControllerStatus(I2C1) &
             DL_I2C_CONTROLLER_STATUS_ERROR);
  }
};

}  // namespace mspm0
