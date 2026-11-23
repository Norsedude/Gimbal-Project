#include <cmath>
#include <cstdint>
#include <cstdio>
#include <span>

#include "ti/driverlib/driverlib.h"

#include "../hal/accelerometer.hpp"
#include "../hal/i2c.hpp"

namespace {

// BMI160 registers/commands used by this driver (see BMI160 datasheet).
constexpr std::uint8_t bmi160_cmd_reg = 0x7E; // 2.11.38 CMD
constexpr std::uint8_t bmi160_data_reg = 0x12;  // first of 6 bytes: X,Y,Z (LSB,MSB each) (2.11.4) DATA
constexpr std::uint8_t bmi160_acc_normal_mode = 0x11; // 2.11.38 CMD  01

constexpr std::uint32_t bus_clock_hz = 32'000'000;  // default SYSOSC/BUSCLK rate

// I2C1 on PB2 (SCL) / PB3 (SDA) - BoosterPack header pins 9/10.
constexpr std::uint32_t i2c_scl_pin = IOMUX_PINCM15;
constexpr std::uint32_t i2c_scl_func = IOMUX_PINCM15_PF_I2C1_SCL;
constexpr std::uint32_t i2c_sda_pin = IOMUX_PINCM16;
constexpr std::uint32_t i2c_sda_func = IOMUX_PINCM16_PF_I2C1_SDA;

class mspm0_i2c final : public lab4::i2c {
public:
  mspm0_i2c() {
    driver_configure({});
  }

private:
  bool driver_configure(settings const& config) override {
    if (config.clock_rate == 0) {
      return false;
    }

    // Power on the GPIO port and the I2C1 peripheral.
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOB);
    DL_I2C_reset(I2C1);
    DL_I2C_enablePower(I2C1);
    delay_cycles(10);

    // Route PB2/PB3 to I2C1, open-drain (HiZ) as the I2C bus requires.
    DL_GPIO_initPeripheralInputFunctionFeatures(
      i2c_scl_pin, i2c_scl_func, DL_GPIO_INVERSION_DISABLE,
      DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
      i2c_sda_pin, i2c_sda_func, DL_GPIO_INVERSION_DISABLE,
      DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_enableHiZ(i2c_scl_pin); // Fake open drain behavior
    DL_GPIO_enableHiZ(i2c_sda_pin); // Fake open drain behavior

    // Run I2C1 off BUSCLK (32 MHz) undivided.
    DL_I2C_ClockConfig clock_config{ DL_I2C_CLOCK_BUSCLK, DL_I2C_CLOCK_DIVIDE_1 };
    DL_I2C_setClockConfig(I2C1, &clock_config);
    DL_I2C_disableAnalogGlitchFilter(I2C1);
    DL_I2C_resetControllerTransfer(I2C1);

    // SCL period = (1 + TPR) * 10 * bus_clock_period, so TPR sets the bit rate.
    auto period = static_cast<std::uint8_t>(bus_clock_hz / (config.clock_rate * 10) - 1);
    DL_I2C_setTimerPeriod(I2C1, period);

    DL_I2C_setControllerTXFIFOThreshold(I2C1, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C1, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C1);
    DL_I2C_enableController(I2C1);

    return true;
  }

  bool driver_transaction(std::uint8_t address,
                          std::span<std::uint8_t const> data_out,
                          std::span<std::uint8_t> data_in) override {
    if (!data_out.empty() && !write(address, data_out)) {
      return false;
    }

    if (!data_in.empty() && !read(address, data_in)) {
      return false;
    }

    return true;
  }

  bool write(std::uint8_t address, std::span<std::uint8_t const> data) {
    wait_idle();
    DL_I2C_fillControllerTXFIFO(I2C1, data.data(), static_cast<std::uint16_t>(data.size()));
    DL_I2C_startControllerTransfer(
      I2C1, address, DL_I2C_CONTROLLER_DIRECTION_TX, static_cast<std::uint16_t>(data.size()));
    delay_cycles(30);  // errata I2C_ERR_13: let the transfer start before polling status
    return wait_done();
  }

  bool read(std::uint8_t address, std::span<std::uint8_t> data) {
    wait_idle();
    DL_I2C_startControllerTransfer(
      I2C1, address, DL_I2C_CONTROLLER_DIRECTION_RX, static_cast<std::uint16_t>(data.size()));
    delay_cycles(30);

    for (auto& byte : data) {
      while (DL_I2C_isControllerRXFIFOEmpty(I2C1)) {
        if (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_ERROR) {
          return false;
        }
      }
      byte = DL_I2C_receiveControllerData(I2C1);
    }

    return wait_done();
  }

  void wait_idle() {
    while (!(DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_IDLE)) {}
  }

  bool wait_done() {
    while (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_BUSY) {}
    return !(DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_ERROR);
  }
};

class bmi160 final : public lab4::accelerometer {
public:
  explicit bmi160(lab4::i2c& bus, std::uint8_t address = 0x68)
    : m_bus(bus), m_address(address) {
    // Chip boots into suspend mode; switch the accelerometer to normal mode.
    std::uint8_t cmd[2] = { bmi160_cmd_reg, bmi160_acc_normal_mode };
    m_bus.transaction(m_address, cmd, {});
    delay_cycles(320'000);  // ~10 ms accelerometer start-up time
  }
// i2c bus parameter, BMI address of the sensor

private:
  read_t driver_read() override {
    std::uint8_t reg = bmi160_data_reg;
    std::uint8_t raw[6]{};
    m_bus.transaction(
      m_address, std::span<std::uint8_t const>(&reg, 1), raw);

    // Default +-2g range -> 16384 counts per g; each axis is LSB then MSB.
    auto to_g = [](std::uint8_t lo, std::uint8_t hi) {
      auto value = static_cast<std::int16_t>((hi << 8) | lo);
      return value / 16384.0f;
    };

    return { to_g(raw[0], raw[1]), to_g(raw[2], raw[3]), to_g(raw[4], raw[5]) };
  }

  lab4::i2c& m_bus;
  std::uint8_t m_address;
};

}

int main() {
  std::printf("CMPE 146 Lab 4 - I2C IMU\n");

  mspm0_i2c i2c_bus;
  bmi160 imu(i2c_bus);

  while (true) {
    auto accel = imu.read();

    // Standard tilt formulas: roll about X, pitch about Y, from the gravity vector.
    float roll = std::atan2(accel.y, accel.z) * 180.0f / 3.14159265f;
    float pitch = std::atan2(-accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z))
                  * 180.0f / 3.14159265f;

    std::printf("Roll: %6.2f  Pitch: %6.2f\n", roll, pitch);
    delay_cycles(3'200'000);  // ~0.1 s between prints
  }
}
