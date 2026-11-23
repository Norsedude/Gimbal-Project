#pragma once

// MSPM0G3507 GPIO driver, implementing the lab1::output_pin interface
// (hal/gpio.hpp) directly against registers. Carried over from lab 1, trimmed
// to the output pin - the only kind this project uses (the heartbeat LED).
//
// Register names, offsets, and bit positions are from the MSPM0G350x TRM,
// chapter 10 (GPIO) and chapter 7 (IOMUX).

#include <cstddef>
#include <cstdint>

#include "../hal/gpio.hpp"

namespace mspm0 {

constexpr std::uintptr_t gpioa = 0x400A0000;
constexpr std::uintptr_t gpiob = 0x400A2000;

// Byte offsets of the GPIO registers this driver uses, taken directly from
// the TRM. Each offset is a fact you can check straight against the
// datasheet, instead of a struct whose correctness depends on summing every
// preceding reserved gap by hand.
namespace gpio_reg {
constexpr std::size_t power_enable = 0x800;
constexpr std::size_t data_out = 0x1280;
constexpr std::size_t data_out_set = 0x1290;
constexpr std::size_t data_out_clear = 0x12A0;
constexpr std::size_t output_enable = 0x12C0;
}  // namespace gpio_reg

inline std::uint32_t volatile& gpio_access(std::uintptr_t p_port,
                                           std::size_t p_reg)
{
  return *reinterpret_cast<std::uint32_t volatile*>(p_port + p_reg);
}

constexpr std::uintptr_t iomux_address = 0x40428000;

/// PINCMn is at IOMUX base + 4 * n.
inline std::uint32_t volatile& gpio_pin_mux(unsigned p_pincm)
{
  return *reinterpret_cast<std::uint32_t volatile*>(
    iomux_address + (p_pincm * sizeof(std::uint32_t)));
}

// PWREN register: a write must carry the unlock KEY in bits 31:24 or the
// write is ignored. ENABLE is bit 0.
constexpr std::uint32_t gpio_pwren_key = 0x26U << 24;
constexpr std::uint32_t gpio_pwren_enable = 1U << 0;
constexpr std::uint32_t gpio_power_on_value = gpio_pwren_key |
                                              gpio_pwren_enable;

/// PINCM.PF - pin function 1 selects plain GPIO.
constexpr std::uint32_t pin_function_gpio = 1U << 0;
/// PINCM.PC - connect the pin to the selected peripheral function.
constexpr std::uint32_t peripheral_connected = 1U << 7;

/**
 * @brief Digital output pin.
 *
 * The constructor performs the complete hardware setup - power the port, mux
 * the pin to GPIO, enable the output driver - so the pin is usable the moment
 * the object exists. There is no second initialization step.
 */
class output_pin final : public lab1::output_pin
{
public:
  /**
   * @param p_port - gpioa or gpiob
   * @param p_pin - pin number within that port
   * @param p_pincm - IOMUX slot for the physical pin
   */
  output_pin(std::uintptr_t p_port, unsigned p_pin, unsigned p_pincm)
    : m_port(p_port)
    , m_mask(1U << p_pin)
    , m_pincm(p_pincm)
  {
    driver_configure({});
  }

private:
  bool driver_configure(settings const&) override
  {
    gpio_access(m_port, gpio_reg::power_enable) = gpio_power_on_value;
    gpio_pin_mux(m_pincm) = pin_function_gpio | peripheral_connected;
    gpio_access(m_port, gpio_reg::output_enable) |= m_mask;
    return true;
  }

  void driver_level(bool p_high) override
  {
    // Separate set and clear registers, so changing one pin never
    // read-modify-writes the others.
    gpio_access(m_port,
                p_high ? gpio_reg::data_out_set : gpio_reg::data_out_clear) =
      m_mask;
  }

  [[nodiscard]] bool driver_level() override
  {
    return (gpio_access(m_port, gpio_reg::data_out) & m_mask) != 0;
  }

  std::uintptr_t m_port;
  std::uint32_t m_mask;
  unsigned m_pincm;
};

}  // namespace mspm0
