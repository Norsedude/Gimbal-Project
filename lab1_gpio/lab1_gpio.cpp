#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../hal/gpio.hpp"

constexpr std::uintptr_t gpioa = 0x400A0000;
constexpr std::uintptr_t gpiob = 0x400A2000;

// Byte offsets of the GPIO registers this driver uses, taken directly from
// the MSPM0G350x TRM. Each offset is a fact you can check straight against
// the datasheet, instead of a struct with reserved-padding gaps whose
// correctness depends on summing every preceding array's size by hand.
namespace reg {
constexpr std::size_t power_enable = 0x800;
constexpr std::size_t data_out = 0x1280;
constexpr std::size_t data_out_set = 0x1290;
constexpr std::size_t data_out_clear = 0x12A0;
constexpr std::size_t output_enable = 0x12C0;
constexpr std::size_t data_in = 0x1380;
}  // namespace reg

std::uint32_t volatile& access(std::uintptr_t port, std::size_t offset)
{
  return *reinterpret_cast<std::uint32_t volatile*>(port + offset);
}

// IOMUX pin-mux registers are contiguous, so a plain struct is fine here.
struct pin_mux_register
{
  std::uint32_t reserved0;
  std::uint32_t volatile pin_function[251];
};

constexpr std::uintptr_t iomux_address = 0x40428000;

pin_mux_register* pin_mux()
{
  return reinterpret_cast<pin_mux_register*>(iomux_address);
}

constexpr std::uint32_t power_on_value = 0x26000001;
constexpr std::uint32_t pin_function_gpio = 1;
constexpr std::uint32_t peripheral_connected = 0x00000080;
constexpr std::uint32_t input_enable = 0x00040000;
constexpr std::uint32_t pull_up_enable = 0x00020000;
constexpr std::uint32_t pull_down_enable = 0x00010000;

class mspm0_output_pin : public lab1::output_pin
{
public:
  // Constructor performs the full pin setup immediately - there is no
  // separate required step before level() can be used.
  mspm0_output_pin(std::uintptr_t port, unsigned pin, unsigned pin_function_slot)
    : m_port(port), m_mask(1u << pin), m_slot(pin_function_slot)
  {
    driver_configure({});
  }

private:
  bool driver_configure(settings const&) override
  {
    access(m_port, reg::power_enable) = power_on_value;
    pin_mux()->pin_function[m_slot - 1] = pin_function_gpio | peripheral_connected;
    access(m_port, reg::output_enable) |= m_mask;
    return true;
  }

  void driver_level(bool p_high) override
  {
    access(m_port, p_high ? reg::data_out_set : reg::data_out_clear) = m_mask;
  }

  [[nodiscard]] bool driver_level() override
  {
    return (access(m_port, reg::data_out) & m_mask) != 0;
  }

  std::uintptr_t m_port;
  std::uint32_t m_mask;
  unsigned m_slot;
};

class mspm0_input_pin : public lab1::input_pin
{
public:
  // Constructor performs the full pin setup immediately, including the
  // resistor choice, so the object is valid as soon as it exists.
  mspm0_input_pin(std::uintptr_t port,
                  unsigned pin,
                  unsigned pin_function_slot,
                  lab1::pin_resistor resistor = lab1::pin_resistor::pull_up)
    : m_port(port), m_mask(1u << pin), m_slot(pin_function_slot)
  {
    driver_configure({ .resistor = resistor });
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    std::uint32_t value = pin_function_gpio | peripheral_connected | input_enable;

    if (p_settings.resistor == lab1::pin_resistor::pull_up) {
      value |= pull_up_enable;
    } else if (p_settings.resistor == lab1::pin_resistor::pull_down) {
      value |= pull_down_enable;
    }

    access(m_port, reg::power_enable) = power_on_value;
    pin_mux()->pin_function[m_slot - 1] = value;
    return true;
  }

  [[nodiscard]] bool driver_level() override
  {
    return (access(m_port, reg::data_in) & m_mask) != 0;
  }

  std::uintptr_t m_port;
  std::uint32_t m_mask;
  unsigned m_slot;
};

int main()
{
  std::printf("Hello, World\n");

  mspm0_output_pin led_red(gpiob, 26, 57);
  mspm0_output_pin led_green(gpiob, 27, 58);
  mspm0_output_pin led_blue(gpiob, 22, 50);

  mspm0_input_pin s1_button(gpioa, 18, 40, lab1::pin_resistor::pull_down);
  mspm0_input_pin s2_button(gpiob, 21, 49, lab1::pin_resistor::pull_up);

  while (true) {
    bool s1_on = s1_button.level();
    bool s2_on = !s2_button.level();

    if (s1_on && s2_on) {  //  Both buttons on = blue
      led_red.level(false);
      led_green.level(false);
      led_blue.level(true);
    } else if (s1_on) {  //  Button 1 on = red
      led_red.level(true);
      led_green.level(false);
      led_blue.level(false);
    } else if (s2_on) {  // Button 2 on = green
      led_red.level(false);
      led_green.level(true);
      led_blue.level(false);
    } else {  //  No activations (default case)
      led_red.level(false);
      led_green.level(false);
      led_blue.level(false);
    }
  }
}
