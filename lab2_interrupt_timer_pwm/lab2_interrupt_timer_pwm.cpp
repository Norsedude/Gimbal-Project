/**
 * @file main.cpp
 * @brief Lab 2 - Timers and PWM: RGB color wheel
 *
 * - Steady clock: TIMG12 (32-bit), BUSCLK / 8 = 4 MHz, extended to 64 bits
 * - Delay:        delay_ms() built on the steady clock
 * - PWM:          frequency passed to the constructor and converted into
 *                 prescaler (CPS) and period (LOAD) register values
 *                   Red   PB26 = TIMG6_C0 (pin function 5)
 *                   Green PB27 = TIMG6_C1 (pin function 5)
 *                   Blue  PB22 = TIMG8_C1 (pin function 3)
 * - Color wheel:  three cosine waves 120 degrees apart
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <type_traits>

#include "../hal/gpio.hpp"
#include "../hal/timer.hpp"

// ============================================================================
// Utilities
// ============================================================================

/**
 * @brief Converts an `enum class` into its underlying integral value
 *
 * @tparam Enum - enum class type (do not supply, let the compiler deduce it)
 * @param p_value - enumeration value
 * @return the integral value of the enumeration
 */
template<typename Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum p_value) noexcept
{
  static_assert(std::is_enum_v<Enum>, "to_underlying requires an enum type");
  return static_cast<std::underlying_type_t<Enum>>(p_value);
}

/// Written to a peripheral's PWREN register: key (0x26) + enable bit.
constexpr std::uint32_t power_on_value = 0x2600'0001;

/// Peripherals need a few cycles after PWREN before their registers respond.
inline void power_up_delay()
{
  for (int i = 0; i < 16; i++) {
    asm volatile("nop");
  }
}

// ============================================================================
// Cosine lookup table (drives the color wheel)
// ============================================================================

constexpr double pi = 3.14159265358979323846;  // M_PI isn't standard C++

// Compile time implementation of COSINE because std::cos() isn't constexpr.
// Borrowed from: https://en.cppreference.com/cpp/numeric/math/cos
constexpr double custom_cosine(double x)
{
  double cos{ 1 }, pow{ x };
  for (auto fac{ 1ull }, n{ 1ull }; n != 19; fac *= ++n, pow *= x)
    if ((n & 1) == 0)
      cos += (n & 2 ? -pow : pow) / fac;
  return cos;
}

/**
 * @brief Generates one full cosine period, scaled to [0, 65535], at compile
 * time: max -> 0 -> max.
 *
 * @tparam CycleSteps - Size of the lookup table and the granularity of the
 * cosine wave steps.
 */
template<std::size_t CycleSteps>
constexpr std::array<std::uint16_t, CycleSteps> generate_cosine_table()
{
  std::array<std::uint16_t, CycleSteps> samples{};
  constexpr auto max = std::numeric_limits<std::uint16_t>::max();
  constexpr double phase_step = (2.0 * pi) / CycleSteps;
  for (std::size_t x = 0; x < CycleSteps; x++) {
    // cosine is in [-1, 1]; add 1.0 and divide by 2 to move it into [0, 1].
    auto y = (custom_cosine(phase_step * static_cast<double>(x)) + 1.0) / 2.0;
    y = (y < 0.0) ? 0.0 : (y > 1.0 ? 1.0 : y);  // guard series rounding
    samples[x] = static_cast<std::uint16_t>(y * max);
  }
  return samples;
}

constexpr auto uint16_cosine = generate_cosine_table<628>();

// ============================================================================
// Pin mux (IOMUX) and GPIO power - needed to route timer outputs to pins
// ============================================================================

constexpr std::uintptr_t pin_mux_address = 0x4042'8000;
constexpr std::uintptr_t gpioa_address = 0x400A'0000;
constexpr std::uintptr_t gpiob_address = 0x400A'2000;
constexpr std::uintptr_t gpio_pwren_offset = 0x800;
constexpr std::uint32_t peripheral_connected = 1u << 7;  // PINCM.PC

/// PINCMn is at IOMUX base + 4 * n
inline std::uint32_t volatile& pin_mux_register(std::uint8_t p_pincm)
{
  return *reinterpret_cast<std::uint32_t volatile*>(
    pin_mux_address + p_pincm * sizeof(std::uint32_t));
}

inline void power_gpio_port(char p_port)
{
  auto const base = (p_port == 'A') ? gpioa_address : gpiob_address;
  *reinterpret_cast<std::uint32_t volatile*>(base + gpio_pwren_offset) =
    power_on_value;
  power_up_delay();
}

// ============================================================================
// Timer definitions (shared by the steady clock and PWM drivers)
// ============================================================================

// Every available capture and compare channel.
enum class timer_channel : std::uint8_t
{
  channel_0 = 0,
  channel_1,
  channel_2,
  channel_3,
};

// Every address for every timer peripheral
enum class timer_peripheral : std::uintptr_t
{
  timer_g0 = 0x4008'4000UL,
  timer_g6 = 0x4086'8000UL,
  timer_g7 = 0x4086'A000UL,
  timer_g8 = 0x4009'0000UL,
  timer_g12 = 0x4087'0000UL,
  timer_a0 = 0x4086'0000UL,
  timer_a1 = 0x4086'2000UL,
};

// Timer register offsets. The *0 entries are the first of 4 per-channel
// registers spaced 4 bytes apart; add (channel * 4) to reach the others.
enum class timer_reg : std::uintptr_t
{
  pwren = 0x800,     // Power enable
  clkdiv = 0x1000,   // Clock divider
  clksel = 0x1008,   // Clock select
  ccpd = 0x1100,     // Capture/compare pin direction
  odis = 0x1104,     // Output disable
  cclkctl = 0x1108,  // Counter clock control
  cps = 0x110C,      // Clock prescaler
  ctr = 0x1800,      // Counter value
  ctrctl = 0x1804,   // Counter control
  load = 0x1808,     // Load value
  cc0 = 0x1810,      // Capture/compare value [4]
  ccctl0 = 0x1830,   // Capture/compare control [4]
  octl0 = 0x1850,    // Output control [4]
  ccact0 = 0x1870,   // Capture/compare action [4]
};

/**
 * @brief Returns a timer register as base address + offset (+ channel * 4 for
 * the per-channel registers).
 */
inline std::uint32_t volatile& timer_register(timer_peripheral p_timer,
                                              timer_reg p_reg,
                                              std::uint8_t p_index = 0)
{
  auto const address = to_underlying(p_timer) + to_underlying(p_reg) +
                       p_index * sizeof(std::uint32_t);
  return *reinterpret_cast<std::uint32_t volatile*>(address);
}

/// BUSCLK frequency at reset (SYSOSC, no PLL). Timer input clock by default.
constexpr std::uint32_t busclk_hz = 32'000'000;

constexpr std::uint32_t clksel_busclk = 1u << 3;  // CLKSEL.BUSCLK_SEL
constexpr std::uint32_t cclkctl_clken = 1u << 0;  // CCLKCTL.CLKEN
constexpr std::uint32_t ctrctl_enable = 1u << 0;  // CTRCTL.EN
constexpr std::uint32_t ctrctl_repeat = 1u << 1;  // CTRCTL.REPEAT = 1
constexpr std::uint32_t ctrctl_count_up = 2u << 4;  // CTRCTL.CM = up
// (CTRCTL.CM = 0, the default, counts down.)

// CCACT actions: 1 = set output high, 2 = set output low
constexpr std::uint32_t ccact_zero_set_high = 1u << 0;        // ZACT
constexpr std::uint32_t ccact_compare_up_set_low = 2u << 9;   // CUACT

// ============================================================================
// Steady clock
// ============================================================================

/**
 * @brief Free-running time base built on a 32-bit general purpose timer.
 *
 * TIMCLK = BUSCLK / (CLKDIV.RATIO + 1). The counter counts down from
 * 0xFFFFFFFF, so the elapsed tick count is simply ~CTR. Each time that
 * 32-bit value wraps, an overflow counter extends it to 64 bits.
 */
class mspm0_steady_clock : public lab2::steady_clock
{
public:
  /**
   * @param p_timer - which timer to use; needs a 32-bit counter (TIMG12)
   * @param p_divider - clock divider, 1 to 8. The default 8 gives 4 MHz.
   */
  explicit mspm0_steady_clock(
    timer_peripheral p_timer = timer_peripheral::timer_g12,
    std::uint32_t p_divider = 8)
    : m_timer(p_timer)
    , m_frequency(busclk_hz / p_divider)
  {
    reg(timer_reg::pwren) = power_on_value;
    power_up_delay();

    reg(timer_reg::clksel) = clksel_busclk;
    reg(timer_reg::clkdiv) = p_divider - 1;  // RATIO: 0 means divide by 1
    reg(timer_reg::cclkctl) = cclkctl_clken;

    reg(timer_reg::ctrctl) = 0;  // stop while configuring
    reg(timer_reg::load) = 0xFFFF'FFFF;
    reg(timer_reg::ctrctl) = ctrctl_repeat | ctrctl_enable;
  }

private:
  std::uint32_t driver_frequency() override
  {
    return m_frequency;
  }

  std::uint64_t driver_uptime() override
  {
    // Counting down from all 1s, so ~CTR counts up from 0.
    std::uint32_t const ticks = ~reg(timer_reg::ctr);
    if (ticks < m_last_ticks) {
      m_overflows++;  // the 32-bit counter wrapped since the last call
    }
    m_last_ticks = ticks;
    return (static_cast<std::uint64_t>(m_overflows) << 32) | ticks;
  }

  std::uint32_t volatile& reg(timer_reg p_reg)
  {
    return timer_register(m_timer, p_reg);
  }

  timer_peripheral m_timer;
  std::uint32_t m_frequency;
  std::uint32_t m_last_ticks = 0;
  std::uint32_t m_overflows = 0;
};

/**
 * @brief Blocks the caller for p_milliseconds using any steady clock.
 */
void delay_ms(lab2::steady_clock& p_clock, std::uint32_t p_milliseconds)
{
  auto const ticks_per_ms = static_cast<std::uint64_t>(p_clock.frequency()) /
                            1000;
  auto const wait_ticks = ticks_per_ms * p_milliseconds;
  auto const start = p_clock.uptime();
  while (p_clock.uptime() - start < wait_ticks) {
  }
}

// ============================================================================
// PWM
// ============================================================================

struct pwm_pin
{
  timer_peripheral peripheral;
  timer_channel channel;
  char port;
  std::uint8_t pin;
  std::uint8_t mux_index;  // pin function number for the timer output
  std::uint8_t pincm;      // IOMUX slot for the physical pin
};

/**
 * @brief Edge-aligned, up-counting PWM on one timer channel.
 *
 * Frequency math (same shape as the slide's TIMCLK equation):
 *   TIMCLK       = BUSCLK / (CPS.PCNT + 1)          (CLKDIV left at /1)
 *   period ticks = TIMCLK / frequency               -> LOAD = ticks - 1
 *   high ticks   = period ticks * duty cycle        -> CC
 * The prescaler is the smallest value that makes the period fit in the
 * 16-bit counter, which keeps the duty-cycle resolution as fine as possible.
 *
 * Output: set HIGH when the counter wraps to 0, set LOW when it counts up to
 * CC (Figure 35-29 in the slides).
 *
 * Channels on the same timer share LOAD, so they share one frequency.
 */
class mspm0_pwm : public lab2::pwm
{
public:
  using reg = timer_reg;

  // Red LED: TIMG6_C0 [5]
  static constexpr pwm_pin pb26_timer_g6_channel_0 = {
    .peripheral = timer_peripheral::timer_g6,
    .channel = timer_channel::channel_0,
    .port = 'B',
    .pin = 26,
    .mux_index = 5,
    .pincm = 57,
  };

  // Green LED: TIMG6_C1 [5]
  static constexpr pwm_pin pb27_timer_g6_channel_1 = {
    .peripheral = timer_peripheral::timer_g6,
    .channel = timer_channel::channel_1,
    .port = 'B',
    .pin = 27,
    .mux_index = 5,
    .pincm = 58,
  };

  // Blue LED: TIMG8_C1 [3]
  static constexpr pwm_pin pb22_timer_g8_channel_1 = {
    .peripheral = timer_peripheral::timer_g8,
    .channel = timer_channel::channel_1,
    .port = 'B',
    .pin = 22,
    .mux_index = 3,
    .pincm = 50,
  };

  static constexpr std::uint32_t max_period_ticks = 0xFFFF;  // 16-bit timer
  static constexpr std::uint32_t max_prescaler = 256;        // 8-bit CPS

  /**
   * @param p_pin - timer channel and pin to drive
   * @param p_frequency_hz - PWM frequency. Must be between about 2 Hz and
   * BUSCLK / 2 for the prescaler and period to be representable.
   */
  mspm0_pwm(pwm_pin const& p_pin, std::uint32_t p_frequency_hz)
    : m_pin(p_pin)
  {
    // 1. Frequency -> register values
    std::uint32_t const total_ticks = busclk_hz / p_frequency_hz;
    std::uint32_t prescaler =
      (total_ticks + max_period_ticks - 1) / max_period_ticks;  // round up
    if (prescaler < 1) {
      prescaler = 1;
    } else if (prescaler > max_prescaler) {
      prescaler = max_prescaler;
    }
    m_period_ticks = busclk_hz / (prescaler * p_frequency_hz);
    if (m_period_ticks > max_period_ticks) {
      m_period_ticks = max_period_ticks;
    } else if (m_period_ticks < 2) {
      m_period_ticks = 2;
    }
    m_actual_frequency = busclk_hz / (prescaler * m_period_ticks);

    // 2. Power and clock
    reg_at(reg::pwren) = power_on_value;
    power_up_delay();
    reg_at(reg::clksel) = clksel_busclk;
    reg_at(reg::clkdiv) = 0;              // divide by 1
    reg_at(reg::cps) = prescaler - 1;     // PCNT: 0 means divide by 1
    reg_at(reg::cclkctl) = cclkctl_clken;

    // 3. Counter period and channel setup (timer stopped meanwhile)
    reg_at(reg::ctrctl) = 0;
    reg_at(reg::load) = m_period_ticks - 1;
    reg_at(reg::ccctl0, channel()) = 0;  // compare mode
    reg_at(reg::octl0, channel()) = 0;   // pin follows the CCACT actions
    driver_duty_cycle(0);                // start dark
    reg_at(reg::ccpd) |= channel_bit();  // channel is an output
    reg_at(reg::odis) &= ~channel_bit();  // don't force the output low

    // 4. Route the timer output to the pin
    power_gpio_port(m_pin.port);
    pin_mux_register(m_pin.pincm) = m_pin.mux_index | peripheral_connected;

    // 5. Start: count up 0 -> LOAD, repeat forever
    reg_at(reg::ctrctl) = ctrctl_count_up | ctrctl_repeat | ctrctl_enable;
  }

  [[nodiscard]] std::uint32_t period_ticks() const
  {
    return m_period_ticks;
  }

private:
  /// The frequency actually produced (integer division can round it).
  std::uint32_t driver_frequency() override
  {
    return m_actual_frequency;
  }

  /// @param p_duty_cycle - 0 (always low) to 65535 (always high)
  void driver_duty_cycle(std::uint16_t p_duty_cycle) override
  {
    constexpr std::uint32_t full_scale =
      std::numeric_limits<std::uint16_t>::max();

    // period <= 65535 and duty <= 65535, so the product fits in 32 bits.
    // Adding half of full_scale rounds to the nearest tick.
    std::uint32_t const high_ticks =
      (m_period_ticks * p_duty_cycle + full_scale / 2) / full_scale;

    if (high_ticks == 0) {
      // CC = 0 would collide with the "set high at zero" action, so drop
      // that action and the output stays low.
      reg_at(reg::cc0, channel()) = 0;
      reg_at(reg::ccact0, channel()) = ccact_compare_up_set_low;
      return;
    }

    // high_ticks == period: the counter never reaches CC (it tops out at
    // LOAD = period - 1), so the output simply stays high.
    reg_at(reg::cc0, channel()) = high_ticks;
    reg_at(reg::ccact0, channel()) =
      ccact_zero_set_high | ccact_compare_up_set_low;
  }

  [[nodiscard]] std::uint8_t channel() const
  {
    return to_underlying(m_pin.channel);
  }

  [[nodiscard]] std::uint32_t channel_bit() const
  {
    return 1u << channel();
  }

  std::uint32_t volatile& reg_at(timer_reg p_reg, std::uint8_t p_index = 0)
  {
    return timer_register(m_pin.peripheral, p_reg, p_index);
  }

  pwm_pin m_pin;
  std::uint32_t m_period_ticks = 0;
  std::uint32_t m_actual_frequency = 0;
};

// ============================================================================
// Color wheel
// ============================================================================

constexpr std::uint32_t pwm_frequency_hz = 1'000;  // well above visible flicker
constexpr std::uint32_t step_delay_ms = 5;  // 628 steps * 5 ms = ~3.1 s/cycle

/**
 * @brief Brightness for one channel at a position in the cosine table,
 * 0 to 65535.
 *
 * Squaring the value (gamma 2.0) makes dim levels darker, which matches how
 * the eye perceives brightness and gives more saturated colors.
 */
std::uint16_t channel_level(std::size_t p_phase)
{
  constexpr std::uint32_t full_scale =
    std::numeric_limits<std::uint16_t>::max();
  std::uint32_t const value = uint16_cosine[p_phase % uint16_cosine.size()];
  return static_cast<std::uint16_t>((value * value) / full_scale);
}

int main()
{
  mspm0_steady_clock clock;

  mspm0_pwm red(mspm0_pwm::pb26_timer_g6_channel_0, pwm_frequency_hz);
  mspm0_pwm green(mspm0_pwm::pb27_timer_g6_channel_1, pwm_frequency_hz);
  mspm0_pwm blue(mspm0_pwm::pb22_timer_g8_channel_1, pwm_frequency_hz);

  std::printf("Steady clock: %u Hz\n",
              static_cast<unsigned>(clock.frequency()));
  std::printf("PWM: %u Hz, period = %u ticks\n",
              static_cast<unsigned>(red.frequency()),
              static_cast<unsigned>(red.period_ticks()));

  // Each channel runs the same wave, offset by a third of a cycle (120 deg).
  constexpr std::size_t third = uint16_cosine.size() / 3;
  std::size_t phase = 0;

  while (true) {
    red.duty_cycle(channel_level(phase));
    green.duty_cycle(channel_level(phase + third));
    blue.duty_cycle(channel_level(phase + 2 * third));

    phase = (phase + 1) % uint16_cosine.size();
    delay_ms(clock, step_delay_ms);
  }
}