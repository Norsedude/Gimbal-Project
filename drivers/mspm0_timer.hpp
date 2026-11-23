#pragma once

// MSPM0G3507 timer drivers, implementing the lab2::steady_clock and lab2::pwm
// interfaces (hal/timer.hpp) directly against registers. This file mirrors
// hal/timer.hpp: the two timer-backed interfaces live together because they
// share the same peripheral register map.
//
// Carried over from lab 2, with the PWM channel retargeted from the on-board
// RGB LED to a BoosterPack header pin so a hobby servo can be driven from it.
//
// Register names and bit positions below are from the MSPM0G350x TRM,
// chapter 35 (Timers).

#include <cstdint>
#include <limits>
#include <type_traits>

#include <ti/driverlib/driverlib.h>

#include "../hal/timer.hpp"

namespace mspm0 {

// ============================================================================
// Shared helpers
// ============================================================================

/**
 * @brief Converts an `enum class` into its underlying integral value
 */
template<typename Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum p_value) noexcept
{
  static_assert(std::is_enum_v<Enum>, "to_underlying requires an enum type");
  return static_cast<std::underlying_type_t<Enum>>(p_value);
}

// PWREN register: a write must carry the unlock KEY in bits 31:24 or the
// write is ignored. ENABLE is bit 0.
constexpr std::uint32_t pwren_key = 0x26U << 24;
constexpr std::uint32_t pwren_enable = 1U << 0;
constexpr std::uint32_t timer_power_on_value = pwren_key | pwren_enable;

/// Peripherals need a few cycles after PWREN before their registers respond.
inline void timer_power_up_delay()
{
  delay_cycles(16);
}

constexpr std::uintptr_t timer_pin_mux_address = 0x40428000;
constexpr std::uintptr_t timer_gpioa_address = 0x400A0000;
constexpr std::uintptr_t timer_gpiob_address = 0x400A2000;
constexpr std::uintptr_t timer_gpio_pwren_offset = 0x800;

/// PINCM.PC - connect the pin to the selected peripheral function.
constexpr std::uint32_t timer_peripheral_connected = 1U << 7;

/// PINCMn is at IOMUX base + 4 * n
inline std::uint32_t volatile& timer_pin_mux_register(std::uint8_t p_pincm)
{
  return *reinterpret_cast<std::uint32_t volatile*>(
    timer_pin_mux_address + p_pincm * sizeof(std::uint32_t));
}

inline void timer_power_gpio_port(char p_port)
{
  auto const base =
    (p_port == 'A') ? timer_gpioa_address : timer_gpiob_address;
  *reinterpret_cast<std::uint32_t volatile*>(base + timer_gpio_pwren_offset) =
    timer_power_on_value;
  timer_power_up_delay();
}

// ============================================================================
// Timer register map
// ============================================================================

/// Capture/compare channels used by this project.
enum class timer_channel : std::uint8_t
{
  channel_0 = 0,
  channel_1,
};

/// Base address of each timer peripheral this project uses.
enum class timer_peripheral : std::uintptr_t
{
  /// 16-bit general purpose timer - drives the servo PWM.
  timer_g8 = 0x40090000UL,
  /// The only 32-bit general purpose timer - drives the steady clock.
  timer_g12 = 0x40870000UL,
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
constexpr std::uint32_t busclk_hz = 32000000;

constexpr std::uint32_t clksel_busclk = 1U << 3;    // CLKSEL.BUSCLK_SEL
constexpr std::uint32_t cclkctl_clken = 1U << 0;    // CCLKCTL.CLKEN
constexpr std::uint32_t ctrctl_enable = 1U << 0;    // CTRCTL.EN
constexpr std::uint32_t ctrctl_repeat = 1U << 1;    // CTRCTL.REPEAT
constexpr std::uint32_t ctrctl_count_up = 2U << 4;  // CTRCTL.CM = up
// (CTRCTL.CM = 0, the default, counts down.)

// CCACT field actions: 1 = set output high, 2 = set output low.
constexpr std::uint32_t ccact_zero_set_high = 1U << 0;       // CCACT.ZACT
constexpr std::uint32_t ccact_compare_up_set_low = 2U << 9;  // CCACT.CUACT

// ============================================================================
// Steady clock
// ============================================================================

/**
 * @brief Free-running time base built on a 32-bit general purpose timer.
 *
 * TIMCLK = BUSCLK / (CLKDIV.RATIO + 1). The counter counts down from
 * 0xFFFFFFFF, so the elapsed tick count is simply ~CTR. Each time that
 * 32-bit value wraps, an overflow counter extends it to 64 bits.
 *
 * uptime() mutates that overflow state, so it runs inside a critical
 * section and is safe to call from more than one task.
 */
class steady_clock final : public lab2::steady_clock
{
public:
  /**
   * @param p_timer - which timer to use; needs a 32-bit counter (TIMG12)
   * @param p_divider - clock divider, 1 to 8. The default 8 gives 4 MHz,
   * which wraps the 32-bit counter about every 18 minutes.
   */
  explicit steady_clock(timer_peripheral p_timer = timer_peripheral::timer_g12,
                        std::uint32_t p_divider = 8)
    : m_timer(p_timer)
    , m_frequency(busclk_hz / p_divider)
  {
    reg(timer_reg::pwren) = timer_power_on_value;
    timer_power_up_delay();

    reg(timer_reg::clksel) = clksel_busclk;
    reg(timer_reg::clkdiv) = p_divider - 1;  // RATIO: 0 means divide by 1
    reg(timer_reg::cclkctl) = cclkctl_clken;

    reg(timer_reg::ctrctl) = 0;  // stop while configuring
    reg(timer_reg::load) = 0xFFFFFFFF;
    reg(timer_reg::ctrctl) = ctrctl_repeat | ctrctl_enable;
  }

private:
  std::uint32_t driver_frequency() override
  {
    return m_frequency;
  }

  std::uint64_t driver_uptime() override
  {
    auto const saved_interrupt_state = __get_PRIMASK();
    __disable_irq();

    // Counting down from all 1s, so ~CTR counts up from 0.
    std::uint32_t const ticks = ~reg(timer_reg::ctr);
    if (ticks < m_last_ticks) {
      m_overflows++;  // the 32-bit counter wrapped since the last call
    }
    m_last_ticks = ticks;
    auto const result = (static_cast<std::uint64_t>(m_overflows) << 32) | ticks;

    __set_PRIMASK(saved_interrupt_state);
    return result;
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
 *
 * Busy-waits, so it is meant for start-up sequencing before the RTOS
 * scheduler is running. Once tasks are running, use vTaskDelay() instead so
 * the CPU can be yielded.
 */
inline void delay_ms(lab2::steady_clock& p_clock, std::uint32_t p_milliseconds)
{
  auto const ticks_per_ms =
    static_cast<std::uint64_t>(p_clock.frequency()) / 1000;
  auto const wait_ticks = ticks_per_ms * p_milliseconds;
  auto const start = p_clock.uptime();
  while (p_clock.uptime() - start < wait_ticks) {
  }
}

// ============================================================================
// PWM
// ============================================================================

/// One timer capture/compare channel routed to a physical pin.
struct pwm_pin
{
  timer_peripheral peripheral;
  timer_channel channel;
  char port;
  std::uint8_t mux_index;  // pin function number for the timer output
  std::uint8_t pincm;      // IOMUX slot for the physical pin
};

/**
 * @brief Edge-aligned, up-counting PWM on one timer channel.
 *
 * Frequency math:
 *   TIMCLK       = BUSCLK / (CPS.PCNT + 1)          (CLKDIV left at /1)
 *   period ticks = TIMCLK / frequency               -> LOAD = ticks - 1
 *   high ticks   = period ticks * duty cycle        -> CC
 * The prescaler is the smallest value that makes the period fit in the
 * 16-bit counter, which keeps the duty-cycle resolution as fine as possible.
 *
 * Output: set HIGH when the counter wraps to 0, set LOW when it counts up to
 * CC.
 *
 * At 50 Hz this yields prescaler 10 and 64000 ticks per period, so one timer
 * tick is 0.3125 us - far finer than a hobby servo can resolve.
 */
class pwm final : public lab2::pwm
{
public:
  using reg = timer_reg;

  /// Servo output: TIMG8_CCP0 on PB15 [pin function 5], BoosterPack pin 17.
  /// TIMG8 is a 16-bit timer, which leaves the only 32-bit timer (TIMG12)
  /// free for mspm0::steady_clock.
  static constexpr pwm_pin pb15_timer_g8_channel_0 = {
    .peripheral = timer_peripheral::timer_g8,
    .channel = timer_channel::channel_0,
    .port = 'B',
    .mux_index = 5,
    .pincm = 32,
  };

  static constexpr std::uint32_t max_period_ticks = 0xFFFF;  // 16-bit counter
  static constexpr std::uint32_t max_prescaler = 256;        // 8-bit CPS

  /**
   * @param p_pin - timer channel and pin to drive
   * @param p_frequency_hz - PWM frequency. Must be between about 2 Hz and
   * BUSCLK / 2 for the prescaler and period to be representable.
   */
  pwm(pwm_pin const& p_pin, std::uint32_t p_frequency_hz)
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
    reg_at(reg::pwren) = timer_power_on_value;
    timer_power_up_delay();
    reg_at(reg::clksel) = clksel_busclk;
    reg_at(reg::clkdiv) = 0;           // divide by 1
    reg_at(reg::cps) = prescaler - 1;  // PCNT: 0 means divide by 1
    reg_at(reg::cclkctl) = cclkctl_clken;

    // 3. Counter period and channel setup (timer stopped meanwhile)
    reg_at(reg::ctrctl) = 0;
    reg_at(reg::load) = m_period_ticks - 1;
    reg_at(reg::ccctl0, channel()) = 0;   // compare mode
    reg_at(reg::octl0, channel()) = 0;    // pin follows the CCACT actions
    driver_duty_cycle(0);                 // start with the output low
    reg_at(reg::ccpd) |= channel_bit();   // channel is an output
    reg_at(reg::odis) &= ~channel_bit();  // do not force the output low

    // 4. Route the timer output to the pin
    timer_power_gpio_port(m_pin.port);
    timer_pin_mux_register(m_pin.pincm) =
      m_pin.mux_index | timer_peripheral_connected;

    // 5. Start: count up 0 -> LOAD, repeat forever
    reg_at(reg::ctrctl) = ctrctl_count_up | ctrctl_repeat | ctrctl_enable;
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
    return 1U << channel();
  }

  std::uint32_t volatile& reg_at(timer_reg p_reg, std::uint8_t p_index = 0)
  {
    return timer_register(m_pin.peripheral, p_reg, p_index);
  }

  pwm_pin m_pin;
  std::uint32_t m_period_ticks = 0;
  std::uint32_t m_actual_frequency = 0;
};

}  // namespace mspm0
