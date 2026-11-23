#pragma once

// Hobby servo (SG90 / MG90S class) driver.
//
// This is a portable device driver: it is written entirely against the
// lab2::pwm class interface and contains no microcontroller register access,
// no DriverLib calls, and no RTOS calls. Driving a servo from a different
// microcontroller means supplying a different lab2::pwm implementation -
// nothing in this file changes.
//
// A hobby servo is commanded by the HIGH time of a repeating pulse, not by
// duty cycle as such. The frame is normally 20 ms (50 Hz), and the HIGH time
// selects the shaft angle: roughly 1000 us at one travel limit, 1500 us at
// centre, 2000 us at the other limit. This driver converts an angle in
// degrees into that pulse width, then into the 16-bit duty-cycle value the
// lab2::pwm interface takes.

#include <cstdint>

#include "../hal/timer.hpp"

namespace actuator {

/**
 * @brief Angle-controlled hobby servo driven by a PWM channel.
 */
class servo
{
public:
  struct settings
  {
    /// Angle reported at min_pulse_microseconds.
    float min_angle = -90.0F;
    /// Angle reported at max_pulse_microseconds.
    float max_angle = 90.0F;
    /// Pulse width at min_angle. Widen towards 500 us only after checking
    /// that the specific servo does not buzz or stall at the end stop.
    std::uint32_t min_pulse_microseconds = 1000;
    /// Pulse width at max_angle.
    std::uint32_t max_pulse_microseconds = 2000;
  };

  /**
   * @param p_pwm - PWM channel wired to the servo signal lead. Its frequency
   * should already be set to about 50 Hz.
   * @param p_settings - angle and pulse-width calibration
   */
  servo(lab2::pwm& p_pwm, settings const& p_settings)
    : m_pwm(p_pwm)
    , m_settings(p_settings)
    , m_period_microseconds(1000000U / p_pwm.frequency())
  {
    // Park at the centre of the configured travel so the servo has a defined
    // position from the moment it is powered.
    angle((p_settings.min_angle + p_settings.max_angle) / 2.0F);
  }

  /**
   * @brief Construct with the default SG90-class calibration.
   *
   * Defined after the class body because a `= {}` default argument cannot
   * see the default member initializers of the nested `settings` struct
   * while the enclosing class is still being defined.
   */
  explicit servo(lab2::pwm& p_pwm);

  /**
   * @brief Command the servo to an angle, in degrees.
   *
   * Angles outside the configured travel are clamped rather than rejected,
   * so a control loop that overshoots simply holds the end stop instead of
   * commanding a pulse width the servo cannot accept.
   *
   * @param p_degrees - requested angle
   */
  void angle(float p_degrees)
  {
    if (p_degrees < m_settings.min_angle) {
      p_degrees = m_settings.min_angle;
    } else if (p_degrees > m_settings.max_angle) {
      p_degrees = m_settings.max_angle;
    }
    m_angle = p_degrees;

    auto const span = m_settings.max_angle - m_settings.min_angle;
    auto const travelled =
      (span == 0.0F) ? 0.0F : (p_degrees - m_settings.min_angle) / span;

    auto const pulse_span =
      static_cast<float>(m_settings.max_pulse_microseconds) -
      static_cast<float>(m_settings.min_pulse_microseconds);
    auto const pulse_microseconds =
      static_cast<float>(m_settings.min_pulse_microseconds) +
      travelled * pulse_span;

    m_pwm.duty_cycle(to_duty_cycle(pulse_microseconds));
  }

  /// The angle last commanded, after clamping.
  [[nodiscard]] float angle() const
  {
    return m_angle;
  }

private:
  /// Converts a pulse width into the proportional 16-bit value that
  /// lab2::pwm::duty_cycle() expects.
  [[nodiscard]] std::uint16_t to_duty_cycle(float p_pulse_microseconds) const
  {
    constexpr float full_scale = 65535.0F;

    if (p_pulse_microseconds <= 0.0F || m_period_microseconds == 0) {
      return 0;
    }

    auto const proportion =
      p_pulse_microseconds / static_cast<float>(m_period_microseconds);
    if (proportion >= 1.0F) {
      return 65535;
    }

    return static_cast<std::uint16_t>(proportion * full_scale + 0.5F);
  }

  lab2::pwm& m_pwm;
  settings m_settings;
  std::uint32_t m_period_microseconds;
  float m_angle = 0.0F;
};

inline servo::servo(lab2::pwm& p_pwm)
  : servo(p_pwm, settings{})
{
}

}  // namespace actuator
