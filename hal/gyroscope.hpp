#pragma once

namespace lab4 {

/**
 * @brief Angular velocity sensing hardware abstraction interface.
 *
 * Mirrors the shape of `lab4::accelerometer`. A chip that senses both
 * acceleration and angular velocity (such as the BMI160) exposes one object
 * per interface rather than inheriting both, because the two interfaces
 * would otherwise declare `driver_read()` with differing return types, which
 * C++ cannot overload.
 *
 */
class gyroscope
{
public:
  /**
   * @brief Result from reading the gyroscope.
   *
   */
  struct read_t
  {
    /// Angular velocity about the X axis, relative to the device's reference
    /// frame, in degrees per second. Positive follows the right-hand rule.
    float x;
    /// Angular velocity about the Y axis, relative to the device's reference
    /// frame, in degrees per second. Positive follows the right-hand rule.
    float y;
    /// Angular velocity about the Z axis, relative to the device's reference
    /// frame, in degrees per second. Positive follows the right-hand rule.
    float z;
  };

  /**
   * @brief Read the latest angular velocity sensed by the device
   *
   * @return read_t - angular velocity data
   */
  [[nodiscard]] read_t read()
  {
    return driver_read();
  }

private:
  virtual read_t driver_read() = 0;
};
}  // namespace lab4
