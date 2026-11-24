#pragma once

// Transmit-only telemetry UART on the LP-MSPM0G3507 XDS110 backchannel.
//
// This is a bring-up and demonstration aid, not one of the project control
// drivers: it exists so the tilt estimate, joystick offset, and servo command
// can be watched live on a serial terminal while tuning, and during the demo.
// Nothing in the control path depends on it.
//
// UART0 on PA10 (TX) / PA11 (RX), the pins wired to the on-board debug probe,
// which presents them as a USB virtual COM port. 115200 8N1.
//
// Transmit only, so there is no interrupt handler and no receive buffer,
// which keeps it entirely out of the way of the control tasks. Writes spin
// while the TX FIFO is full; with no flow control the FIFO always drains, so
// this cannot wedge when nothing is listening.
//
// Register names, offsets, and bit positions are from the MSPM0G350x TRM,
// chapter 21 (UART).

#include <cstddef>
#include <cstdint>

namespace mspm0 {

/**
 * @brief Blocking, transmit-only UART0 driver for telemetry output.
 *
 * The constructor performs the complete hardware setup - power the
 * peripheral, mux the pins, set the baud rate, enable the transmitter - so
 * the port is usable the moment the object exists.
 */
class telemetry_uart
{
public:
  explicit telemetry_uart(std::uint32_t p_baud_rate = 115200)
  {
    configure(p_baud_rate);
  }

  /// Write a null-terminated string, blocking until the last byte has been
  /// handed to the TX FIFO.
  void write(char const* p_text)
  {
    if (p_text == nullptr) {
      return;
    }

    for (auto const* cursor = p_text; *cursor != '\0'; cursor++) {
      while (reg(uart_stat) & stat_tx_fifo_full) {
      }
      // Via unsigned char: a plain char is signed here, so a byte above
      // 0x7F would otherwise sign-extend into the upper bits.
      reg(uart_txdata) = static_cast<std::uint8_t>(*cursor);
    }
  }

private:
  static constexpr std::uintptr_t uart0_base = 0x40108000;
  static constexpr std::uintptr_t iomux_base = 0x40428000;
  static constexpr std::uintptr_t gpioa_base = 0x400A0000;

  static constexpr std::size_t uart_pwren = 0x800;
  static constexpr std::size_t uart_clksel = 0x1008;
  static constexpr std::size_t uart_ctl0 = 0x1100;
  static constexpr std::size_t uart_lcrh = 0x1104;
  static constexpr std::size_t uart_stat = 0x1108;
  static constexpr std::size_t uart_ibrd = 0x1110;
  static constexpr std::size_t uart_fbrd = 0x1114;
  static constexpr std::size_t uart_txdata = 0x1120;

  // PWREN: unlock KEY in bits 31:24, ENABLE in bit 0.
  static constexpr std::uint32_t pwren_key = 0x26U << 24;
  static constexpr std::uint32_t pwren_enable = 1U << 0;
  static constexpr std::uint32_t power_on_value = pwren_key | pwren_enable;

  static constexpr std::uint32_t peripheral_connected = 1U << 7;  // PINCM.PC
  static constexpr std::uint32_t uart0_pin_function = 2U << 0;    // PINCM.PF

  static constexpr std::uint32_t clksel_busclk = 1U << 3;  // CLKSEL.BUSCLK_SEL
  static constexpr std::uint32_t lcrh_word_length_8 = 3U << 4;  // LCRH.WLEN
  static constexpr std::uint32_t ctl0_fifo_enable = 1U << 17;   // CTL0.FEN
  static constexpr std::uint32_t ctl0_enable = 1U << 0;         // CTL0.ENABLE
  static constexpr std::uint32_t ctl0_transmit_enable = 1U << 4;  // CTL0.TXE
  static constexpr std::uint32_t stat_tx_fifo_full = 1U << 7;     // STAT.TXFF

  /// PA10 = IOMUX slot 21 (TX), PA11 = IOMUX slot 22 (RX).
  static constexpr unsigned tx_pincm = 21;
  static constexpr unsigned rx_pincm = 22;

  static constexpr std::uint32_t bus_clock_hz = 32000000;

  static std::uint32_t volatile& reg(std::size_t p_offset)
  {
    return *reinterpret_cast<std::uint32_t volatile*>(uart0_base + p_offset);
  }

  static std::uint32_t volatile& pin_mux(unsigned p_pincm)
  {
    return *reinterpret_cast<std::uint32_t volatile*>(
      iomux_base + (p_pincm * sizeof(std::uint32_t)));
  }

  bool configure(std::uint32_t p_baud_rate)
  {
    if (p_baud_rate == 0) {
      return false;
    }

    // Baud divisor is a 16.6 fixed-point value: the integer part goes to
    // IBRD and the six fractional bits to FBRD.
    //   divisor = round(64 * BUSCLK / (16 * baud))
    std::uint32_t const divisor = ((bus_clock_hz * 8) / p_baud_rate + 1) / 2;
    constexpr std::uint32_t max_divisor = (0xFFFFU << 6) | 0x3FU;
    if (divisor < 64 || divisor > max_divisor) {
      return false;
    }

    // Power on UART0 and GPIOA. GPIOA is only powered, never reset, because
    // port A also carries the SWD debug pins.
    reg(uart_pwren) = power_on_value;
    *reinterpret_cast<std::uint32_t volatile*>(gpioa_base + uart_pwren) =
      power_on_value;

    pin_mux(tx_pincm) = uart0_pin_function | peripheral_connected;
    pin_mux(rx_pincm) = uart0_pin_function | peripheral_connected;

    reg(uart_clksel) = clksel_busclk;

    reg(uart_ibrd) = divisor >> 6;
    reg(uart_fbrd) = divisor & 0x3F;

    // 8 data bits, no parity, 1 stop bit.
    reg(uart_lcrh) = lcrh_word_length_8;

    // The FIFO depth must be set before the port is enabled.
    reg(uart_ctl0) = ctl0_fifo_enable;
    reg(uart_ctl0) = ctl0_fifo_enable | ctl0_enable | ctl0_transmit_enable;

    return true;
  }
};

/// Decimal places printed by append_float.
constexpr std::uint32_t telemetry_decimals = 2;
constexpr std::uint32_t telemetry_scale = 100;  // 10^telemetry_decimals

/**
 * @brief Append a float to a character buffer with two decimal places.
 *
 * Written by hand rather than with snprintf because the "%f" conversion on
 * this toolchain pulls in a large and slow formatting path, and because
 * std::printf here would route through debugger semihosting, which blocks
 * when no debugger is attached.
 *
 * @param p_buffer - destination buffer
 * @param p_capacity - total size of p_buffer
 * @param p_at - current write offset, advanced past the characters written
 * @param p_value - value to format
 */
inline void append_float(char* p_buffer,
                         std::size_t p_capacity,
                         std::size_t& p_at,
                         float p_value)
{
  auto append = [&](char p_character) {
    if (p_at + 1 < p_capacity) {
      p_buffer[p_at++] = p_character;
    }
  };

  if (p_value < 0.0F) {
    append('-');
    p_value = -p_value;
  }

  // Scale by 100 and round, so the whole and fractional parts can both be
  // emitted with integer arithmetic.
  auto const scaled = static_cast<std::uint32_t>(
    (p_value * static_cast<float>(telemetry_scale)) + 0.5F);
  auto const whole = scaled / telemetry_scale;
  auto const fraction = scaled % telemetry_scale;

  // Digits come out least-significant first, so collect then emit reversed.
  char digits[12];
  std::size_t digit_count = 0;
  auto remaining = whole;
  do {
    digits[digit_count++] = static_cast<char>('0' + (remaining % 10));
    remaining /= 10;
  } while (remaining != 0 && digit_count < sizeof(digits));

  while (digit_count > 0) {
    append(digits[--digit_count]);
  }

  append('.');
  append(static_cast<char>('0' + ((fraction / 10) % 10)));
  append(static_cast<char>('0' + (fraction % 10)));

  p_buffer[(p_at < p_capacity) ? p_at : p_capacity - 1] = '\0';
}

/**
 * @brief Append an unsigned integer to a character buffer.
 *
 * Same hand-rolled approach as append_float and for the same reason: the
 * library formatting path on this toolchain is large, and std::printf would
 * route through debugger semihosting, which blocks with no debugger attached.
 *
 * @param p_buffer - destination buffer
 * @param p_capacity - total size of p_buffer
 * @param p_at - current write offset, advanced past the characters written
 * @param p_value - value to format
 */
inline void append_unsigned(char* p_buffer,
                            std::size_t p_capacity,
                            std::size_t& p_at,
                            std::uint32_t p_value)
{
  // Digits come out least-significant first, so collect then emit reversed.
  char digits[12];
  std::size_t digit_count = 0;
  auto remaining = p_value;
  do {
    digits[digit_count++] = static_cast<char>('0' + (remaining % 10));
    remaining /= 10;
  } while (remaining != 0 && digit_count < sizeof(digits));

  while (digit_count > 0) {
    if (p_at + 1 < p_capacity) {
      p_buffer[p_at++] = digits[--digit_count];
    } else {
      digit_count--;
    }
  }

  p_buffer[(p_at < p_capacity) ? p_at : p_capacity - 1] = '\0';
}

/// Append a null-terminated string, advancing the write offset.
inline void append_text(char* p_buffer,
                        std::size_t p_capacity,
                        std::size_t& p_at,
                        char const* p_text)
{
  for (auto const* cursor = p_text; *cursor != '\0'; cursor++) {
    if (p_at + 1 < p_capacity) {
      p_buffer[p_at++] = *cursor;
    }
  }
  p_buffer[(p_at < p_capacity) ? p_at : p_capacity - 1] = '\0';
}

}  // namespace mspm0
