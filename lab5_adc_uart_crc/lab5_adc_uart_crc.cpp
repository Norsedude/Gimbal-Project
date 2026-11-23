// cspell:ignore VDDA IIDX DDEVICE

#include <array>
#include <cstdint>
#include <cstdio>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <ti/driverlib/driverlib.h>

#include "../hal/adc.hpp"
#include "../hal/crc.hpp"
#include "../hal/gpio.hpp"
#include "../hal/serial.hpp"
#include "../hal/timer.hpp"
#include "../drivers/mspm0_gpio.hpp"

// =============================================================================
// Role 1: ADC
// =============================================================================

class adc_driver : public lab5::adc16_stream
{
public:
  adc_driver()
  {
    DL_ADC12_reset(ADC0);
    DL_ADC12_enablePower(ADC0);
    delay_cycles(32);

    // The internal temperature sensor is ADC0 channel 11. A 4 MHz sample
    // clock and 50 us acquisition window satisfy its settling requirement.
    DL_ADC12_ClockConfig const clock_config = {
      DL_ADC12_CLOCK_SYSOSC,
      DL_ADC12_CLOCK_FREQ_RANGE_1_TO_4,
      DL_ADC12_CLOCK_DIVIDE_8,
    };
    DL_ADC12_setClockConfig(ADC0, &clock_config);
    DL_ADC12_initSingleSample(ADC0,
                              DL_ADC12_REPEAT_MODE_ENABLED,
                              DL_ADC12_SAMPLING_SOURCE_AUTO,
                              DL_ADC12_TRIG_SRC_SOFTWARE,
                              DL_ADC12_SAMP_CONV_RES_12_BIT,
                              DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC0,
                                 DL_ADC12_MEM_IDX_0,
                                 DL_ADC12_INPUT_CHAN_11,
                                 DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                                 DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                                 DL_ADC12_AVERAGING_MODE_DISABLED,
                                 DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                                 DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                                 DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(ADC0, 199);
    DL_ADC12_setDMASamplesCnt(ADC0, 1);
    DL_ADC12_enableDMATrigger(ADC0, DL_ADC12_DMA_MEM0_RESULT_LOADED);
    DL_ADC12_clearInterruptStatus(ADC0, DL_ADC12_INTERRUPT_DMA_DONE);
    DL_ADC12_enableInterrupt(ADC0, DL_ADC12_INTERRUPT_DMA_DONE);
    DL_ADC12_enableConversions(ADC0);

    DL_DMA_Config const dma_config = {
      DMA_ADC0_EVT_GEN_BD_TRIG,
      DL_DMA_TRIGGER_TYPE_EXTERNAL,
      DL_DMA_SINGLE_TRANSFER_MODE,
      DL_DMA_NORMAL_MODE,
      DL_DMA_WIDTH_HALF_WORD,
      DL_DMA_WIDTH_HALF_WORD,
      DL_DMA_ADDR_UNCHANGED,
      DL_DMA_ADDR_INCREMENT,
    };
    DL_DMA_initChannel(DMA, dma_channel, &dma_config);
    DL_DMA_setSrcAddr(
      DMA, dma_channel, DL_ADC12_getMemResultAddress(ADC0, DL_ADC12_MEM_IDX_0));

    NVIC_SetPriority(ADC0_INT_IRQn,
                     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(ADC0_INT_IRQn);
  }

  // Converts one of this driver's own upscaled 16-bit samples back to a
  // temperature. Formula and constants (coefficient, trim temperature, and
  // using the same VDDA reference for both the live sample and the factory
  // trim constant) match the SDK's adc12_internal_temp_sensor_mathacl
  // example, which the README cites as the reference for this calculation.
  [[nodiscard]] static float temperature_celsius(
    std::uint16_t p_normalized_sample)
  {
    constexpr float vdda = 3.3F;
    constexpr float adc_codes = 4096.0F;
    constexpr float calibration_temperature = 30.0F;
    constexpr float inverse_sensor_slope = -555.55F;

    auto const raw_sample = static_cast<std::uint32_t>(p_normalized_sample) >> 4;
    auto const trim_sample = DL_SYSCTL_getTempCalibrationConstant();
    auto const sample_voltage =
      vdda * (static_cast<float>(raw_sample) - 0.5F) / adc_codes;
    auto const trim_voltage =
      vdda * (static_cast<float>(trim_sample) - 0.5F) / adc_codes;
    return calibration_temperature +
           inverse_sensor_slope * (sample_voltage - trim_voltage);
  }

  static void interrupt()
  {
    if (DL_ADC12_getPendingInterrupt(ADC0) != DL_ADC12_IIDX_DMA_DONE) {
      return;
    }

    auto const waiting_task = s_waiting_task;
    if (waiting_task != nullptr) {
      BaseType_t higher_priority_task_woken = pdFALSE;
      vTaskNotifyGiveFromISR(waiting_task, &higher_priority_task_woken);
      portYIELD_FROM_ISR(higher_priority_task_woken);
    }
  }

private:
  void driver_read(std::span<std::uint16_t> p_buffer) override
  {
    if (p_buffer.empty()) {
      return;
    }
    configASSERT(p_buffer.size() <= 0xFFFFU);
    configASSERT(s_waiting_task == nullptr);

    (void) ulTaskNotifyTake(pdTRUE, 0);
    s_waiting_task = xTaskGetCurrentTaskHandle();

    DL_DMA_disableChannel(DMA, dma_channel);
    DL_DMA_setDestAddr(DMA,
                       dma_channel,
                       reinterpret_cast<std::uintptr_t>(p_buffer.data()));
    DL_DMA_setTransferSize(DMA, dma_channel, p_buffer.size());
    DL_DMA_enableChannel(DMA, dma_channel);
    DL_ADC12_clearInterruptStatus(ADC0, DL_ADC12_INTERRUPT_DMA_DONE);
    DL_ADC12_enableDMA(ADC0);
    DL_ADC12_startConversion(ADC0);

    (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    DL_ADC12_stopConversion(ADC0);
    DL_DMA_disableChannel(DMA, dma_channel);
    s_waiting_task = nullptr;

    // adc16_stream requires a full-range 16-bit result. Bit duplication maps
    // 0x000 -> 0x0000 and 0xFFF -> 0xFFFF without division.
    for (auto& sample : p_buffer) {
      sample = static_cast<std::uint16_t>((sample << 4) | (sample >> 8));
    }
  }

  static constexpr std::uint8_t dma_channel = 0;
  inline static TaskHandle_t volatile s_waiting_task = nullptr;
};

extern "C" void ADC0_IRQHandler()
{
  adc_driver::interrupt();
}

// =============================================================================
// Role 2: UART
// =============================================================================

// UART1 on PA8 (TX) / PA9 (RX) - the BoosterPack header's own labeled
// LP_UART_TX/LP_UART_RX pins (header pins 4/3). Avoids the XDS110 backchannel
// (UART0/PA10-11) entirely.
constexpr std::uintptr_t uart1_base = 0x4010'0000;
constexpr std::uintptr_t gpioa_base = 0x400A'0000;
constexpr std::uintptr_t iomux_base = 0x4042'8000;

constexpr std::uint32_t power_on_value = (0x26u << 24) | (1u << 0);  // KEY | ENABLE
constexpr std::uint32_t peripheral_connected = 1u << 7;
constexpr std::uint32_t uart1_pin_function = 2;  // IOMUX function value for UART1 TX/RX
constexpr unsigned pa8_pincm = 19;
constexpr unsigned pa9_pincm = 20;

namespace uart_reg {
constexpr std::size_t pwren = 0x800;
constexpr std::size_t clksel = 0x1008;
constexpr std::size_t int_iidx = 0x1020;
constexpr std::size_t int_imask = 0x1028;
constexpr std::size_t ctl0 = 0x1100;
constexpr std::size_t lcrh = 0x1104;
constexpr std::size_t stat = 0x1108;
constexpr std::size_t ifls = 0x110C;
constexpr std::size_t ibrd = 0x1110;
constexpr std::size_t fbrd = 0x1114;
constexpr std::size_t txdata = 0x1120;
constexpr std::size_t rxdata = 0x1124;
}  // namespace uart_reg

inline std::uint32_t volatile& uart1_reg(std::size_t offset)
{
  return *reinterpret_cast<std::uint32_t volatile*>(uart1_base + offset);
}

inline std::uint32_t volatile& pin_mux(unsigned pincm)
{
  return *reinterpret_cast<std::uint32_t volatile*>(
    iomux_base + 4 + 4 * (pincm - 1));
}

class serial_driver : public lab6::serial
{
public:
  serial_driver()
  {
    g_active_serial_driver = this;
    driver_configure({});
  }

  // Called from UART1_IRQHandler - not part of the lab6::serial interface.
  static void on_interrupt()
  {
    if (g_active_serial_driver != nullptr) {
      g_active_serial_driver->handle_interrupt();
    }
  }

private:
  void handle_interrupt()
  {
    constexpr std::uint32_t rx_code = 0x0B;
    constexpr std::uint32_t rxfe_mask = 1u << 2;

    if (uart1_reg(uart_reg::int_iidx) != rx_code) {
      return;
    }

    // Drain every byte currently available - more than one may have
    // arrived before this handler ran.
    while (!(uart1_reg(uart_reg::stat) & rxfe_mask)) {
      m_buffer[m_head] = static_cast<std::uint8_t>(uart1_reg(uart_reg::rxdata));
      m_head = (m_head + 1) % m_buffer.size();
    }
  }

  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.baud_rate == 0) {
      return false;
    }

    // Check the 16-bit integer divider before changing any peripheral state.
    constexpr std::uint32_t bus_clock_hz = 32'000'000;
    std::uint32_t const divisor =
      ((bus_clock_hz * 8) / p_settings.baud_rate + 1) / 2;
    if (divisor < 64 || divisor > ((0xFFFFu << 6) | 0x3Fu)) {
      return false;
    }

    // Power on GPIOA and UART1.
    uart1_reg(uart_reg::pwren) = power_on_value;
    *reinterpret_cast<std::uint32_t volatile*>(gpioa_base + uart_reg::pwren) =
      power_on_value;

    // Route PA8/PA9 to UART1.
    pin_mux(pa8_pincm) = uart1_pin_function | peripheral_connected;
    pin_mux(pa9_pincm) = uart1_pin_function | peripheral_connected;

    uart1_reg(uart_reg::clksel) = 1u << 3;  // BUSCLK_SEL

    // Divider = round(64 * BUSCLK / (16 * baud)).
    uart1_reg(uart_reg::ibrd) = divisor >> 6;
    uart1_reg(uart_reg::fbrd) = divisor & 0x3F;

    // 8 data bits, always (this interface only supports 8-bit frames).
    std::uint32_t lcrh = 0b11u << 4;  // WLEN = 8 bits

    if (p_settings.stop == settings::stop_bits::two) {
      lcrh |= 1u << 3;  // STP2
    }

    switch (p_settings.parity) {
      case settings::parity::odd:
        lcrh |= 1u << 1;  // PEN
        break;
      case settings::parity::even:
        lcrh |= (1u << 1) | (1u << 2);  // PEN | EPS
        break;
      case settings::parity::forced1:
        lcrh |= (1u << 1) | (1u << 6);  // PEN | SPS
        break;
      case settings::parity::forced0:
        lcrh |= (1u << 1) | (1u << 2) | (1u << 6);  // PEN | EPS | SPS
        break;
      case settings::parity::none:
      default:
        break;
    }

    uart1_reg(uart_reg::lcrh) = lcrh;

    constexpr std::uint32_t fen = 1u << 17;
    uart1_reg(uart_reg::ctl0) = fen;

    constexpr std::uint32_t rx_fifo_at_least_1 = 0b111u << 4;
    uart1_reg(uart_reg::ifls) = rx_fifo_at_least_1;

    constexpr std::uint32_t rxint = 1u << 10;
    uart1_reg(uart_reg::int_imask) = rxint;

    NVIC_EnableIRQ(UART1_INT_IRQn);

    constexpr std::uint32_t enable = 1u;
    constexpr std::uint32_t rxe = 1u << 3;
    constexpr std::uint32_t txe = 1u << 4;
    uart1_reg(uart_reg::ctl0) = fen | enable | rxe | txe;

    return true;
  }

  void driver_write(std::span<std::uint8_t const> p_data) override
  {
    constexpr std::uint32_t txff_mask = 1u << 7;

    for (auto const byte : p_data) {
      while (uart1_reg(uart_reg::stat) & txff_mask) {
      }
      uart1_reg(uart_reg::txdata) = byte;
    }
  }

  [[nodiscard]] std::span<std::uint8_t const> driver_receive_buffer() override
  {
    return std::span<std::uint8_t const>(m_buffer.data(), m_buffer.size());
  }

  [[nodiscard]] std::size_t driver_receive_cursor() override
  {
    return m_head;
  }

  static inline serial_driver* g_active_serial_driver = nullptr;

  std::array<std::uint8_t, 256> m_buffer{};
  // Written by the UART ISR and read by application tasks.
  volatile std::size_t m_head = 0;
};

// Role 4 can assign the shared sensor-board serial object here before the
// scheduler starts. The fallback in transmit_task keeps Role 3 testable while
// the task architecture is still being assembled.
serial_driver* sensor_serial = nullptr;

extern "C" void UART1_IRQHandler()
{
  serial_driver::on_interrupt();
}

// =============================================================================
// Role 3: CRC
// =============================================================================

/**
 * @brief Free-running TIMG12 clock used to benchmark the two CRC drivers.
 *
 * TIMG12 runs from the 32 MHz BUSCLK divided by eight, producing a 4 MHz
 * monotonic counter. The timer is configured directly because Role 3 is a
 * register-level peripheral exercise and the timer interface is only used for
 * the measurement abstraction.
 */
class mspm0_steady_clock final : public lab2::steady_clock
{
public:
  mspm0_steady_clock()
    : m_frequency(bus_clock_hz / timer_divider)
  {
    register_at(timer_register::power_enable) = power_on_value;
    for (int i = 0; i < 16; ++i) {
      __asm__ volatile("nop");
    }

    register_at(timer_register::clock_select) = bus_clock_select;
    register_at(timer_register::clock_divider) = timer_divider - 1;
    register_at(timer_register::counter_clock_control) = counter_clock_enable;

    register_at(timer_register::counter_control) = 0;
    register_at(timer_register::load) = 0xFFFF'FFFFu;
    register_at(timer_register::counter_control) = counter_repeat | counter_enable;
  }

private:
  enum class timer_register : std::uintptr_t
  {
    power_enable = 0x800,
    clock_divider = 0x1000,
    clock_select = 0x1008,
    counter_clock_control = 0x1108,
    counter_control = 0x1804,
    load = 0x1808,
    counter = 0x1800,
  };

  static constexpr std::uintptr_t timer_base = 0x4087'0000;
  static constexpr std::uint32_t bus_clock_hz = 32'000'000;
  static constexpr std::uint32_t timer_divider = 8;
  static constexpr std::uint32_t power_on_value = 0x2600'0001;
  static constexpr std::uint32_t bus_clock_select = 1u << 3;
  static constexpr std::uint32_t counter_clock_enable = 1u;
  static constexpr std::uint32_t counter_enable = 1u;
  static constexpr std::uint32_t counter_repeat = 1u << 1;

  std::uint32_t volatile& register_at(timer_register p_register)
  {
    return *reinterpret_cast<std::uint32_t volatile*>(
      timer_base + static_cast<std::uintptr_t>(p_register));
  }

  std::uint32_t driver_frequency() override
  {
    return m_frequency;
  }

  std::uint64_t driver_uptime() override
  {
    std::uint32_t const ticks = ~register_at(timer_register::counter);
    if (ticks < m_last_ticks) {
      ++m_overflows;
    }
    m_last_ticks = ticks;
    return (static_cast<std::uint64_t>(m_overflows) << 32) | ticks;
  }

  std::uint32_t m_frequency;
  std::uint32_t m_last_ticks = 0;
  std::uint32_t m_overflows = 0;
};

/**
 * @brief Register-level CRC-32 accelerator driver.
 *
 * The reference implementation below is the reflected IEEE CRC-32 used by
 * zlib: polynomial 0xEDB88320, initial value 0xFFFFFFFF, and final XOR
 * 0xFFFFFFFF. The MSPM0 CRC peripheral uses the normal CRC-32 polynomial, so
 * BITREVERSE_REVERSED makes its byte input and result equivalent to the
 * reflected software algorithm. The peripheral does not apply the final XOR,
 * so checksum() performs that last step in software.
 */
class crc32_hardware final : public lab6::crc32
{
public:
  crc32_hardware()
  {
    CRC->GPRCM.PWREN =
      CRC_PWREN_KEY_UNLOCK_W | CRC_PWREN_ENABLE_ENABLE;

    for (int i = 0; i < 16; ++i) {
      __asm__ volatile("nop");
    }

    CRC->GPRCM.RSTCTL =
      CRC_RSTCTL_KEY_UNLOCK_W |
      CRC_RSTCTL_RESETSTKYCLR_CLR |
      CRC_RSTCTL_RESETASSERT_ASSERT;

    CRC->CRCCTRL =
      CRC_CRCCTRL_POLYSIZE_CRC32 |
      CRC_CRCCTRL_BITREVERSE_REVERSED |
      CRC_CRCCTRL_INPUT_ENDIANNESS_LITTLE_ENDIAN |
      CRC_CRCCTRL_OUTPUT_BYTESWAP_DISABLE;

    driver_reset();
  }

private:
  void driver_reset() override
  {
    CRC->CRCSEED = initial_value;
  }

  void driver_update(std::span<std::uint8_t const> p_data) override
  {
    auto* input = reinterpret_cast<std::uint8_t volatile*>(&CRC->CRCIN);
    for (auto const byte : p_data) {
      *input = byte;
    }
  }

  std::uint32_t driver_checksum() override
  {
    return CRC->CRCOUT ^ final_xor;
  }

  static constexpr std::uint32_t initial_value = 0xFFFF'FFFFu;
  static constexpr std::uint32_t final_xor = 0xFFFF'FFFFu;
};

// Supplied reference implementation - do not modify. This is the standard
// reflected CRC-32 (IEEE 802.3 / zlib) algorithm, computed bit-by-bit in
// software. Your crc32_hardware driver's output must match this for the
// same input, and should be measurably faster.
class crc32_software : public lab6::crc32
{
private:
  void driver_reset() override
  {
    m_crc = initial_value;
  }

  void driver_update(std::span<std::uint8_t const> p_data) override
  {
    for (auto const byte : p_data) {
      m_crc ^= byte;
      for (int bit = 0; bit < 8; bit++) {
        std::uint32_t const mask = -(m_crc & 1U);
        m_crc = (m_crc >> 1) ^ (polynomial & mask);
      }
    }
  }

  std::uint32_t driver_checksum() override
  {
    return m_crc ^ initial_value;
  }

  static constexpr std::uint32_t polynomial = 0xEDB8'8320U;
  static constexpr std::uint32_t initial_value = 0xFFFF'FFFFU;

  std::uint32_t m_crc = initial_value;
};

namespace {
// =============================================================================
// Shared between both boards
// =============================================================================

// The one-byte message the display board sends to ask the sensor board for
// a fresh batch of readings.
constexpr std::uint8_t sample_request_command = 0xA5;

constexpr std::size_t samples_per_batch = 64;

// Each sample is signed tenths of a degree Celsius, sent least-significant
// byte first by both MSPM0 boards. A sample of 253 represents 25.3 degC.
using temperature_sample_t = std::int16_t;

// One batch of converted samples. This is the item type of batch_queue.
using temperature_batch = std::array<temperature_sample_t, samples_per_batch>;

// A packet on the wire: the batch's bytes, followed by the 4-byte CRC-32.
constexpr std::size_t packet_size =
  sizeof(temperature_batch) + sizeof(std::uint32_t);

// =============================================================================
// Sensor board: semaphore and queue (role 4 creates these in main())
// =============================================================================

// request_task -> sampler_task: "take a batch now". Carries no data.
// TODO(lab, role 4): Create with xSemaphoreCreateBinaryStatic().
SemaphoreHandle_t sample_request = nullptr;

// sampler_task -> transmit_task: a copy of one temperature_batch.
// TODO(lab, role 4): Create with xQueueCreateStatic(), depth 1, item size
// sizeof(temperature_batch).
QueueHandle_t batch_queue = nullptr;

// =============================================================================
// Sensor board: tasks
// =============================================================================

// TODO(lab, role 4): Every few ms (vTaskDelay()), compare the serial driver's
// receive_cursor() with the last position you read. When
// sample_request_command arrives, xSemaphoreGive(sample_request). This task
// only reads from the serial driver.
void request_task(void*)
{
  while (true) {
  }
}

void sampler_task(void*)
{
  adc_driver adc;
  std::array<std::uint16_t, samples_per_batch> raw_samples{};
  temperature_batch temperatures{};

  while (true) {
    xSemaphoreTake(sample_request, portMAX_DELAY);
    adc.read(std::span<std::uint16_t>(raw_samples.data(), raw_samples.size()));

    // Transmit signed tenths of a degree Celsius, resolving 0.1 degC.
    for (std::size_t i = 0; i < samples_per_batch; ++i) {
      float const tenths = adc_driver::temperature_celsius(raw_samples[i]) * 10.0F;
      temperatures[i] = static_cast<temperature_sample_t>(
        tenths + (tenths >= 0.0F ? 0.5F : -0.5F));
    }
    xQueueSend(batch_queue, &temperatures, portMAX_DELAY);
  }
}

// Role 3: calculate and benchmark both CRC implementations, optionally
// corrupt the payload, and transmit the batch followed by its CRC. Loop:
//   1. xQueueReceive(batch_queue, &batch, portMAX_DELAY)
//   2. Compute the CRC-32 over the batch's bytes with both crc32_hardware and
//      crc32_software, timing each with a lab2::steady_clock. Print both
//      durations.
//   3. If the button is held, flip one bit in the batch (after the CRC, so
//      the CRC no longer matches).
//   4. Write the batch, then the CRC, with the serial driver. This task only
//      writes to the serial driver.
void transmit_task(void*)
{
  serial_driver* serial = sensor_serial;
  if (serial == nullptr) {
    static serial_driver fallback_serial;
    serial = &fallback_serial;
  }

  crc32_hardware hardware_crc;
  crc32_software software_crc;
  mspm0::input_pin corruption_button(
    mspm0::gpioa,
    18,
    40,
    lab1::pin_resistor::pull_down
  );
  mspm0_steady_clock clock;
  temperature_batch batch{};

  while (true) {
    if (xQueueReceive(batch_queue, &batch, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    auto const bytes = std::span<std::uint8_t const>(
      reinterpret_cast<std::uint8_t const*>(batch.data()),
      sizeof(batch)
    );

    hardware_crc.reset();
    auto const hardware_start = clock.uptime();
    hardware_crc.update(bytes);
    auto const hardware_checksum = hardware_crc.checksum();
    auto const hardware_ticks = clock.uptime() - hardware_start;

    software_crc.reset();
    auto const software_start = clock.uptime();
    software_crc.update(bytes);
    auto const software_checksum = software_crc.checksum();
    auto const software_ticks = clock.uptime() - software_start;

    std::printf(
      "CRC hardware: 0x%08lX (%llu ticks), software: 0x%08lX (%llu ticks)\n",
      static_cast<unsigned long>(hardware_checksum),
      static_cast<unsigned long long>(hardware_ticks),
      static_cast<unsigned long>(software_checksum),
      static_cast<unsigned long long>(software_ticks)
    );

    if (hardware_checksum != software_checksum) {
      std::printf("CRC implementation mismatch; packet not sent\n");
      continue;
    }

    // Corrupt the payload only after calculating the CRC so the display board
    // can demonstrate that it rejects a damaged packet.
    if (corruption_button.level()) {
      batch[0] ^= 1;
    }

    serial->write(std::span<std::uint8_t const>(
      reinterpret_cast<std::uint8_t const*>(batch.data()),
      sizeof(batch)
    ));

    // The packet format uses least-significant byte first for the CRC.
    std::uint8_t crc_bytes[sizeof(hardware_checksum)]{
      static_cast<std::uint8_t>(hardware_checksum),
      static_cast<std::uint8_t>(hardware_checksum >> 8),
      static_cast<std::uint8_t>(hardware_checksum >> 16),
      static_cast<std::uint8_t>(hardware_checksum >> 24),
    };
    serial->write(std::span<std::uint8_t const>(crc_bytes, sizeof(crc_bytes)));
  }
}

// =============================================================================
// Display board: task
// =============================================================================

// TODO(lab, role 4): Construct a serial_driver, a crc32_hardware, two
// lab1::input_pin (S1 and S2), and three lab1::output_pin (the RGB LED).
// Loop forever:
//   1. Wait for S1 or S2 to be pressed.
//   2. Save the current receive_cursor(), then write sample_request_command.
//   3. Wait until packet_size bytes have arrived after the saved position. If
//      they don't arrive within 500 ms, print a timeout message and go back
//      to step 1.
//   4. Recompute the CRC over the received batch and compare it to the
//      received CRC. On a mismatch, print a message and set the LED magenta
//      (red + blue).
//   5. Otherwise, average the batch and set the LED: blue for cold, yellow
//      for warm, red for hot. You pick the bands.
void display_task(void*)
{
  while (true) {
  }
}

// =============================================================================
// Task storage
// =============================================================================

// Resize task_stack_words if necessary. Crashes can be caused by stack
// overflows.
constexpr std::uint32_t task_stack_words = 512;

// Display board: one task.
StaticTask_t display_task_tcb;
StackType_t display_task_stack[task_stack_words];

// TODO(lab, role 4): Add a stack and TCB for each of the three sensor tasks.

// This lab uses two boards playing different roles. Both roles live in this
// one file, and `this_device` below picks which tasks main() creates.
//
// For this lab, flip this by hand and rebuild before flashing each board.
// A more automated setup (e.g. a CI job that builds and flashes both roles
// from a single invocation) would instead read this from a compiler define
// passed on the build system's command line - something like
// `-DDEVICE_ROLE=DEVICE_ROLE_SENSOR` added to the project's build flags -
// rather than a hardcoded constant, so the role never has to be hand-edited
// in source.
enum class device_role : std::uint8_t
{
  sensor,
  display,
};

constexpr device_role this_device = device_role::sensor;
}  // namespace

int main()
{
  std::printf("Hello, World\n");

  if constexpr (this_device == device_role::sensor) {
    // TODO(lab, role 4): Before the scheduler starts:
    //   1. Create sample_request and batch_queue.
    //   2. Create request_task, sampler_task, and transmit_task with
    //      xTaskCreateStatic().
    // request_task and transmit_task share one serial_driver. main()'s stack
    // is reused once the scheduler starts, so don't make it a local here.
  } else {
    xTaskCreateStatic(display_task,
                      "display_task",
                      task_stack_words,
                      nullptr,
                      tskIDLE_PRIORITY + 1,
                      display_task_stack,
                      &display_task_tcb);
  }

  vTaskStartScheduler();

  // Only reached if vTaskStartScheduler() itself fails to start. The
  // idle/timer task memory callbacks required by
  // configSUPPORT_STATIC_ALLOCATION are already supplied by the freertos_kernel
  // project's dpl/StaticAllocs_freertos.c - don't redefine them here.
  while (true) {
  }
}
