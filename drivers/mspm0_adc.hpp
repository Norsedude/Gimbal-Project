#pragma once

// MSPM0G3507 ADC driver, implementing the lab5::adc16_stream interface
// (hal/adc.hpp). Carried over from lab 5, with the input retargeted from the
// internal temperature sensor to an external BoosterPack pin so a joystick
// potentiometer can be sampled.
//
// ADC0 channel 2 on PA25, which is BoosterPack header pin 2. The reference is
// VDDA (3.3 V), which matches a joystick powered from the 3V3 rail, so the
// full travel of the potentiometer spans the full ADC range.
//
// A DMA channel moves each conversion result into the caller buffer and
// raises one interrupt when the whole buffer is full, so the calling task
// sleeps for the duration of the transfer instead of once per sample.
//
// This driver blocks on a task notification, so read() must only be called
// from a task - never before vTaskStartScheduler() and never from an ISR.

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include <ti/driverlib/driverlib.h>

#include "../hal/adc.hpp"
#include "../third_party/span/span.hpp"

namespace mspm0 {

/// ADC0 channel 2 lives on PA25 = BoosterPack pin 2.
constexpr std::uint32_t adc_joystick_pincm = IOMUX_PINCM55;

/**
 * @brief Blocking, DMA-backed ADC sampler on ADC0 channel 2 (PA25).
 *
 * Only one instance may exist, because it owns ADC0, DMA channel 0, and the
 * ADC0 interrupt vector.
 */
class adc final : public lab5::adc16_stream
{
public:
  adc()
  {
    // PA25 must be switched to its analog function before conversions.
    // GPIOA is only powered, never reset, because port A also carries the
    // SWD debug pins.
    DL_GPIO_enablePower(GPIOA);
    delay_cycles(10);
    DL_GPIO_initPeripheralAnalogFunction(adc_joystick_pincm);

    DL_ADC12_reset(ADC0);
    DL_ADC12_enablePower(ADC0);
    delay_cycles(32);

    // A 4 MHz sample clock with a 50 us acquisition window is far longer
    // than a 10 kOhm potentiometer needs to charge the sampling capacitor.
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
                                 DL_ADC12_INPUT_CHAN_2,
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

    // The ISR calls a FromISR API, so its priority must not be higher than
    // the kernel max syscall priority.
    NVIC_SetPriority(ADC0_INT_IRQn,
                     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(ADC0_INT_IRQn);
  }

  /// Called from the ADC0 interrupt vector to wake the waiting task.
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
    DL_DMA_setDestAddr(
      DMA, dma_channel, reinterpret_cast<std::uintptr_t>(p_buffer.data()));
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

}  // namespace mspm0
