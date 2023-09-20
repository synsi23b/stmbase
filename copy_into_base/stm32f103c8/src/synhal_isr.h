#pragma once
#include <synhal.h>

inline void syn_exti_0_isr()
{
}

inline void syn_exti_1_isr()
{
}

inline void syn_exti_2_isr()
{
}

inline void syn_exti_3_isr()
{
}

inline void syn_exti_4_isr()
{
}

inline void syn_exti_5_isr()
{
}

inline void syn_exti_6_isr()
{
}

inline void syn_exti_7_isr()
{
}

inline void syn_exti_8_isr()
{
}

inline void syn_exti_9_isr()
{
}

inline void syn_exti_10_isr()
{
}

inline void syn_exti_11_isr()
{
}

inline void syn_exti_12_isr()
{
}

inline void syn_exti_13_isr()
{
}

inline void syn_exti_14_isr()
{
}

inline void syn_exti_15_isr()
{
}

inline void syn_timer_1_isr()
{
}

inline void syn_timer_2_isr()
{
}

inline void syn_timer_3_isr()
{
}

inline void syn_timer_4_isr()
{
}

inline void syn_timer_5_isr()
{
}

inline void syn_dma_0_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

// STM32F103xB: ADC1 TIM2_CH3 TIM4_CH1
inline void syn_dma_1_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

// STM32F103xB: SPI1_RX USART3_TX TIM1_CH1 TIM2_UP TIM3_CH3
inline void syn_dma_2_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

// STM32F103xB: SPI1_TX USART3_RX TIM3_CH4 TIM3_UP
inline void syn_dma_3_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

// STM32F103xB: SPI2/I2S2_RX USART1_TX I2C2_TX TIM1_CH4 TIM1_TRIG TIM1_COM TIM4_CH2
inline void syn_dma_4_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

// STM32F103xB: SPI2/I2S2_TX USART1_RX I2C2_RX TIM1_UP TIM2_CH1 TIM4_CH3
inline void syn_dma_5_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

// STM32F103xB: USART2_RX I2C1_TX TIM1_CH3 TIM3_CH1 TIM3_TRIG
inline void syn_dma_6_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

// STM32F103xB: USART2_TX I2C1_RX TIM2_CH2 TIM2_CH4 TIM4_UP
inline void syn_dma_7_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_8_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_9_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_10_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_11_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_12_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_13_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_14_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}

inline void syn_dma_15_isr(uint16_t irq_status)
{
  (void)irq_status; // avoid "unsued variable" compiler warning
}