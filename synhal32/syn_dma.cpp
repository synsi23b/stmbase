#include "synhal.h"

#include "../../src/synhal_isr.h"

using namespace syn;

extern "C"
{
#ifdef STM32F103xB
#if (SYN_DMA_IRQ_TYPE_1 != 0)
  void DMA1_Channel1_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_1 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 4 * (1 - 1);
    uint16_t status = DMA1->ISR & (0xE << shift);
    syn_dma_1_isr(status >> (shift + 1));
    DMA1->IFCR = status;
#if (SYN_DMA_IRQ_TYPE_1 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_2 != 0)
  void DMA1_Channel2_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_2 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 4 * (2 - 1);
    uint16_t status = DMA1->ISR & (0xE << shift);
    syn_dma_2_isr(status >> (shift + 1));
    DMA1->IFCR = status;
#if (SYN_DMA_IRQ_TYPE_2 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_3 != 0)
  void DMA1_Channel3_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_3 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 4 * (3 - 1);
    uint16_t status = DMA1->ISR & (0xE << shift);
    syn_dma_3_isr(status >> (shift + 1));
    DMA1->IFCR = status;
#if (SYN_DMA_IRQ_TYPE_3 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_4 != 0)
  void DMA1_Channel4_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_4 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 4 * (4 - 1);
    uint16_t status = DMA1->ISR & (0xE << shift);
    syn_dma_4_isr(status >> (shift + 1));
    DMA1->IFCR = status;
#if (SYN_DMA_IRQ_TYPE_4 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_5 != 0)
  void DMA1_Channel5_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_5 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 4 * (5 - 1);
    uint16_t status = DMA1->ISR & (0xE << shift);
    syn_dma_5_isr(status >> (shift + 1));
    DMA1->IFCR = status;
#if (SYN_DMA_IRQ_TYPE_5 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_6 != 0)
  void DMA1_Channel6_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_6 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 4 * (6 - 1);
    uint16_t status = DMA1->ISR & (0xE << shift);
    syn_dma_6_isr(status >> (shift + 1));
    DMA1->IFCR = status;
#if (SYN_DMA_IRQ_TYPE_6 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_7 != 0)
  void DMA1_Channel7_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_7 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 4 * (7 - 1);
    uint32_t status = DMA1->ISR & (0xE << shift);
    syn_dma_7_isr(status >> (shift + 1));
    DMA1->IFCR = status;
#if (SYN_DMA_IRQ_TYPE_7 == 1)
    Core::leave_isr();
#endif
  }
#endif
#endif // ifdef STM32F103xB
#ifdef STM32F401xC
#if (SYN_DMA_IRQ_TYPE_0 != 0)
  void DMA1_Stream0_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_0 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 0;
    uint32_t status = DMA1->LISR & (0x3F << shift);
    syn_dma_0_isr(status >> shift);
    DMA1->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_0 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_1 != 0)
  void DMA1_Stream1_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_1 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 6;
    uint32_t status = DMA1->LISR & (0x3F << shift);
    syn_dma_1_isr(status >> shift);
    DMA1->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_1 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_2 != 0)
  void DMA1_Stream2_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_2 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 16;
    uint32_t status = DMA1->LISR & (0x3F << shift);
    syn_dma_2_isr(status >> shift);
    DMA1->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_2 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_3 != 0)
  void DMA1_Stream3_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_3 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 22;
    uint32_t status = DMA1->LISR & (0x3F << shift);
    syn_dma_3_isr(status >> shift);
    DMA1->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_3 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_4 != 0)
  void DMA1_Stream4_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_4 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 0;
    uint32_t status = DMA1->HISR & (0x3F << shift);
    syn_dma_4_isr(status >> shift);
    DMA1->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_4 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_5 != 0)
  void DMA1_Stream5_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_5 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 6;
    uint32_t status = DMA1->HISR & (0x3F << shift);
    syn_dma_5_isr(status >> shift);
    DMA1->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_5 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_6 != 0)
  void DMA1_Stream6_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_6 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 16;
    uint32_t status = DMA1->HISR & (0x3F << shift);
    syn_dma_6_isr(status >> shift);
    DMA1->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_6 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_7 != 0)
  void DMA1_Stream7_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_7 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 22;
    uint32_t status = DMA1->HISR & (0x3F << shift);
    syn_dma_7_isr(status >> shift);
    DMA1->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_7 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_8 != 0)
  void DMA2_Stream0_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_8 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 0;
    uint32_t status = DMA2->LISR & (0x3F << shift);
    syn_dma_8_isr(status >> shift);
    DMA2->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_8 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_9 != 0)
  void DMA2_Stream1_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_9 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 6;
    uint32_t status = DMA2->LISR & (0x3F << shift);
    syn_dma_9_isr(status >> shift);
    DMA2->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_9 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_10 != 0)
  void DMA2_Stream2_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_10 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 16;
    uint32_t status = DMA2->LISR & (0x3F << shift);
    syn_dma_10_isr(status >> shift);
    DMA2->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_10 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_11 != 0)
  void DMA2_Stream3_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_11 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 22;
    uint32_t status = DMA2->LISR & (0x3F << shift);
    syn_dma_11_isr(status >> shift);
    DMA2->LIFCR = status;
#if (SYN_DMA_IRQ_TYPE_11 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_12 != 0)
  void DMA2_Stream4_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_12 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 0;
    uint32_t status = DMA2->HISR & (0x3F << shift);
    syn_dma_12_isr(status >> shift);
    DMA2->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_12 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_13 != 0)
  void DMA2_Stream5_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_13 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 6;
    uint32_t status = DMA2->HISR & (0x3F << shift);
    syn_dma_13_isr(status >> shift);
    DMA2->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_13 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_14 != 0)
  void DMA2_Stream6_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_14 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 16;
    uint32_t status = DMA2->HISR & (0x3F << shift);
    syn_dma_14_isr(status >> shift);
    DMA2->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_14 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_DMA_IRQ_TYPE_15 != 0)
  void DMA2_Stream7_IRQHandler()
  {
#if (SYN_DMA_IRQ_TYPE_15 == 1)
    Core::enter_isr();
#endif
    const uint16_t shift = 22;
    uint32_t status = DMA2->HISR & (0x3F << shift);
    syn_dma_15_isr(status >> shift);
    DMA2->HIFCR = status;
#if (SYN_DMA_IRQ_TYPE_15 == 1)
    Core::leave_isr();
#endif
  }
#endif
#endif
} // extern "C"

void Dma::enableIrq(uint16_t irq_status_mask, uint16_t priority)
{
#ifdef STM32F103xB
  _pChannel->CCR |= (irq_status_mask << 1);
  uint32_t irqn = _pChannel - DMA1_Channel1;
  irqn += DMA1_Channel1_IRQn - 1;
#endif
#ifdef STM32F401xC
  _pStream->CR |= (irq_status_mask >> 1);
  uint32_t irqn = 0;
  if(_number < 7)
  {
    irqn = DMA1_Stream0_IRQn + _number;
  }
  else if(_number == 7)
  {
    irqn = DMA1_Stream7_IRQn;
  }
  else if(_number < 13)
  {
    irqn = DMA2_Stream0_IRQn + _number - 8;
  }
  else
  {
    irqn = DMA2_Stream5_IRQn + _number - 13;
  }
#endif
#ifdef STM32G030xx
#error "Unknown chip!"
#endif
  Core::enable_isr((IRQn_Type)irqn, priority);
}