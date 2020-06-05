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
#endif
#ifdef STM32F401xC
      
#endif
} // extern "C"
