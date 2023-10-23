#include "synhal.h"

using namespace syn;


void Adc::init_auto_dma(uint16_t* data_store, uint16_t count)
{
#ifdef STM32F103xB
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR2 = 0;
  ADC1->CR2 = 0;
  ADC1->CR2 = ADC_CR2_ADON;
  syn::System::delay(1);
  // calibrate ADC
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL)
    ;
  // clear eoc to wait on it after the first set of converstions is done
  volatile uint32_t junk = ADC1->DR;
  (void)junk; // kill warning
  // set the sample time to 41.5 cycles for all channels.
  // so converting all 10 channels will take about 46 usec
  ADC1->SMPR2 = 0x24924924;
  // set up the sequence channel 0 to 9
  //ADC1->SQR3 = (5 << 25) | (4 << 20) | (3 << 15) | (2 << 10) | (1 << 5);
  //ADC1->SQR2 = (9 << 15) | (8 << 10) | (7 << 5) | 6;
  ADC1->SQR1 = ((count - 1) << 20); // converstion count
  // turn on continous mode using DMA channel 1
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;
  // setup DMA to write the data into static array in a repititive manner
  Dma dma;
  dma.init(1);
  dma.cyclicP2M(&(ADC1->DR), data_store, count);
  dma.start();
#endif
#ifdef STM32F401xC
#error "Unknown chip!"
#endif
}

void Adc::init_auto_dma_8bit(uint8_t* data_store, uint16_t count)
{
#ifdef STM32F103xB
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR2 = 0;
  ADC1->CR2 = 0;
  ADC1->CR2 = ADC_CR2_ADON;
  syn::System::delay(1);
  // calibrate ADC
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL)
    ;
  // clear eoc to wait on it after the first set of converstions is done
  volatile uint32_t junk = ADC1->DR;
  (void)junk; // kill warning
  // set the sample time to 41.5 cycles for all channels.
  // so converting all 10 channels will take about 46 usec
  ADC1->SMPR2 = 0x24924924;
  // set up the sequence channel 0 to 9
  //ADC1->SQR3 = (5 << 25) | (4 << 20) | (3 << 15) | (2 << 10) | (1 << 5);
  //ADC1->SQR2 = (9 << 15) | (8 << 10) | (7 << 5) | 6;
  ADC1->SQR1 = ((count - 1) << 20); // converstion count
  // turn on continous mode using DMA channel 1
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 |= ADC_CR2_ALIGN | ADC_CR2_CONT | ADC_CR2_DMA;
  // setup DMA to write the data into static array in a repititive manner
  Dma dma;
  dma.init(1);
  dma.cyclicP2M(((uint8_t*)&(ADC1->DR)) + 1, data_store, count);
  dma.start();
#endif
#ifdef STM32F401xC
#error "Unknown chip!"
#endif
}

void Adc::start()
{
  // start converting
  ADC1->CR2 |= ADC_CR2_ADON;
  while (!(ADC1->SR & ADC_SR_EOC))
    ;
}

void Adc::enable(uint16_t channel, uint16_t conversion_idx)
{
#ifdef STM32F103xB
  OS_ASSERT(channel < 10 && conversion_idx < 16, ERR_BAD_INDEX);
  if (channel < 8)
  {
    Gpio pin('A', channel);
    pin.mode(Gpio::in_analog, Gpio::Input);
  }
  else if(channel < 10)
  {
    Gpio pin('B', channel - 8);
    pin.mode(Gpio::in_analog, Gpio::Input);
  }
  if(conversion_idx < 6)
  {
    conversion_idx = conversion_idx * 5;
    uint32_t mask = ~ (uint32_t(0x1F) << conversion_idx);
    ADC1->SQR3 = (ADC1->SQR3 & mask) | (channel << conversion_idx);
  } else if(conversion_idx < 12)
  {
    conversion_idx = (conversion_idx - 6) * 5;
    uint32_t mask = ~ (uint32_t(0x1F) << conversion_idx);
    ADC1->SQR2 = (ADC1->SQR2 & mask) | (channel << conversion_idx);
  } else 
  {
    conversion_idx = (conversion_idx - 12) * 5;
    uint32_t mask = ~ (uint32_t(0x1F) << conversion_idx);
    ADC1->SQR1 = (ADC1->SQR1 & mask) | (channel << conversion_idx);
  }
#endif
#ifdef STM32F401xC
#error "Unknown chip!"
#endif
}