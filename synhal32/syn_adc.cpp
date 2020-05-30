#if (SYN_ENABLE_ADC == 1)
#include "synhal.h"
using namespace syn;

#if (SYN_USBCDC_DMA_CHANNEL == 1)
#error "DMA Channel 1 is required by ADC"
#endif

uint16_t Adc::_channels[ADC_CHANNEL_COUNT];

void Adc::init()
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR2 = 0;
  ADC1->CR2 = 0;
  ADC1->CR2 = ADC_CR2_ADON;
  System::udelay(100);
  // calibrate ADC
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL)
    ;
  // clear eoc to wait on it after the first set of converstions is done
  volatile uint32_t junk = ADC1->DR;
  (void)junk; // kill warning
  // set the sample time to 41.5 cycles for all channels. like 3.5usec / channel conversion takes 14 cycels
  // so converting all 10 channels will take about 46 usec
  ADC1->SMPR2 = 0x24924924;
  // set up the sequence channel 0 to 9
  ADC1->SQR3 = (5 << 25) | (4 << 20) | (3 << 15) | (2 << 10) | (1 << 5);
  ADC1->SQR2 = (9 << 15) | (8 << 10) | (7 << 5) | 6;
  ADC1->SQR1 = (9 << 20); // converstion count = 10
  // turn on continous mode using DMA channel 1
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;
  // setup DMA to write the data into static array in a repititive manner
  Dma dma(1);
  dma.cyclicP2M(&(ADC1->DR), _channels, ADC_CHANNEL_COUNT);
  // start converting
  ADC1->CR2 |= ADC_CR2_ADON;
  while (!(ADC1->SR & ADC_SR_EOC))
    ;
}
#endif