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
#ifdef STM32G431xx
  // enable ADC clock
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  ADC12_COMMON->CCR = ADC_CCR_VSENSESEL | ADC_CCR_CKMODE; // ADC FCLK / 4 -> 36MHz
  // calibrate
  //1.Ensure DEEPPWD = 0, ADVREGEN = 1 and that ADC voltage regulator startup time has elapsed.
  // 2. Ensure that ADEN = 0.
  // 3. Select the input mode for this calibration by setting ADCALDIF = 0 (single-ended input) or ADCALDIF = 1 (differential input).
  // 4. Set ADCAL.
  // 5. Wait until ADCAL = 0.
  ADC1->CR = ADC_CR_ADVREGEN;
  syn::System::delay(1);
  ADC1->CR = ADC_CR_ADCAL | ADC_CR_ADVREGEN;
  while( ADC1->CR & ADC_CR_ADCAL)
    ;
  ADC1->CR = ADC_CR_ADEN | ADC_CR_ADVREGEN;
  syn::System::delay(1);
  ADC1->CFGR = ADC_CFGR_CONT | ADC_CFGR_AUTDLY | ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;
  //ADC1->CFGR =  ADC_CFGR_OVRMOD | ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;
  //ADC1->CFGR =  ADC_CFGR_OVRMOD | ADC_CFGR_DMAEN;
  // clear eoc to wait on it after the first set of converstions is done
  volatile uint32_t junk = ADC1->DR;
  (void)junk; // kill warning
  // set sequence lenght to conversion count
  ADC1->SQR1 = count - 1;
  // set conversion sample rate
  ADC1->SMPR1 = 0x12492492;
  ADC1->SMPR2 = 0x12492492;
  Dma dma;
  dma.init(1);
  dma.cyclicP2M(&(ADC1->DR), data_store, count);
  dma.setup_multiplexer(0, 5);
  dma.start();
#endif
}


void Adc::init_cont_single(uint16_t channel)
{
#ifdef STM32F103xB
#error "Unknown chip!"
#endif
#ifdef STM32F401xC
#error "Unknown chip!"
#endif
#ifdef STM32G431xx
  // enable ADC clock
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  ADC12_COMMON->CCR = ADC_CCR_VSENSESEL | ADC_CCR_CKMODE; // ADC FCLK / 4 -> 36MHz
  // calibrate
  //1.Ensure DEEPPWD = 0, ADVREGEN = 1 and that ADC voltage regulator startup time has elapsed.
  // 2. Ensure that ADEN = 0.
  // 3. Select the input mode for this calibration by setting ADCALDIF = 0 (single-ended input) or ADCALDIF = 1 (differential input).
  // 4. Set ADCAL.
  // 5. Wait until ADCAL = 0.
  ADC1->CR = ADC_CR_ADVREGEN;
  syn::System::delay(1);
  ADC1->CR = ADC_CR_ADCAL | ADC_CR_ADVREGEN;
  while( ADC1->CR & ADC_CR_ADCAL)
    ;
  ADC1->CR = ADC_CR_ADEN | ADC_CR_ADVREGEN;
  syn::System::delay(1);
  ADC1->CFGR = ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
  // set sequence lenght to conversion count
  ADC1->SQR1 = 0; // 1 channel
  // set conversion sample rate
  ADC1->SMPR1 = 0x12492492;
  ADC1->SMPR2 = 0x12492492;
#endif
  enable(channel, 0);
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
#ifdef STM32G431xx
  //#error "Unknown chip!"
#endif
}

void Adc::start()
{
  // start converting
#ifdef STM32G431xx
  //OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
  ADC1->CR = ADC_CR_ADSTART | ADC_CR_ADVREGEN;
#else
  ADC1->CR2 |= ADC_CR2_ADON;
  while (!(ADC1->SR & ADC_SR_EOC))
    ;
#endif
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
#ifdef STM32G431xx
  OS_ASSERT(channel < 20 && conversion_idx < 16, ERR_BAD_INDEX);
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
  ++conversion_idx; // start at 0, but the first slot is the total conversion count from setup
  if(conversion_idx < 5)
  {
    conversion_idx = conversion_idx * 6;
    uint32_t mask = ~ (uint32_t(0x1F) << conversion_idx);
    ADC1->SQR1 = (ADC1->SQR1 & mask) | (channel << conversion_idx);
  } else if(conversion_idx < 10)
  {
    conversion_idx = (conversion_idx - 5) * 6;
    uint32_t mask = ~ (uint32_t(0x1F) << conversion_idx);
    ADC1->SQR2 = (ADC1->SQR2 & mask) | (channel << conversion_idx);
  } else if(conversion_idx < 15)
  {
    conversion_idx = (conversion_idx - 10) * 6;
    uint32_t mask = ~ (uint32_t(0x1F) << conversion_idx);
    ADC1->SQR3 = (ADC1->SQR3 & mask) | (channel << conversion_idx);
  }
#endif
#ifdef STM32F401xC
#error "Unknown chip!"
#endif
}


static float calibration_mul = 0.0;
// at 30 degree
#define TS_CAL1 (*((uint16_t*)0x1FFF75A8))
// at 130 degree
#define TS_CAL2 (*((uint16_t*)0x1FFF75CA))

void Adc::enable_temp()
{
  //ADC1->CR1;
  // Temperature in °C
  // (TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1) x (TD_DATA - TS_CAL1) + TS_CAL1_TEMP
  calibration_mul = 100.0 / float((TS_CAL2 - TS_CAL1));
}

int16_t Adc::temperature(uint16_t measured_value, float vref_plus)
{
  // Temperature in °C
  // (TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1) x (TD_DATA - TS_CAL1) + TS_CAL1_TEMP
  if(vref_plus != 3.0f)
  {
    float fac = vref_plus / 3.0f;
    measured_value = uint16_t(float(measured_value) * fac);
  }
  return int16_t(calibration_mul * (measured_value - TS_CAL1)) + 30;
}