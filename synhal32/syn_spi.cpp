#include "synhal.h"

using namespace syn;

void SpiMaster::init(uint16_t port, uint32_t frequency, bool clock_polarity, bool clock_phase, bool transfer_size, bool hardware_slave_sel)
{
  Gpio nss;
  Gpio sck;
  Gpio miso;
  Gpio mosi;
  uint32_t perifreq = 0;
  --port;
  OS_ASSERT(port < 2, ERR_DEVICE_NOT_ENABLED);
#if (defined(STM32F103xB) || defined(STM32F401xC))
  switch (port)
  {
  case 0:
    _pSpi = SPI1;
    OS_ASSERT((RCC->APB2ENR & RCC_APB2ENR_SPI1EN) == 0, ERR_FORBIDDEN);
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    nss.init('A', 4);
    sck.init('A', 5);
    miso.init('A', 6);
    mosi.init('A', 7);
    perifreq = SystemCoreClock;
    break;
  case 1:
    _pSpi = SPI2;
    OS_ASSERT((RCC->APB1ENR & RCC_APB1ENR_SPI2EN) == 0, ERR_FORBIDDEN);
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    nss.init('B', 12);
    sck.init('B', 13);
    miso.init('B', 14);
    mosi.init('B', 15);
    perifreq = SystemCoreClock / 2;
    break;
  }
  uint32_t baudrateselect = 1;
  for(; baudrateselect < 8; ++baudrateselect)
  {
    uint32_t baudrate = perifreq >> baudrateselect;
    if(baudrate <= frequency)
    {
      break;
    }
  }
  --baudrateselect;

  uint32_t cfg = baudrateselect << 3 | SPI_CR1_MSTR | clock_polarity << 1 | clock_phase;
  if(transfer_size)
  {
    // 16 bit mode
    _pSpi->CR1 = SPI_CR1_DFF | cfg;
  }

  Gpio::Speed speed;
  if (frequency < 10000000)
  {
    speed = Gpio::MHz_10;
  }
  else
  {
    speed = Gpio::MHz_50;
  }

  if(hardware_slave_sel)
  {
    nss.mode(Gpio::out_alt_push_pull, Gpio::MHz_10, Gpio::SPI);
    nss.setWeakPullUpDown(true, false);
    _pSpi->CR2 = SPI_CR2_SSOE;
  }
  sck.mode(Gpio::out_alt_push_pull, speed, Gpio::SPI);
  if(clock_polarity)
  {
    sck.setWeakPullUpDown(true, false);
  }
  else
  {
    sck.setWeakPullUpDown(false, true);
  }
  miso.mode(Gpio::out_alt_push_pull, speed, Gpio::SPI);
  miso.setWeakPullUpDown(true, false);
  mosi.mode(Gpio::out_alt_push_pull, speed, Gpio::SPI);
  mosi.setWeakPullUpDown(true, false);
#endif
#if (defined(STM32G030xx))
  Gpio::Speed speed;
  if (frequency < 10000000)
  {
    speed = Gpio::MHz_10;
  }
  else
  {
    speed = Gpio::MHz_50;
  }
  switch (port)
  {
  case 0:
    _pSpi = SPI1;
    OS_ASSERT((RCC->APBENR2 & RCC_APBENR2_SPI1EN) == 0, ERR_FORBIDDEN);
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
    if(hardware_slave_sel)
    {
      nss.init('B', 0);
      nss.mode(Gpio::out_alt_push_pull, Gpio::MHz_10, Gpio::AF0);
      nss.setWeakPullUpDown(true, false);
      _pSpi->CR2 |= SPI_CR2_SSOE;
    }
    sck.init('A', 5);
    sck.mode(Gpio::out_alt_push_pull, speed, Gpio::AF0);
    miso.init('A', 6);
    miso.mode(Gpio::out_alt_push_pull, speed, Gpio::AF0);
    mosi.init('A', 7);
    mosi.mode(Gpio::out_alt_push_pull, speed, Gpio::AF0);
    perifreq = SystemCoreClock;
    break;
  case 1:
    _pSpi = SPI2;
    OS_ASSERT((RCC->APBENR1 & RCC_APBENR1_SPI2EN) == 0, ERR_FORBIDDEN);
    RCC->APBENR1 |= RCC_APBENR1_SPI2EN;
    if(hardware_slave_sel)
    {
      nss.init('B', 9);
      nss.mode(Gpio::out_alt_push_pull, Gpio::MHz_10, Gpio::AF5);
      nss.setWeakPullUpDown(true, false);
      _pSpi->CR2 |= SPI_CR2_SSOE;
    }
    sck.init('A', 0);
    sck.mode(Gpio::out_alt_push_pull, speed, Gpio::AF0);
    miso.init('A', 3);
    miso.mode(Gpio::out_alt_open_drain, speed, Gpio::AF0);
    mosi.init('A', 4);
    mosi.mode(Gpio::out_alt_push_pull, speed, Gpio::AF1);
    perifreq = SystemCoreClock;
    break;
  }
    uint32_t baudrateselect = 1;
  for(; baudrateselect < 8; ++baudrateselect)
  {
    uint32_t baudrate = perifreq >> baudrateselect;
    if(baudrate <= frequency)
    {
      break;
    }
  }
  --baudrateselect;
  if(transfer_size)
  {
    // 16 bit mode
    _pSpi->CR2 |= SPI_CR2_DS_3;
  }
  _pSpi->CR1 = baudrateselect << 3 | SPI_CR1_MSTR | clock_polarity << 1 | clock_phase;
  
  if(clock_polarity)
  {
    sck.setWeakPullUpDown(true, false);
  }
  else
  {
    sck.setWeakPullUpDown(false, true);
  }
  miso.setWeakPullUpDown(false, false);
  mosi.setWeakPullUpDown(true, false);
#endif

}

bool SpiMaster::busy_tx(const uint8_t *pbuffer, uint16_t size)
{
  // check 8 bit data frame format
#if (defined(STM32G030xx))
  OS_ASSERT((_pSpi->CR2 & SPI_CR2_DS_3) == 0, ERR_FORBIDDEN);
#else
  OS_ASSERT((_pSpi->CR1 & SPI_CR1_DFF) == 0, ERR_FORBIDDEN);
#endif
  if(_pSpi->SR & SPI_SR_BSY)
    return false;
  _pSpi->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
  while(size > 0)
  {
    if(_pSpi->SR & SPI_SR_TXE)
    {
      *((uint8_t*)&_pSpi->DR) = *pbuffer++;
      --size;
    }
  }
  while(_pSpi->SR & SPI_SR_BSY)
    ;
  while(_pSpi->SR & SPI_SR_RXNE)
  {
    uint8_t junk = *((uint8_t*)&_pSpi->DR);
    (void)junk;
  }
  _pSpi->CR1 &= ~SPI_CR1_SPE;
  return true;
}

bool SpiMaster::busy_tx(const uint16_t *pbuffer, uint16_t size)
{
  // check 16 bit data frame format
#if (defined(STM32G030xx))
  OS_ASSERT((_pSpi->CR2 & SPI_CR2_DS_3) != 0, ERR_FORBIDDEN);
#else
  OS_ASSERT((_pSpi->CR1 & SPI_CR1_DFF) != 0, ERR_FORBIDDEN);
#endif
  if(_pSpi->SR & SPI_SR_BSY)
    return false;
  _pSpi->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
  while(size > 0)
  {
    if(_pSpi->SR & SPI_SR_TXE)
    {
      _pSpi->DR = *pbuffer++;
      --size;
    }
  }
  while(_pSpi->SR & SPI_SR_BSY)
    ;
  while(_pSpi->SR & SPI_SR_RXNE)
  {
    uint16_t junk = _pSpi->DR;
    (void)junk;
  }
  _pSpi->CR1 &= ~SPI_CR1_SPE;
  return true;
}

bool SpiMaster::busy_bidi(uint8_t *pbuffer, uint16_t size)
{
  // check 8 bit data frame format
#if (defined(STM32G030xx))
  OS_ASSERT((_pSpi->CR2 & SPI_CR2_DS_3) == 0, ERR_FORBIDDEN);
#else
  OS_ASSERT((_pSpi->CR1 & SPI_CR1_DFF) == 0, ERR_FORBIDDEN);
#endif
  if(_pSpi->SR & SPI_SR_BSY)
    return false;
  _pSpi->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
  while(size > 0)
  {
    //while((_pSpi->SR & SPI_SR_TXE) == 0)
    //  ;
    *((uint8_t*)&_pSpi->DR) = *pbuffer;
    while((_pSpi->SR & SPI_SR_RXNE) == 0)
      ;
    *pbuffer++ = *((uint8_t*)&_pSpi->DR);
    --size;
  }
  _pSpi->CR1 &= ~SPI_CR1_SPE;
  return true;
}

bool SpiMaster::busy_read_regs(uint8_t startaddress, uint8_t* pbuffer, uint16_t size)
{
  // check 8 bit data frame format
#if (defined(STM32G030xx))
  OS_ASSERT((_pSpi->CR2 & SPI_CR2_DS_3) == 0, ERR_FORBIDDEN);
#else
  OS_ASSERT((_pSpi->CR1 & SPI_CR1_DFF) == 0, ERR_FORBIDDEN);
#endif
  if(_pSpi->SR & SPI_SR_BSY)
    return false;
  // start transfer
  _pSpi->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
  // push register address from where to read
  *((uint8_t*)&_pSpi->DR) = startaddress;
  // wait to receive first byte
  while((_pSpi->SR & SPI_SR_RXNE) == 0)
      ;
  uint8_t junk = *((uint8_t*)&_pSpi->DR);
  (void)junk;
  while(size > 0)
  {
    //while((_pSpi->SR & SPI_SR_TXE) == 0)
    //  ;
    *((uint8_t*)&_pSpi->DR) = 0;
    while((_pSpi->SR & SPI_SR_RXNE) == 0)
      ;
    *pbuffer++ = *((uint8_t*)&_pSpi->DR);
    --size;
  }
  // turn off SPI
  _pSpi->CR1 &= ~SPI_CR1_SPE;
  return true;
}