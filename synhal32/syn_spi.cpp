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
    if(baudrate < frequency)
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
}

bool SpiMaster::busy_tx(const uint8_t *pbuffer, uint16_t size)
{
  OS_ASSERT((_pSpi->CR1 & SPI_CR1_DFF) == 0, ERR_FORBIDDEN);
}

bool SpiMaster::busy_tx(const uint16_t *pbuffer, uint16_t size)
{
  OS_ASSERT((_pSpi->CR1 & SPI_CR1_DFF) != 0, ERR_FORBIDDEN);
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
  _pSpi->CR1 &= ~SPI_CR1_SPE;
  return true;
}