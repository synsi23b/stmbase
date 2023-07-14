#include "synhal.h"

#if(SYN_ENABLE_UART_1 != 0 || SYN_ENABLE_UART_2 != 0 || SYN_ENABLE_UART_3 != 0)
using namespace syn;


void _usart_set_br(USART_TypeDef* pPeri, Usart::eBaudrate br)
{
  OS_ASSERT(SystemCoreClock == 72000000, ERR_NOT_IMPLMENTED);
  if(pPeri == USART1)
  {
    if(br == Usart::eBaudrate::b9600)
      pPeri->BRR = 7500;
    else if(br == Usart::eBaudrate::b19200)
      pPeri->BRR = 3750;
    else if(br == Usart::eBaudrate::b57600)
      pPeri->BRR = 1250;
    else if(br == Usart::eBaudrate::b115200)
      pPeri->BRR = 625;
    else if(br == Usart::eBaudrate::b230400)
      pPeri->BRR = 312;
    else if(br == Usart::eBaudrate::b460800)
      pPeri->BRR = 156;
    else if(br == Usart::eBaudrate::b921600)
      pPeri->BRR = 78;
  }
  else
  {
    // 36MHz baseclock
    if(br == Usart::eBaudrate::b9600)
      pPeri->BRR = 3750;
    else if(br == Usart::eBaudrate::b19200)
      pPeri->BRR = 1250;
    else if(br == Usart::eBaudrate::b57600)
      pPeri->BRR = 625;
    else if(br == Usart::eBaudrate::b115200)
      pPeri->BRR = 312;
    else if(br == Usart::eBaudrate::b230400)
      pPeri->BRR = 156;
    else if(br == Usart::eBaudrate::b460800)
      pPeri->BRR = 78;
    else if(br == Usart::eBaudrate::b921600)
      pPeri->BRR = 39;
  }
}

// mode dma tx / rx
#if(SYN_ENABLE_UART_2 == 3)

uint8_t _usart2_rxbuf[SYN_USART2_RXBUF_SIZE];
uint8_t* _usart2_pread;
Dma _usart2_rx_dma;
Dma _usart2_tx_dma;

void _usart2_ll_init(Usart::eBaudrate baudrate, bool halfduplex)
{
  _usart2_pread = _usart2_rxbuf;
  // enable rx dma
  _usart2_rx_dma.init(6);
  _usart2_rx_dma.cyclicP2M(&(USART2->DR), (uint8_t*)_usart2_rxbuf, SYN_USART2_RXBUF_SIZE);
  _usart2_rx_dma.start();
  // set up tx dma
  _usart2_tx_dma.init(7);
  _usart2_tx_dma.enableIrq(Dma::IRQ_STATUS_ERROR | Dma::IRQ_STATUS_FULL);
  _usart2_tx_dma.start();

  _usart_set_br(USART2, baudrate);
  if(halfduplex)
  {
    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_HDSEL;
    Gpio tx('a', 2);
    tx.mode(Gpio::out_alt_push_pull, Gpio::MHz_10, Gpio::USART_1_2);
  }
  else
  {
    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    Gpio tx('a', 2);
    tx.mode(Gpio::out_alt_push_pull, Gpio::MHz_10, Gpio::USART_1_2);
    Gpio rx('a', 3);
    rx.mode(Gpio::in_floating);
  }
  USART2->CR1 = USART_CR1_UE | USART_CR1_RE;
}

void _usart2_ll_tx(const uint8_t* pdata, uint16_t count)
{
  _usart2_tx_dma.oneshotM2P(pdata, &(USART2->DR), count);
  USART2->SR = 0;
  USART2->CR1 |= USART_CR1_TE;
  _usart2_tx_dma.start();
  while(!(USART2->SR & USART_SR_TC))
    ;
}

void _usart2_ll_dma_tx_done(uint32_t status)
{
  status++;
}

uint16_t _usart2_ll_avail()
{
  uint16_t current = SYN_USART2_RXBUF_SIZE - _usart2_rx_dma.count();
  uint16_t done = _usart2_pread - _usart2_rxbuf;
  if(current >= done)
    return current - done;
  uint16_t left = SYN_USART2_RXBUF_SIZE - done;
  return current + left;
}

void _usart2_ll_read(uint8_t* pbuf, uint16_t count)
{
  uint16_t avail = _usart2_ll_avail();
  if(avail >= count)
  {
    while(count > 0)
    {
      *pbuf++ = *_usart2_pread++;
      if(_usart2_pread > _usart2_rxbuf + SYN_USART2_RXBUF_SIZE - 1)
      {
        _usart2_pread = _usart2_rxbuf;
      }
      --count;
    }
  }
  else
  {
    count -= avail;
    while(avail > 0)
    {
      *pbuf++ = *_usart2_pread++;
      if(_usart2_pread > _usart2_rxbuf + SYN_USART2_RXBUF_SIZE)
      {
        _usart2_pread = _usart2_rxbuf;
      }
      --avail;
    }
    while(count > 0)
    {
      if(_usart2_ll_avail() > 0)
      {
        *pbuf++ = *_usart2_pread++;
        if(_usart2_pread > _usart2_rxbuf + SYN_USART2_RXBUF_SIZE)
        {
          _usart2_pread = _usart2_rxbuf;
        }
        --count;
      }
    }
  }
}

#endif //#if(SYN_ENABLE_UART_2 == 3)


void Usart::write(uint16_t dev, const uint8_t* pdata, uint16_t count)
{
  if(dev == 1)
  {

  }
  else if(dev == 2)
  {
    _usart2_ll_tx(pdata, count);
  }
  else if(dev == 3)
  {

  }
}

uint16_t Usart::available(uint16_t dev)
{
  if(dev == 1)
  {

  }
  else if(dev == 2)
  {
    return _usart2_ll_avail();
  }
  else if(dev == 3)
  {

  }
  return 0;
}

uint16_t Usart::read(uint16_t dev, uint8_t* data, uint16_t count)
{
  if(dev == 1)
  {

  }
  else if(dev == 2)
  {
    _usart2_ll_read(data, count);
  }
  else if(dev == 3)
  {

  }
  return count;
}

void Usart::init(uint16_t dev, eBaudrate baudrate, bool halfduplex)
{
  if(dev == 1)
  {

  }
  else if(dev == 2)
  {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    _usart2_ll_init(baudrate, halfduplex);
  }
  else if(dev == 3)
  {

  }
}

/*
void Usart::remap(uint16_t dev, bool remap)
{

}
*/


extern "C" {
#if(SYN_ENABLE_UART_2 == 3)
  void DMA1_Channel7_IRQHandler()
  {
    Core::enter_isr();
    const uint16_t shift = 4 * (7 - 1);
    uint32_t status = DMA1->ISR & (0xE << shift);
    _usart2_ll_dma_tx_done(status >> (shift + 1));
    DMA1->IFCR = status;
    Core::leave_isr();
  }
#endif //#if(SYN_ENABLE_UART_2 == 3)
}
#endif // syn_enable_usart