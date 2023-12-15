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
Signal _usart2_tx_done;
volatile bool _usart2_rx_loop;

#define USART2_LL_CR1_BASE (USART_CR1_UE | USART_CR1_RE)

void _usart2_ll_init(Usart::eBaudrate baudrate, bool halfduplex)
{
  if(RCC->APB1ENR & RCC_APB1ENR_USART2EN)
  {
    // already initialized
    return;
  }
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  // initialize usart2 globals (private to this file)
  _usart2_tx_done.init();
  _usart2_rx_loop = false;
  _usart2_pread = _usart2_rxbuf;
  // enable rx dma
  _usart2_rx_dma.init(6);
  _usart2_rx_dma.cyclicP2M(&(USART2->DR), (uint8_t*)_usart2_rxbuf, SYN_USART2_RXBUF_SIZE);
  _usart2_rx_dma.enableIrq(Dma::IRQ_STATUS_ERROR | Dma::IRQ_STATUS_FULL);
  _usart2_rx_dma.start();
  // set up tx dma
  _usart2_tx_dma.init(7);
  _usart2_tx_dma.enableIrq(Dma::IRQ_STATUS_ERROR | Dma::IRQ_STATUS_FULL);
  _usart2_tx_dma.start();
  // enable usart hardware
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
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
  USART2->CR1 = USART2_LL_CR1_BASE;
}

void _usart2_ll_tx(const uint8_t* pdata, uint16_t count)
{
  _usart2_tx_dma.oneshotM2P(pdata, &(USART2->DR), count);
  USART2->SR = 0;
  USART2->CR1 = USART2_LL_CR1_BASE | USART_CR1_TE;
  _usart2_tx_dma.start();
  _usart2_tx_done.wait();
  while(!(USART2->SR & USART_SR_TC))
    ;
  USART2->CR1 = USART2_LL_CR1_BASE;
}

// void _usart2_ll_tx_hd(const uint8_t* pdata, uint16_t count)
// {
//   _usart2_tx_dma.oneshotM2P(pdata, &(USART2->DR), count);
//   USART2->SR = 0;
//   // turn off receiver before sending in halfduplex mode
//   USART2->CR1 = (USART2_LL_CR1_BASE | USART_CR1_TE) & ~USART_CR1_RE;
//   _usart2_tx_dma.start();
//   _usart2_tx_done.wait();
//   while(!(USART2->SR & USART_SR_TC))
//     ;
//   USART2->CR1 = USART2_LL_CR1_BASE;
// }

void _usart2_ll_dma_tx_done(uint32_t status)
{
  if(status & 0x1)
  {
    _usart2_tx_done.set();
  }
#ifndef NDEBUG
  // Error
  while (status & 0x4)
    ;
#endif
}

void _usart2_ll_dma_rx_done(uint32_t status)
{
  if(status & 0x1)
  {
    _usart2_rx_loop = true;
  }
#ifndef NDEBUG
  // Error
  while (status & 0x4)
    ;
#endif
}

uint16_t _usart2_ll_avail()
{
  uint16_t done = _usart2_pread - _usart2_rxbuf;
  if(_usart2_rx_loop)
  {
    // if this number gets bigger than buffer size, it will be the overflow condition
    return SYN_USART2_RXBUF_SIZE * 2 - done - _usart2_rx_dma.count();
  }
  else
  {
    return SYN_USART2_RXBUF_SIZE - done - _usart2_rx_dma.count();
  }
}

void _usart2_ll_pread_check()
{
  uint8_t* pbufend = _usart2_rxbuf + SYN_USART2_RXBUF_SIZE - 1;
  if(_usart2_pread > pbufend)
  {
      _usart2_pread = _usart2_rxbuf;
      _usart2_rx_loop = false;
  }
}

uint16_t _usart2_ll_read(uint8_t* pbuf, uint16_t count, uint32_t timeout)
{
  uint16_t avail = _usart2_ll_avail();
  if(avail >= count)
  {
    uint16_t cc = count;
    while(cc > 0)
    {
      *pbuf++ = *_usart2_pread++;
      _usart2_ll_pread_check();
      --cc;
    }
    return count;
  }
  uint16_t readcount = 0;
  while(readcount < avail)
  {
    *pbuf++ = *_usart2_pread++;
    _usart2_ll_pread_check();
    ++readcount;
  }
  // if no timeout, than return with read as much as available
  if(timeout == 0)
  {
    return readcount;
  }
  while(readcount < count)
  {
    if(_usart2_ll_avail() > 0)
    {
      *pbuf++ = *_usart2_pread++;
      _usart2_ll_pread_check();
      ++readcount;
    }
    else
    {
      if(timeout > 0)
      {
        --timeout;
        Thread::sleep(1);
      }
      else
      {
        break;
      }
    }
  }
  return readcount;
}

void _usart2_ll_reset()
{
  _usart2_pread = _usart2_rxbuf;
  _usart2_rx_loop = false;
  _usart2_rx_dma.reset(SYN_USART2_RXBUF_SIZE);
  _usart2_rx_dma.start();
}

#endif //#if(SYN_ENABLE_UART_2 == 3)


void Usart::init(uint16_t dev, eBaudrate baudrate, bool halfduplex)
{
  if(dev == 1)
  {

  }
  else if(dev == 2)
  {
    _usart2_ll_init(baudrate, halfduplex);
    //if(halfduplex)
    //  _write = _usart2_ll_tx_hd;
    //else
    _write = _usart2_ll_tx;
    _read = _usart2_ll_read;
    _avail = _usart2_ll_avail;
    _reset = _usart2_ll_reset;
  }
  else if(dev == 3)
  {

  }
}


extern "C" {
#if(SYN_ENABLE_UART_2 == 3)
  void DMA1_Channel7_IRQHandler()
  {
    Core::enter_isr();
    const uint16_t shift = 4 * (7 - 1);
    uint32_t status = DMA1->ISR & (0xE << shift);
    // 0b100 -> Error
    // 0b010 -> Half complete
    // 0b001 -> Transfer complete
    _usart2_ll_dma_tx_done(status >> (shift + 1));
    DMA1->IFCR = status;
    Core::leave_isr();
  }
  void DMA1_Channel6_IRQHandler()
  {
    //Core::enter_isr();
    const uint16_t shift = 4 * (6 - 1);
    uint32_t status = DMA1->ISR & (0xE << shift);
    // 0b100 -> Error
    // 0b010 -> Half complete
    // 0b001 -> Transfer complete
    _usart2_ll_dma_rx_done(status >> (shift + 1));
    DMA1->IFCR = status;
    //Core::leave_isr();
  }
#endif //#if(SYN_ENABLE_UART_2 == 3)
}
#endif // syn_enable_usart