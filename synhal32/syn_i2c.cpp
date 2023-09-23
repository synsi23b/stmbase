#include "synhal.h"
#if(SYN_ENABLE_I2C_1 != 0 || SYN_ENABLE_I2C_2 != 0)
using namespace syn;


#ifdef SYN_I2C_ENABLE_DYN_REMAP
static Gpio _scl_n_b8;
static Gpio _sda_n_b9;
static Gpio _scl_r_b6;
static Gpio _sda_r_b7;
#endif

namespace i2c
{
  class Device
  {
  public:
    Device(I2C_TypeDef *port)
    {
      _port = port;
      _isinit = false;
      _data = 0;
    }

    void init(bool remap)
    {
      Atomic a;
      if (_isinit)
        return;
      _isinit = true;
      _opdone.init();


      if (_port == I2C1)
      {
#ifdef SYN_I2C_ENABLE_DYN_REMAP
  // put the correct working mode to the other pins that have not been set by the ini function
#ifdef STM32F103xB
        _scl_n_b8 = Gpio('B', 8);
        _sda_n_b9 = Gpio('B', 9);
        _scl_r_b6 = Gpio('B', 6);
        _sda_r_b7 = Gpio('B', 7);
#else //STM32F103xB
  OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
#endif
#endif //SYN_I2C_ENABLE_DYN_REMAP
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        if(remap)
        {
          Gpio::remap(Gpio::i2c1_scl_pb8_sda_pb9);
          Gpio scl('B', 8);
          Gpio sda('B', 9);
          scl.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
          sda.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
        }
        else
        {
          Gpio::clear_remap(Gpio::i2c1_scl_pb8_sda_pb9);
          Gpio scl('B', 6);
          Gpio sda('B', 7);
          scl.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
          sda.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
        }
        // set irq priority highest - 1 possible with beeing able to call free rtos functions
        NVIC_SetPriority(I2C1_EV_IRQn, 10);
        NVIC_EnableIRQ(I2C1_EV_IRQn);
        NVIC_SetPriority(I2C1_ER_IRQn, 10);
        NVIC_EnableIRQ(I2C1_ER_IRQn);
      }
      else
      {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        Gpio scl('B', 10);
        Gpio sda('B', 11);
        scl.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_2);
        sda.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_2);
        // set irq priority highest - 1 possible with beeing able to call free rtos functions
        NVIC_SetPriority(I2C2_EV_IRQn, 10);
        NVIC_EnableIRQ(I2C2_EV_IRQn);
        NVIC_SetPriority(I2C2_ER_IRQn, 10);
        NVIC_EnableIRQ(I2C2_ER_IRQn);
      }
      _port->CR1 = 0;
      while (_port->CR1 & I2C_CR1_PE)
        ;
#ifdef STM32F103xB
      _port->CR2 = I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | 36;
      //_port->OAR1 = 0x4000; // Bit 14 Should always be kept at 1 by software
      if(_port == I2C1)
      {
#if (SYN_ENABLE_I2C_1 == 400)
      _port->CCR = I2C_CCR_FS | 30;
      _port->TRISE = 11;
#else
      _port->CCR = 180;
      _port->TRISE = 36;
#endif
      }
      else
      {
#if (SYN_ENABLE_I2C_2 == 400)
      _port->CCR = I2C_CCR_FS | 30;
      _port->TRISE = 11;
#else
      _port->CCR = 180;
      _port->TRISE = 36;
#endif        
      }
#else //STM32F103xB
      OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
#endif
    }

    bool aquireDev(uint8_t *data, uint16_t size, uint8_t address, uint16_t *success)
    {
      OS_ASSERT(size > 0, ERR_BAD_INDEX);
      bool ret = false;
      Atomic a;
      if (_data == 0)
      {
        _data = data;
        _remain = size;
        _address = address;
        _success = success;
        _opdone.try_wait(); // make sure the signal is clear
        ret = true;
      }
      return ret;
    }


    bool masterStartWrite(uint16_t *success)
    {
      // make sure we actually acquired the device
      if (_success == success)
      {
        _port->CR1 = I2C_CR1_START | I2C_CR1_PE;
        return true;
      }
      return false;
    }

    bool masterStartRead(uint16_t *success)
    {
      // make sure we actually acquired the device
      if (_success == success)
      {
        _address |= 0x01;
        if (_remain == 1)
        {
          // dont even set the ACK bit
          _port->CR1 = I2C_CR1_START | I2C_CR1_PE;
        }
        // else if (size == 2)
        // {
        //   // set ACK and POS to nack the 2nd byte
        //   _port->CR1 = I2C_CR1_POS | I2C_CR1_ACK | I2C_CR1_START | I2C_CR1_PE;
        // }
        else
        {
          _port->CR1 = I2C_CR1_ACK | I2C_CR1_START | I2C_CR1_PE;
        }
        return true;
      }
      return false;
    }

    bool waitDone(OS_TIME timeout)
    {
      return _opdone.wait(timeout);
    }

    void isr()
    {
      uint32_t status_1 = _port->SR1;

      if (status_1 & I2C_SR1_SB)
      {
        // Start condition generated, write out address
        _port->DR = _address;
      }
      else if (status_1 & I2C_SR1_ADDR)
      {
        // Address was acknowleged
        uint32_t status_2 = _port->SR2;
        // check wether tx or rx
        if (status_2 & I2C_SR2_TRA)
        {
          _port->DR = *_data++;
          --_remain;
          if (_remain == 0)
          {
            // only had to transmit one byte, set stop to finish after this
            _port->CR1 = I2C_CR1_STOP | I2C_CR1_PE;
          }
        }
        else
        {
          if (_remain == 1)
          {
            // 1 byte reception, assert the stop allready
            _port->CR1 = I2C_CR1_STOP | I2C_CR1_PE;
          }
          // else if (_remain == 2)
          // {
          //   // 2 byte reception clear ack
          //   _port->CR1 = I2C_CR1_PE;
          // }
        }
      }
      else if (status_1 & I2C_SR1_RXNE)
      {
        // need to read data from the rx buffer
        *_data++ = _port->DR;
        --_remain;
        // if (_remain == 2)
        // {
        //   // clear ACK
        //   //_port->CR1 &= ~I2C_CR1_ACK;
        // }
        if (_remain == 1)
        {
          // assert stop and clear ACK to NACK byte currently in shift register
          //_port->CR1 &= ~I2C_CR1_ACK;
          _port->CR1 = I2C_CR1_STOP | I2C_CR1_PE;
        }
        else if (_remain == 0)
        {
          // donezo
          _port->CR1 = I2C_CR1_STOP;
          *_success = 1;
          _success = 0;
          _data = 0;
          _opdone.set();
        }
      }
      else if (status_1 & I2C_SR1_TXE)
      {
        // need to write data to the tx buffer
        if (_remain)
        {
          --_remain;
          _port->DR = *_data++;
        }
        else
        {
          // this was the last byte, transmit stop condtion. and donezo
          _port->CR1 = I2C_CR1_STOP;
          *_success = 1;
          _success = 0;
          _data = 0;
          _opdone.set();
        }
      }
      else if (status_1 & (I2C_SR1_BERR | I2C_SR1_STOPF))
      {
        isr_err();
      }
    }

    void isr_err()
    {
      uint32_t status_1 = _port->SR1;
      _port->SR1 = 0;
      if(status_1 & I2C_SR1_AF)
        _port->CR1 = I2C_CR1_STOP;
      else
        _port->CR1 = 0;
      *_success = 2;
      if(_data != 0)
      {
        _data = 0;
        _success = 0;
        _opdone.set();
      }
    }

  private:

    I2C_TypeDef *_port;
    uint8_t *_data;
    uint16_t *_success;
    syn::Signal _opdone;
    uint16_t _remain;
    volatile bool _isinit;
    uint8_t _address;
  };

#if (SYN_ENABLE_I2C_1 != 0)
  Device dev_1(I2C1);
#endif
#if (SYN_ENABLE_I2C_2 != 0)
  Device dev_2(I2C2);
#endif
} // namespace i2c

I2cMaster::I2cMaster()
{

}

I2cMaster::I2cMaster(uint16_t port, uint8_t address)
{
  init(port, address);
}

void syn::I2cMaster::init(uint16_t port, uint8_t address, bool remap)
{
  OS_ASSERT(port == 1 || port == 2, ERR_BAD_PORT_NAME);
  if (port == 1)
  {
    _remapped = remap ? 1 : 0;
#if (SYN_ENABLE_I2C_1 != 0)
    _pdev = &i2c::dev_1;
#else
    OS_ASSERT(true == false, ERR_DEVICE_NOT_ENABLED);
#endif
  }
  else
  {
    _remapped = 0xFF;
#if (SYN_ENABLE_I2C_2 != 0)
    _pdev = &i2c::dev_2;
#else
    OS_ASSERT(true == false, ERR_DEVICE_NOT_ENABLED);
#endif
  }
  _address = address;
  ((i2c::Device *)_pdev)->init(remap);
}

bool syn::I2cMaster::write(uint8_t *data, uint16_t size, uint16_t timeout_ms)
{
  uint16_t state = 0;
  while(true)
  {
    if(((i2c::Device *)_pdev)->aquireDev(data, size, _address, &state))
      break;
    if(timeout_ms == 0)
      return false; // couldnt aquire device
    syn::Thread::sleep(1);
    --timeout_ms;
  }

  if(_remapped != 0xFF)
  {
    I2cMaster::runtime_remap_i2c1(_remapped);
  }

  if(!((i2c::Device *)_pdev)->masterStartWrite(&state))
    return false; // should never happen
  // managed to start the transmission, wait for donezo signal again
  while (state == 0)
  {
    if(!((i2c::Device *)_pdev)->waitDone(1) && timeout_ms == 0)
    {
      return false;
    }
    --timeout_ms;
  }
  return state == 1;
}

bool syn::I2cMaster::read(uint8_t *data, uint16_t size, uint16_t timeout_ms)
{
  uint16_t state = 0;
  while(true)
  {
    if(((i2c::Device *)_pdev)->aquireDev(data, size, _address, &state))
      break;
    if(timeout_ms == 0)
      return false; // couldnt aquire device
    syn::Thread::sleep(1);
    --timeout_ms;
  }

  if(_remapped != 0xFF)
  {
    I2cMaster::runtime_remap_i2c1(_remapped);
  }

  if(!((i2c::Device *)_pdev)->masterStartRead(&state))
    return false; // should never happen
  // minimum timeout is about 32 byte per millisecond for 400kHz bus speed
  uint16_t min_t = size / 32 + 1;
  if(timeout_ms < min_t)
    timeout_ms = min_t;
  // managed to start the transmission, wait for donezo signal again
  while (state == 0)
  {
    if(!((i2c::Device *)_pdev)->waitDone(1) && timeout_ms == 0)
    {
      return false;
    }
    --timeout_ms;
  }
  return state == 1;
}

#ifdef SYN_I2C_ENABLE_DYN_REMAP
void syn::I2cMaster::runtime_remap_i2c1(bool remap)
{
  if(remap == syn::Gpio::is_remapped(Gpio::i2c1_scl_pb8_sda_pb9))
    return;
#ifdef STM32F103xB
  if(!remap)
  {
    _scl_n_b8.mode(Gpio::in_floating);
    _sda_n_b9.mode(Gpio::in_floating);
    Gpio::clear_remap(Gpio::i2c1_scl_pb8_sda_pb9);
    _scl_r_b6.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
    _sda_r_b7.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
  }
  else
  {
    _scl_r_b6.mode(Gpio::in_floating);
    _sda_r_b7.mode(Gpio::in_floating);
    Gpio::remap(Gpio::i2c1_scl_pb8_sda_pb9);
    _scl_n_b8.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
    _sda_n_b9.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
  }
#else //STM32F103xB
  OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
#endif
}
#endif //SYN_I2C_ENABLE_DYN_REMAP

extern "C" {
#if (SYN_ENABLE_I2C_1 != 0)
  void I2C1_EV_IRQHandler()
  {
    Core::enter_isr();
    i2c::dev_1.isr();
    Core::leave_isr();
  }
  void I2C1_ER_IRQHandler()
  {
    Core::enter_isr();
    i2c::dev_1.isr_err();
    Core::leave_isr();
  }
#endif
#if (SYN_ENABLE_I2C_2 != 0)
  void I2C2_EV_IRQHandler()
  {
    Core::enter_isr();
    i2c::dev_2.isr();
    Core::leave_isr();
  }
  void I2C2_ER_IRQHandler()
  {
    Core::enter_isr();
    i2c::dev_2.isr_err();
    Core::leave_isr();
  }
#endif
}
#endif // syn_enable_i2c