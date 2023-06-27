#include "synhal.h"
#if(SYN_ENABLE_I2C_1 != 0 || SYN_ENABLE_I2C_2 != 0)
using namespace syn;

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

    void init()
    {
      Atomic a;
      if (_isinit)
        return;
      _isinit = true;
      _opdone.init();

      if (_port == I2C1)
      {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
#if (SYN_I2C_1_REMAP == 0)
        Gpio::remap(Gpio::i2c1_scl_pb8_sda_pb9);
        Gpio scl('B', 8);
        Gpio sda('B', 9);
#else
        Gpio scl('B', 6);
        Gpio sda('B', 7);
#endif
        scl.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
        sda.mode(Gpio::out_alt_open_drain, Gpio::MHz_10, Gpio::Alternate::I2C_1);
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
      _port->CR2 = I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | 36;
      if (_port == I2C1)
      {
#ifdef STM32F103xB
#if (SYN_ENABLE_I2C_1 == 400)
        _port->CCR = I2C_CCR_FS | I2C_CCR_DUTY | 4;
        _port->TRISE = 11;
#else
        _port->CCR = 5000 / (1000000000 / 36000000);
        _port->TRISE = (1000 / (1000000000 / 36000000)) + 1;
#endif
      }
      else
      {
#if (SYN_ENABLE_I2C_2 == 400)
        _port->CCR = I2C_CCR_FS | I2C_CCR_DUTY | 4;
        _port->TRISE = 11;
#else
        _port->CCR = 5000 / (1000000000 / 36000000);
        _port->TRISE = (1000 / (1000000000 / 36000000)) + 1;
#endif
      }
#endif //STM32F103xB
      _port->CR1 = I2C_CR1_PE;
    }

    bool masterStartWrite(uint8_t *data, uint16_t size, uint8_t address, uint16_t *success)
    {
      if (_aquireDev(data, size, address, success))
      {
        _port->CR1 = I2C_CR1_START | I2C_CR1_PE;
        return true;
      }
      return false;
    }

    bool masterStartRead(uint8_t *data, uint16_t size, uint8_t address, uint16_t *success)
    {
      if (_aquireDev(data, size, address, success))
      {
        _address |= 0x01;
        if (size == 1)
        {
          // dont even set the ACK bit
          _port->CR1 = I2C_CR1_START | I2C_CR1_PE;
        }
        else if (size == 2)
        {
          // set ACK and POS to nack the 2nd byte
          _port->CR1 = I2C_CR1_POS | I2C_CR1_ACK | I2C_CR1_START | I2C_CR1_PE;
        }
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
            _port->CR1 |= I2C_CR1_STOP;
          }
        }
        else
        {
          if (_remain == 1)
          {
            // 1 byte reception, assert the stop allready
            _port->CR1 |= I2C_CR1_STOP;
          }
          else if (_remain == 2)
          {
            // 2 byte reception clear ack
            _port->CR1 &= ~I2C_CR1_ACK;
          }
        }
      }
      else if (status_1 & I2C_SR1_RXNE)
      {
        // need to read data from the rx buffer
        *_data++ = _port->DR;
        --_remain;
        if (_remain == 2)
        {
          // clear ACK
          _port->CR1 &= ~I2C_CR1_ACK;
        }
        else if (_remain == 1)
        {
          // assert stop
          _port->CR1 |= I2C_CR1_STOP;
        }
        else if (_remain == 0)
        {
          // donezo
          _port->CR1 |= I2C_CR1_STOP;
          *_success = 1;
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
          _port->CR1 |= I2C_CR1_STOP;
          *_success = 1;
          _data = 0;
          _opdone.set();
        }
      }
    }

    void isr_err()
    {
      _port->SR1 = 0;
      _port->CR1 &= ~I2C_CR1_PE;
      System::nop();
      System::nop();
      _port->CR1 |= I2C_CR1_PE;
      *_success = 2;
      _data = 0;
      _opdone.set();
    }

  private:
    bool _aquireDev(uint8_t *data, uint16_t size, uint8_t address, uint16_t *success)
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
        ret = true;
      }
      return ret;
    }

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

void syn::I2cMaster::init(uint16_t port, uint8_t address)
{
  OS_ASSERT(port == 1 || port == 2, ERR_BAD_PORT_NAME);
  if (port == 1)
  {
#if (SYN_ENABLE_I2C_1 != 0)
    _pdev = &i2c::dev_1;
#else
    OS_ASSERT(true == false, ERR_DEVICE_NOT_ENABLED);
#endif
  }
  else
  {
#if (SYN_ENABLE_I2C_2 != 0)
    _pdev = &i2c::dev_2;
#else
    OS_ASSERT(true == false, ERR_DEVICE_NOT_ENABLED);
#endif
  }
  _address = address;
  ((i2c::Device *)_pdev)->init();
}

bool syn::I2cMaster::write(uint8_t *data, uint16_t size)
{
  uint16_t state = 0;
  while (!((i2c::Device *)_pdev)->masterStartWrite(data, size, _address, &state))
  {
    ((i2c::Device *)_pdev)->waitDone(10); // check every 10ms at least to make sure never stuck
  }
  // managed to start the transmission, wait for donezo signal again
  while (state == 0)
  {
    ((i2c::Device *)_pdev)->waitDone(10);
  }
  return state == 1;
}

bool syn::I2cMaster::read(uint8_t *data, uint16_t size)
{
  uint16_t state = 0;
  while (!((i2c::Device *)_pdev)->masterStartRead(data, size, _address, &state))
  {
    ((i2c::Device *)_pdev)->waitDone(10); // check every 10ms at least to make sure never stuck
  }
  // managed to start the transmission, wait for donezo signal again
  while (state == 0)
  {
    ((i2c::Device *)_pdev)->waitDone(10);
  }
  return state == 1;
}

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