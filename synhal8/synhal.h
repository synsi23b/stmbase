#ifndef SYN_STM8S_HAL
#define SYN_STM8S_HAL
#include "stm8s.h"
#include "../../src/synhal_cfg.h"

namespace syn
{

  inline void swap_bytes(uint16_t &value)
  {
    uint8_t *b1, *b2;
    b1 = (uint8_t *)&value;
    b2 = ((uint8_t *)&value) + 1;
    uint8_t tmp = *b1;
    *b1 = *b2;
    *b2 = tmp;
  }

  // block for x milliseconds using an estimation.
  void busy_delay(uint16_t millis);
  // block for x microseconds using an estimation.
  void busy_udelay(uint16_t micros);

  class GpioBase
  {
  public:
    enum eExtiLevel
    {
      Exti_level_low = 0,
      Exti_edge_rise = 1,
      Exti_edge_fall = 2,
      Exti_edge_both = 3
    };

    // INTERRUPT_HANDLER(EXTI_PORTA, 3) {}
    // INTERRUPT_HANDLER(EXTI_PORTB, 4) {}
    // INTERRUPT_HANDLER(EXTI_PORTC, 5) {}
    // INTERRUPT_HANDLER(EXTI_PORTD, 6) {}
    // INTERRUPT_HANDLER(EXTI_PORTE, 7) {}
    template <typename Pin_t>
    static void enableExti(Pin_t pin, eExtiLevel level)
    {
      uint8_t tmp;
      sim();
      switch (pin.baseAddress())
      {
      case GPIOA_BaseAddress:
        tmp = EXTI->CR1 & 0xFC;
        EXTI->CR1 = tmp | level;
        break;
      case GPIOB_BaseAddress:
        tmp = EXTI->CR1 & 0xF3;
        EXTI->CR1 = tmp | (level << 2);
        break;
      case GPIOC_BaseAddress:
        tmp = EXTI->CR1 & 0xCF;
        EXTI->CR1 = tmp | (level << 4);
        break;
      case GPIOD_BaseAddress:
        tmp = EXTI->CR1 & 0x3F;
        EXTI->CR1 = tmp | (level << 6);
        break;
      case GPIOE_BaseAddress:
        tmp = EXTI->CR2 & 0xFC;
        EXTI->CR2 = tmp | level;
        break;
      default:
        while (true)
          ;
      }
      ((GPIO_TypeDef *)pin.baseAddress())->CR2 |= (1 << pin.pinNr());
      rim();
    }

    template <typename Pin_t>
    static void disableExti(Pin_t pin)
    {
      ((GPIO_TypeDef *)pin.baseAddress())->CR2 &= ~(1 << pin.pinNr());
    }
  };

  template <uint16_t BaseAddress>
  class Port
  {
  public:
    // set the direction and behavior of a pin. Allways uses maximum
    // speed possible on the pin in output mode. Pins that don't support
    // high speed will just ignore that setting
    void mode(uint8_t pin, bool output, bool pushpull_pullup)
    {
      if (output)
      {
        if (pushpull_pullup)
        {
          // out_pushpull, max speed
          ((GPIO_TypeDef *)BaseAddress)->DDR |= (1 << pin);
          ((GPIO_TypeDef *)BaseAddress)->CR1 |= (1 << pin);
          ((GPIO_TypeDef *)BaseAddress)->CR2 |= (1 << pin);
        }
        else
        {
          // out_opendrain, max speed
          ((GPIO_TypeDef *)BaseAddress)->DDR |= (1 << pin);
          ((GPIO_TypeDef *)BaseAddress)->CR1 &= ~(1 << pin);
          ((GPIO_TypeDef *)BaseAddress)->CR2 |= (1 << pin);
        }
      }
      else
      {
        ((GPIO_TypeDef *)BaseAddress)->CR2 &= ~(1 << pin);
        if (pushpull_pullup)
        {
          // in_pullup
          ((GPIO_TypeDef *)BaseAddress)->DDR &= ~(1 << pin);
          ((GPIO_TypeDef *)BaseAddress)->CR1 |= (1 << pin);
        }
        else
        {
          // in_float
          ((GPIO_TypeDef *)BaseAddress)->DDR &= ~(1 << pin);
          ((GPIO_TypeDef *)BaseAddress)->CR1 &= ~(1 << pin);
        }
      }
    }

    // set the entire port to the value specified
    // only works for pins configured as output
    void write(uint8_t value)
    {
      ((GPIO_TypeDef *)BaseAddress)->ODR = value;
    }

    // read all the pins of the port
    uint8_t read()
    {
      return ((GPIO_TypeDef *)BaseAddress)->IDR;
    }

    // 1 - pin is set to high, 0 no effect
    // only works for pins configured as output
    void set(uint8_t mask)
    {
      ((GPIO_TypeDef *)BaseAddress)->ODR |= mask;
    }

    // 1 - pin is cleared to low, 0 no effect
    // only works for pins configured as output
    void clear(uint8_t mask)
    {
      ((GPIO_TypeDef *)BaseAddress)->ODR &= ~mask;
    }

    // get the Base Address of the port
    uint16_t baseAddress()
    {
      return BaseAddress;
    }
  };

  template <uint16_t BaseAddress, uint8_t PinNr>
  class Pin
  {
  public:
    // see class Port
    void mode(bool output, bool pushpull_pullup)
    {
      Port<BaseAddress> port;
      port.mode(PinNr, output, pushpull_pullup);
    }

    // Read the value of the Pin
    bool read()
    {
      Port<BaseAddress> port;
      return port.read() & (1 << PinNr);
    }

    // set the Pin to high in output mode
    // should have no effect for input pins
    void set()
    {
      Port<BaseAddress> port;
      port.set(1 << PinNr);
    }

    // clear the Pin to low in output mode
    // should have no effect for input pins
    void clear()
    {
      Port<BaseAddress> port;
      port.clear(1 << PinNr);
    }

    // get the Base Address of the port of the pin
    uint16_t baseAddress()
    {
      return BaseAddress;
    }

    // get the Nr of the pin
    uint8_t pinNr()
    {
      return PinNr;
    }
  };

  template <uint16_t BaseAddress, uint8_t PinNr>
  class DebouncedPin
  {
  public:
    // see class Port
    void mode(bool output, bool pushpull_pullup)
    {
      Port<BaseAddress> port;
      port.mode(PinNr, output, pushpull_pullup);
    }

    // Read the value of the Pin
    bool read()
    {
      Port<BaseAddress> port;
      return port.read() & (1 << PinNr);
    }

    // set the Pin to high in output mode
    // should have no effect for input pins
    void set()
    {
      Port<BaseAddress> port;
      port.set(1 << PinNr);
    }

    // clear the Pin to low in output mode
    // should have no effect for input pins
    void clear()
    {
      Port<BaseAddress> port;
      port.clear(1 << PinNr);
    }

    // get the Base Address of the port of the pin
    uint16_t baseAddress()
    {
      return BaseAddress;
    }

    // get the Nr of the pin
    uint8_t pinNr()
    {
      return PinNr;
    }

    void initDebounce()
    {
      uint8_t count = 0;
      for (uint8_t i = 0; i < 255; ++i)
      {
        if (read())
        {
          ++count;
        }
        busy_udelay(4);
      }
      if (count > 190)
      {
        debounce_ = 0xA0; // set debounce to highest
      }
      else
      {
        debounce_ = 0; // or to lowest
      }
    }

    void setDebounce(bool value)
    { // set debounce start valaue to high or low
      if (value)
        debounce_ = 0xA0;
      else
        debounce_ = 0;
    }

    bool debounced()
    {
      int8_t value = (int8_t)(debounce_ & 0x7F); // dont read debounce bit
      if (read())
      {
        if (++value >= 0x20)
        {
          debounce_ = 0x80 | 0x20;
          return true;
        }
      }
      else
      {
        if (--value <= 0)
        {
          debounce_ = 0;
          return false;
        }
      }
      if (debounce_ & 0x80)
      {
        debounce_ = 0x80 | value;
        return true;
      }
      else
      {
        debounce_ = value;
        return false;
      }
    }

  private:
    uint8_t debounce_;
  };

  class Gpio
  {
  public:
    Gpio()
    {
      //pPort_ = (GPIO_TypeDef*)GPIOF_BaseAddress;
      //pinmask_ = 0;
    }

    Gpio(int8_t port, uint8_t pinnum)
    {
      init(port, pinnum);
    }

    void init(int8_t port, uint8_t pinnum)
    {
      if (port > 'F')
        port -= 32; // only use big letters
      switch (port)
      {
      case 'A':
        pPort_ = (GPIO_TypeDef *)GPIOA_BaseAddress;
        break;
      case 'B':
        pPort_ = (GPIO_TypeDef *)GPIOB_BaseAddress;
        break;
      case 'C':
        pPort_ = (GPIO_TypeDef *)GPIOC_BaseAddress;
        break;
      case 'D':
        pPort_ = (GPIO_TypeDef *)GPIOD_BaseAddress;
        break;
      // case 'E':
      //   pPort_ = (GPIO_TypeDef*)GPIOE_BaseAddress;
      //   break;
      // case 'F':
      //   pPort_ = (GPIO_TypeDef*)GPIOF_BaseAddress;
      //   break;
      default:
        while (true)
          ;
      }
      while (pinnum > 7)
        ;
      pinmask_ = 1 << pinnum;
    }

    // set the direction and behavior of a pin. Allways uses maximum
    // speed possible on the pin in output mode. Pins that don't support
    // high speed will just ignore that setting
    void mode(bool output, bool pushpull_pullup)
    {
      if (output)
      {
        if (pushpull_pullup)
        {
          // out_pushpull, max speed
          pPort_->DDR |= pinmask_;
          pPort_->CR1 |= pinmask_;
          pPort_->CR2 |= pinmask_;
        }
        else
        {
          // out_opendrain, max speed
          pPort_->DDR |= pinmask_;
          pPort_->CR1 &= ~pinmask_;
          pPort_->CR2 |= pinmask_;
        }
      }
      else
      {
        pPort_->CR2 &= ~pinmask_;
        if (pushpull_pullup)
        {
          // in_pullup
          pPort_->DDR &= ~pinmask_;
          pPort_->CR1 |= pinmask_;
        }
        else
        {
          // in_float
          pPort_->DDR &= ~pinmask_;
          pPort_->CR1 &= ~pinmask_;
        }
      }
    }

    // set the entire port to the value specified
    // only works for pins configured as output
    void set()
    {
      pPort_->ODR |= pinmask_;
    }

    void clear()
    {
      pPort_->ODR &= ~pinmask_;
    }

    // read all the pins of the port
    bool read()
    {
      return pPort_->IDR & pinmask_;
    }

    void initDebounce()
    {
      uint8_t count = 0;
      for (uint8_t i = 0; i < 255; ++i)
      {
        if (read())
        {
          ++count;
        }
        busy_udelay(4);
      }
      if (count > 190)
      {
        debounce_ = 0xA0; // set debounce to highest
      }
      else
      {
        debounce_ = 0; // or to lowest
      }
    }

    void setDebounce(bool value)
    { // set debounce start valaue to high or low
      if (value)
        debounce_ = 0xA0;
      else
        debounce_ = 0;
    }

    bool debounced()
    {
      int8_t value = (int8_t)(debounce_ & 0x7F); // dont read debounce bit
      if (read())
      {
        if (++value >= 0x20)
        {
          debounce_ = 0x80 | 0x20;
          return true;
        }
      }
      else
      {
        if (--value <= 0)
        {
          debounce_ = 0;
          return false;
        }
      }
      if (debounce_ & 0x80)
      {
        debounce_ = 0x80 | value;
        return true;
      }
      else
      {
        debounce_ = value;
        return false;
      }
    }

  private:
    GPIO_TypeDef *pPort_;
    uint8_t pinmask_;
    uint8_t debounce_;
  };

  // 640 byte internal eeprom
  class Eeprom
  {
  public:
    // clear the EEPROM write protection to enable writing
    // if this is not called, writing to the EEPROM has no effect
    static void clearWriteProtection()
    {
      FLASH->DUKR = 0xAE;
      FLASH->DUKR = 0x56;
    }

    // enable the write protection of the EEPROM again
    static void enableWriteProtection()
    {
      FLASH->IAPSR &= ~FLASH_IAPSR_DUL;
    }

    // eeprom is organized as 10 blocks with 64 bytes each
    // so input to this function can be block 0 - 9
    // and byte number 0 - 63
    // no special care has to be taken when reading / writing the eeprom
    // other than disabling the write protection first of course
    static uint8_t *getByte(uint8_t block, uint8_t num)
    {
      return (uint8_t *)(0x4000 + block * 64 + num);
    }
  };

  class System
  {
  public:
    enum eClockOutSrc
    {
      Off = 0x00,
      HSIdiv = 0x01,
      LSI = 0x03,
      HSE = 0x05,
      HSI = 0x17,
      Master = 0x19,
      CPU = 0x09,
      CPU_2 = 0x0B,
      CPU_4 = 0x0D,
      CPU_8 = 0x0F,
      CPU_16 = 0x11,
      CPU_32 = 0x13,
      CPU_64 = 0x15
    };

    // set up the system clocks and systick, finally enable interrupts.
    static void init()
    {
      // enable LSI, HSI and fast wakeup from halt
      // disable voltage regulator when power down
      CLK->ICKR = 0x0D | CLK_ICKR_REGAH;
      // wait till LSI and HSI are ready
      while (!(CLK->ICKR & 0x12))
        ;
      // set HSI and CPU prescaler to 1 for full speed 16MHz
      CLK->CKDIVR = 0;
      // enable programm memory write protection
      FLASH->IAPSR &= ~FLASH_IAPSR_PUL;
      // power down flash in active halt as well
      FLASH->CR1 |= FLASH_CR1_AHALT;
      // set up the systick
      systick_init();
      // enable all interrupts
      //rim();
    }

    // disable everything but systick and AWU
    static void disablePeripherals(bool keep_systick = true, bool keep_awu = true)
    {
      if (keep_systick)
        CLK->PCKENR1 = CLK_PCKENR1_TIM4;
      else
        CLK->PCKENR1 = 0;
      if (keep_awu)
        CLK->PCKENR2 = CLK_PCKENR2_AWU;
      else
        CLK->PCKENR2 = 0;
    }

    // enter lowest possible power mode, can only wake up by external interrupt
    static void enterDeepSleep()
    {
      // switch to a low frequency clock to avoid flash reading error from power down
      CLK->CKDIVR = 0x06;
      halt();
      // go back to full speed clock
      CLK->CKDIVR = 0;
    }

    // return the address to the 12 byte unique id of this device
    static uint8_t *uniqueID()
    {
      return (uint8_t *)0x4865;
    }

    // enable / disable the selected alternate Functions
    // table for 20 pin devices
    // bit 0 -> C5 = T2_CH1 / C6 = T1_CH1 / C7 = T1_CH2
    // bit 1 -> A3 = SPI_NSS / D2 = T2_CH3
    // bit 3 -> C3 = TLI
    // bit 4 -> B4 = ADC_ETR / B5 = T1_BKIN
    // bit 7 -> C3 = T1_CH1N / C4 = T1_CH2N
    // table for 32 pin devices
    // bit 1 -> A3 = SPI_NSS / D2 = T2_CH3
    // bit 5 -> D0 = CLK_CCO
    // bit 6 -> D7 = T1_CH4
    // only one alternate function can be selected per pin
    // option bytes can also be changed during flashing process over swim
    // writing option bytes like this inside the application might not work
    // use IAR -> ST-LINK -> Option Bytes menu instead
    static void alternate(uint8_t bitmask)
    {
      // enable write access to option bytes
      FLASH->CR2 |= FLASH_CR2_OPT;
      FLASH->NCR2 &= ~FLASH_CR2_OPT;
      Eeprom::clearWriteProtection();
      OPT->OPT2 = bitmask;
      OPT->NOPT2 = ~bitmask;
      // disable write access to option bytes
      Eeprom::enableWriteProtection();
      FLASH->CR2 &= ~FLASH_CR2_OPT;
      FLASH->NCR2 |= FLASH_CR2_OPT;
    }

    // read current alternate function option bytes
    static uint8_t alternate()
    {
      return OPT->OPT2;
    }

    // When entering halt or active halt mode the iRet instruction at the end
    // of an interrupt will not return to the context, but instead just return to halt mode
    static void noReturnFromISR(bool active)
    {
      if (active)
        CFG->GCR |= CFG_GCR_AL;
      else
        CFG->GCR &= ~CFG_GCR_AL;
    }

    // enable or disable the debugging interface. Pin can be used as a regular GPIO if disabled
    // true = SWIM enabled false = SWIM disabled
    static void setSwim(bool active)
    {
      if (active)
        CFG->GCR &= ~CFG_GCR_SWD;
      else
        CFG->GCR |= CFG_GCR_SWD;
    }

    // enbale the clock out pin with the set source or disable it again with Off as source
    static void setCco(eClockOutSrc source)
    {
      // disable clock out so the source can be switched
      CLK->CCOR &= ~CLK_CCOR_CCOEN;
      // wait for clock to be turned off
      while (CLK->CCOR & CLK_CCOR_CCOBSY)
        ;
      if (source != Off)
      {
#ifndef SYN_HAL_32_PIN_DEVICE
        Pin<GPIOC_BaseAddress, 4> ccopin;
#else
        Gpio ccopin;
        if (alternate() & (1 << 5))
          ccopin = Gpio('D', 0);
        else
          ccopin = Gpio('C', 4);
#endif
        ccopin.mode(true, true);
        CLK->CCOR = source;
      }
    }

    // returns the raw content of the systick
    // is 4 microsec per lsb
    static uint8_t systickRaw()
    {
      return TIM4->CNTR;
    }

    // returns current value of the millis counter
    static uint16_t millis()
    {
      return sMillis;
    }

    // return the lower 8 bit of the millis counter
    static uint8_t millis8()
    {
      return *(((uint8_t *)&sMillis) + 1);
    }

    // block for the specidifed ammount of millis using systick
    static void delay(uint16_t millis)
    {
      uint16_t end = millis + sMillis;
      while ((end - sMillis) <= millis)
        ;
    }

    // Systick ISR, don't call manually
    static void _systick_isr()
    {
      TIM4->SR1 = 0; // clear irq flag
      ++sMillis;
#if (SYN_SYSTICK_FREQ == 2)
      ++sMillis;
#endif
    }

  private:
    static void systick_init()
    {
#if (SYN_SYSTICK_FREQ == 1)
      TIM4->PSCR = 0x6;        // 16MHz / 64 -> 250KHz
      TIM4->ARR = 249;         // 125KHz / 250 -> 1000Hz
      TIM4->EGR = TIM4_EGR_UG; // load the values from shadow registers
      TIM4->IER = TIM4_IER_UIE;
      TIM4->CR1 = TIM4_CR1_URS | TIM4_CR1_CEN;
#endif
#if (SYN_SYSTICK_FREQ == 2)
      TIM4->PSCR = 0x7;        // 16MHz / 128 -> 125KHz
      TIM4->ARR = 249;         // 125KHz / 250 -> 500Hz
      TIM4->EGR = TIM4_EGR_UG; // load the values from shadow registers
      TIM4->IER = TIM4_IER_UIE;
      TIM4->CR1 = TIM4_CR1_URS | TIM4_CR1_CEN;
#endif
    }

    static volatile uint16_t sMillis;
  };

  class DeadlineTimer
  {
  public:
    DeadlineTimer(uint16_t millis)
    {
      millis_ = millis;
    }

    void reset()
    {
      end_ = millis_ - System::millis();
    }

    bool expired() const
    {
      return (end_ - System::millis()) <= millis_;
    }

  private:
    uint16_t millis_;
    uint16_t end_;
  };

#ifndef SYN_HAL_32_PIN_DEVICE
  // The led is connected to the chip at pin B5 with GND, so a low level will make it shine
  // That pin is also a true open drain that doesn't have push capability and is used for sda
  // of the i2c interface. Therefore configuring it with output opendrain and when i2c is active
  // it will probably overwrite the state the was put in by this class anyway. It glows nicely
  // when there is action on the i2c bus thou
  class Led
  {
    typedef Pin<GPIOB_BaseAddress, 5> Pin_t;

  public:
    // configure the pin the LED is connected to
    static void init()
    {
      Pin_t p;
      p.mode(true, false);
    }

    // turn the LED on or off
    // without specifing a the parameter, the led will be turned on
    static void set(bool poweron = true)
    {
      Pin_t p;
      if (poweron)
        p.clear();
      else
        p.set();
    }

    // turn the LED off
    static void clear()
    {
      Pin_t p;
      p.set();
    }

    // switch the LED on or off depending on current state
    static void toggle()
    {
      Pin_t p;
      set(p.read());
    }
  };
#endif

  class Autowakeup
  {
  public:
    // sleep for 1ms up to 30720ms
    // only call this with a constant value and optimization enabled
    // else it will generate a huge mess of code.
    // if another external interrupt needs to stop the sleep, it is
    // requiered to call stopsleep() of this class, or wait until the
    // timeout is reached
    static void sleep(uint16_t millis)
    {
      uint8_t awutb, aprdiv;
      if (millis <= 2)
      {
        awutb = 3;
        aprdiv = (millis * 128) / (1 << (awutb - 1));
      }
      else if (millis <= 4)
      {
        awutb = 4;
        aprdiv = (millis * 128) / (1 << (awutb - 1));
      }
      else if (millis <= 8)
      {
        awutb = 5;
        aprdiv = (millis * 128) / (1 << (awutb - 1));
      }
      else if (millis <= 16)
      {
        awutb = 6;
        aprdiv = (millis * 128) / (1 << (awutb - 1));
      }
      else if (millis <= 32)
      {
        awutb = 7;
        aprdiv = (millis * 128) / (1 << (awutb - 1));
      }
      else if (millis <= 64)
      {
        awutb = 8;
        aprdiv = (millis * 128) / (1 << (awutb - 1));
      }
      else if (millis <= 128)
      {
        awutb = 9;
        aprdiv = (millis * 128) / uint32_t(1 << (awutb - 1));
      }
      else if (millis <= 256)
      {
        awutb = 10;
        aprdiv = (millis * 128) / uint32_t(1 << (awutb - 1));
      }
      else if (millis <= 512)
      {
        awutb = 11;
        aprdiv = uint32_t(uint32_t(millis) * 128) / uint32_t(1 << (awutb - 1));
      }
      else if (millis <= 1024)
      {
        awutb = 12;
        aprdiv = uint32_t(uint32_t(millis) * 128) / uint32_t(1 << (awutb - 1));
      }
      else if (millis <= 2048)
      {
        awutb = 13;
        aprdiv = uint32_t(uint32_t(millis) * 128) / uint32_t(1 << (awutb - 1));
      }
      else if (millis <= 5120)
      {
        awutb = 14;
        aprdiv = uint32_t(uint32_t(millis) * 128) / uint32_t(10240);
      }
      else if (millis <= 30720)
      {
        awutb = 15;
        aprdiv = uint32_t(uint32_t(millis) * 128) / uint32_t(61440);
      }
      else
      {
        // maximum values
        awutb = 15;
        aprdiv = 0x3E;
      }
      AWU->APR = aprdiv;
      AWU->TBR = awutb;
      AWU->CSR = AWU_CSR_AWUEN;
      // keep sleeping until the timeout was reached
      // if another external pin intterupt is triggered halt mode will be left
      // therefore if the interrupt needs to wake up the device, call stopsleep()
      while (true)
      {
        sim();
        if (AWU->CSR == 0)
        {
          rim();
          break;
        }
        System::enterDeepSleep();
      }
    }

    // abort the current autowakeup
    static void stopsleep()
    {
      // clear AWU_EN interrupt
      AWU->CSR = 0;
      // clear TBR to reduce power consumption
      AWU->TBR = 0;
    }
  };

  class Beeper
  {
  public:
    enum eFrequency
    {
      KHz1,
      KHz2,
      KHz4
    };

    // set up the Beeper to use one of the specified standard passive Beeper Frequencies
    static void init(eFrequency fr)
    {
      uint8_t cfg = 0x0E; // Default value when no LSI calibration is done
      switch (fr)
      {
      case KHz2:
        cfg |= (1 << 6);
        break;
      case KHz4:
        cfg |= (1 << 7);
        break;
      }
      BEEP->CSR = cfg;
      Pin<GPIOD_BaseAddress, 4> pin;
      pin.mode(true, true);
    }

    // enable the sound
    static void set()
    {
      BEEP->CSR |= BEEP_CSR_BEEPEN;
    }

    // disable the sound
    static void clear()
    {
      BEEP->CSR &= ~BEEP_CSR_BEEPEN;
    }
  };

  class IWatchdog
  {
  public:
    // set up the Watchdog to cause a device reset after the given ammout of milli seconds
    // a value of 0 is forbidden. If the parameter slow is set true, one lsb of the millis
    // parameter ammounts to 4 milliseconds. Keep in mind that the Watchdog can not be turned
    // off and even runs when using the Atuosleep feature. At startup the Device can check
    // whether the cause of the reset was the watchdog or not.
    static void start(uint8_t millis, bool slow = false)
    {
      // start watchdog operation
      IWDG->KR = 0xCC;
      // enable access to watchdog registers
      IWDG->KR = 0x55;
      // set divider of 1 / 64 so the reload value is in milliseconds
      // if slow, set divider to maximum 1 / 256, 4ms per lsb
      if (slow)
      {
        IWDG->PR = 6;
      }
      else
      {
        IWDG->PR = 4;
      }
      IWDG->RLR = millis;
      // refresh to disable access to registers
      refresh();
    }

    // when the watchdog is enabled, this needs to be called before the timeout
    // is reached, else the device is reseted
    static void refresh()
    {
      IWDG->KR = 0xAA;
    }

    // returns true if the IWDG was the cause of the previous reset
    static bool causeOfReset()
    {
      return RST->SR & RST_SR_IWDGF;
    }
  };

  class WWatchdog
  {
  public:
    // the Window Watchdog has a fixed prescaler and is connected to the CPU frequency
    // the prescaler decrements the window watchdog timer every 12288 cpu cycles
    // a reset is generated if the timer becomes less than 0x40 or if it is refreshed
    // before runnig for at least minimumcnt x 12288 cpu cycles
    // to calculate the propper startvalue at 16Mhz:
    // startvalue = 0x39 + 0.768ms * x  WITH x = 1 .. 64
    // Example: a refresh has to occur at least every 3.84ms but not before running for
    // at least 1.536ms
    // startvalue = 3.84 / 0.768 + 0x39 = 0x44
    // minimumcnt = startvalue - 1.536 / 0.768 = startvalue - 2 = 0x42
    static void start(uint8_t startvalue, uint8_t minimumcnt)
    {
      WWDG->CR = WWDG_CR_WDGA | startvalue;
      WWDG->WR = minimumcnt;
    }

    // see description of start()
    static void refresh(uint8_t startvalue)
    {
      WWDG->CR = startvalue;
    }

    // returns true if the WWDG was the cause of the previous reset
    static bool causeOfReset()
    {
      return RST->SR & RST_SR_WWDGF;
    }
  };

  class Timer1
  {
  public:
    // a config structure that holds the time base generation values for a timer
    // methods to be initialized with some standard values depending on the use
    // of the timer
    struct Timercfg
    {
      // Radio Controll Device conforming PWM signal with 50 Hz
      // trimmed for highest possible resolution
      void rcpwm()
      {
        // divide pclk by 15 -> 1MHz
        prescaler_high = 0;
        prescaler_low = 15;
        // reload at 20.000 -> 50 Hz
        reload_high = 0x4E;
        reload_low = 0x20;
      }
      // arduino like pwm, around 500Hz and duty cycle 0 to 255 (see analogWrite)
      void arduinopwm()
      {
        // divide by 128 to get roughly an update at 125KHz
        prescaler_high = 0;
        prescaler_low = 127;
        // reload at 255 -> ~490Hz
        reload_high = 0;
        reload_low = 0xFE;
      }

      // enable channel 1 and 2 mapped to encoder interface
      void encoder()
      {
        prescaler_high = 0;
        prescaler_low = 0;
        reload_low = 0xFF;
        reload_high = 0xFF;
      }

      uint8_t prescaler_high;
      uint8_t prescaler_low;
      uint8_t reload_high;
      uint8_t reload_low;
    };

    // initialize the timebase according to the cfg value
    static void init(const Timercfg &cfg)
    {
      TIM1->PSCRH = cfg.prescaler_high;
      TIM1->PSCRL = cfg.prescaler_low;
      TIM1->ARRH = cfg.reload_high;
      TIM1->ARRL = cfg.reload_low;
      TIM1->CR1 = TIM1_CR1_CEN;
    }

    // enable PWM output on the channels specified by channelmask
    // bit 0 -> channel 1
    // bit 1 -> channel 2
    // bit 2 -> channel 3
    // bit 3 -> channel 4
    static void enablePWM(uint8_t channelmask)
    {
      TIM1->BKR = TIM1_BKR_MOE | TIM1_BKR_AOE;
      Port<GPIOC_BaseAddress> portc;
      if (channelmask & 0x01)
      {
        // enable PWM Mode 1 with preload
        TIM1->CCMR1 = (0x6 << 4) | TIM1_CCMR_OCxPE;
        TIM1->CCER1 |= TIM1_CCER1_CC1E;
        TIM1->CCR1H = 0;
        TIM1->CCR1L = 0;
#ifndef SYN_HAL_32_PIN_DEVICE
        if (System::alternate() & (1 << 0))
          portc.mode(6, true, true);
#else
        portc.mode(1, true, true);
#endif
      }
      if (channelmask & 0x02)
      {
        // enable PWM Mode 1 with preload
        TIM1->CCMR2 = (0x6 << 4) | TIM1_CCMR_OCxPE;
        TIM1->CCER1 |= TIM1_CCER1_CC2E;
        TIM1->CCR2H = 0;
        TIM1->CCR2L = 0;
#ifndef SYN_HAL_32_PIN_DEVICE
        if (System::alternate() & (1 << 0))
          portc.mode(7, true, true);
#else
        portc.mode(2, true, true);
#endif
      }
      if (channelmask & 0x04)
      {
        // enable PWM Mode 1 with preload
        TIM1->CCMR3 = (0x6 << 4) | TIM1_CCMR_OCxPE;
        TIM1->CCER2 |= TIM1_CCER2_CC3E;
        TIM1->CCR3H = 0;
        TIM1->CCR3L = 0;
        portc.mode(3, true, true);
      }
      if (channelmask & 0x08)
      {
        // enable PWM Mode 1 with preload
        TIM1->CCMR4 = (0x6 << 4) | TIM1_CCMR_OCxPE;
        TIM1->CCER2 |= TIM1_CCER2_CC4E;
        TIM1->CCR4H = 0;
        TIM1->CCR4L = 0;
#ifndef SYN_HAL_32_PIN_DEVICE
        portc.mode(4, true, true);
#else
        if (System::alternate() & (1 << 6))
          Pin<GPIOD_BaseAddress, 7>().mode(true, true);
        else
          portc.mode(4, true, true);
#endif
      }
      TIM1->EGR |= TIM1_EGR_UG;
    }

    // set a pwm duty cycle for the specified channel.
    // channel can be any value between 1 to 4
    // to comply with rc pwm
    // minimum pulsewidth: 1ms should be at 1000
    // maximum pulsewidth: 2ms should be at 2000 giving a resolution of 1000 steps
    // arduino pwm: the minimum is 0 and the maximum is 255.
    static void setPWM(uint16_t duty, uint8_t channel)
    {
      channel -= 1;
      //*((&TIM1->CCR1H) + channel * 2) = duty >> 8;
      // try using a trick instead of shifting 8 times
      // this writes the lower register first and than the high register
      // which doesn't really update the shadow register, but if we write the
      // low register again, it should be ok
      *((uint16_t *)((&TIM1->CCR1H) + channel * 2)) = duty;
      *((&TIM1->CCR1L) + channel * 2) = duty;
    }

    // enable encoder mode for channel 1 and 2 till rolover as specified by the cfg
    static void enableEncoder()
    {
#ifndef SYN_HAL_32_PIN_DEVICE
      if (System::alternate() & (1 << 0))
      {
        Port<GPIOC_BaseAddress> port;
        port.mode(6, false, true);
        port.mode(7, false, true);
      }
#else
      Port<GPIOC_BaseAddress> port;
      port.mode(1, false, true);
      port.mode(2, false, true);
#endif
      TIM1->CCMR1 = 0xF1; // T1 -> input t1fp1, max filter
      TIM1->CCMR2 = 0xF1; // T2 -> input t2fp2, max filter
      TIM1->SMCR = 0x53;  // encoder mode 3
      TIM1->EGR |= TIM1_EGR_UG;
      TIM1->CR1 = TIM1_CR1_CEN;
    }

    // returns the current encoder count and resets it back to zero
    static int16_t readEncoder(uint8_t minval)
    {
      int16_t ret = *((int16_t *)(&TIM1->CNTRH));
      if (ret < 0)
      {
        ret = -ret;
        if (ret > minval)
        {
          *((int16_t *)(&TIM1->CNTRH)) = 0;
          ret /= minval;
          ret = -ret;
        }
        else
        {
          ret = 0;
        }
      }
      else if (ret > minval)
      {
        *((int16_t *)(&TIM1->CNTRH)) = 0;
        ret /= minval;
      }
      else
      {
        ret = 0;
      }
      return ret;
    }
  };

  class Timer2
  {
  public:
    // a config structure that holds the time base generation values for a timer
    // methods to be initialized with some standard values depending on the use
    // of the timer
    struct Timercfg
    {
      // Radio Controll Device conforming PWM signal with 50 Hz
      // trimmed for highest possible resolution
      void rcpwm()
      {
        // divide pclk by 2^4 (16) -> 1MHz
        prescaler = 4;
        // reload at 20.000 -> 50 Hz
        reload_high = 0x4E;
        reload_low = 0x20;
      }

      // arduino like pwm, around 500Hz and duty cycle 0 to 255 (see analogWrite)
      void arduinopwm()
      {
        // divide by 128 to get roughly an update at 125KHz
        prescaler = 7;
        // reload at 255 -> ~490Hz
        reload_high = 0;
        reload_low = 0xFE;
      }
      uint8_t prescaler;
      uint8_t reload_high;
      uint8_t reload_low;
    };

    // initialize the timebase according to the cfg value
    static void init(const Timercfg &cfg)
    {
      TIM2->PSCR = cfg.prescaler;
      TIM2->ARRH = cfg.reload_high;
      TIM2->ARRL = cfg.reload_low;
      TIM2->CR1 = TIM2_CR1_CEN;
    }

    // enable PWM output on the channels specified by channelmask
    // bit 0 -> channel 1
    // bit 1 -> channel 2
    // bit 2 -> channel 3
    static void enablePWM(uint8_t channelmask)
    {
      if (channelmask & 0x01)
      {
        // enable PWM Mode 1 with preload
        TIM2->CCER1 = TIM2->CCER1 & 0xF0;
        TIM2->CCMR1 = (0x6 << 4) | TIM2_CCMR_OCxPE;
        TIM2->CCER1 |= TIM2_CCER1_CC1E;
        TIM2->CCR1H = 0;
        TIM2->CCR1L = 0;
#ifndef SYN_HAL_32_PIN_DEVICE
        if (System::alternate() & (1 << 0))
          Pin<GPIOC_BaseAddress, 5>().mode(true, true);
        else
          Pin<GPIOD_BaseAddress, 4>().mode(true, true);
#else
        Pin<GPIOD_BaseAddress, 4>().mode(true, true);
#endif
      }
      if (channelmask & 0x02)
      {
        // enable PWM Mode 1 with preload
        TIM2->CCER1 = TIM2->CCER1 & 0x0F;
        TIM2->CCMR2 = (0x6 << 4) | TIM2_CCMR_OCxPE;
        TIM2->CCER1 |= TIM2_CCER1_CC2E;
        TIM2->CCR2H = 0;
        TIM2->CCR2L = 0;
        Pin<GPIOD_BaseAddress, 3>().mode(true, true);
      }
      if (channelmask & 0x04)
      {
        // enable PWM Mode 1 with preload
        TIM2->CCER2 = 0;
        TIM2->CCMR3 = (0x6 << 4) | TIM2_CCMR_OCxPE;
        TIM2->CCER2 = TIM2_CCER2_CC3E;
        TIM2->CCR3H = 0;
        TIM2->CCR3L = 0;
        if (System::alternate() & (1 << 1))
          Pin<GPIOD_BaseAddress, 2>().mode(true, true);
        else
          Pin<GPIOA_BaseAddress, 3>().mode(true, true);
      }
      TIM2->EGR |= TIM2_EGR_UG;
    }

    // set a pwm duty cycle for the specified channel.
    // channel can be any value between 1 to 3
    // to comply with rc pwm
    // minimum pulsewidth: 1ms should be at 1000
    // maximum pulsewidth: 2ms should be at 2000 giving a resolution of 1000 steps
    // arduino pwm: the minimum is 0 and the maximum is 255.
    static void setPWM(uint16_t duty, uint8_t channel)
    {
      channel -= 1;
      //*((&TIM1->CCR1H) + channel * 2) = duty >> 8;
      // try useing a trick instead of shifting 8 times
      // this writes the lower register first and than the high register
      // which doesn't really update the shadow register, but if we write the
      // low register again, it should be ok
      *((uint16_t *)((&TIM2->CCR1H) + channel * 2)) = duty;
      *((&TIM2->CCR1L) + channel * 2) = duty;
    }
  };

#ifdef SYN_HAL_SPI
#ifndef SYN_HAL_SPI_SLAVE
  class SpiNC
  {
  public:
    enum eBaudrate
    {
      MHz8 = 0,
      MHz4 = 1,
      MHz2 = 2,
      MHz1 = 3,
      KHz500 = 4,
      KHz250 = 5,
      KHz125 = 6,
      KHz63 = 7
    };
    // initialize the SPI interface with the specifed speed in KHz
    // when leaveing the other parameters blank it uses the most common transfer mode
    // cpol -> clock polarity -> set high or low state during idle (false = low idle)
    // cpha -> clock phase -> set data capture clock edge -> (false = capture on first edge)
    static void init(eBaudrate bd, bool cpol = true, bool cpha = true, bool lsbfirst = false)
    {
      SPI->CR1 = 0;
      SPI->CR2 = SPI_CR2_SSM | SPI_CR2_SSI;
      uint8_t cr1 = (bd << 3) | SPI_CR1_MSTR;
      if (cpol)
        cr1 |= SPI_CR1_CPOL;
      if (cpha)
        cr1 |= SPI_CR1_CPHA;
      if (lsbfirst)
        cr1 |= SPI_CR1_LSBFIRST;
      SPI->CR1 = cr1 | SPI_CR1_SPE;
      Port<GPIOC_BaseAddress> portc;
      portc.mode(5, true, true);   // SCK
      portc.mode(6, true, true);   // MOSI
      portc.mode(7, false, false); // MISO
      // set CLK idle according to cpol
      if (cpol)
        portc.set(1 << 5);
      else
        portc.clear(1 << 5);
    }

    // read count bytes into the data buffer
    static void read(uint8_t *data, uint8_t count);

    // write count bytes from the data buffer
    static void write(const uint8_t *data, uint8_t count);

    // write count bytes of the buffer and read back
    // the same ammount back into the buffer
    static void transceive(uint8_t *data, uint8_t count);

    // read and write 1 byte
    static uint8_t transceive1(uint8_t data);
    // read and write 2 bytes
    static void transceive2(uint8_t *data);
    // write 2 bytes, but read only the second byte
    static void transceive2_01(uint8_t *data);
  };

  template <typename ChipSelect_t>
  class Spi
  {
  public:
    enum eBaudrate
    {
      MHz8 = 0,
      MHz4 = 1,
      MHz2 = 2,
      MHz1 = 3,
      KHz500 = 4,
      KHz250 = 5,
      KHz125 = 6,
      KHz63 = 7
    };
    // initialize the SPI interface with the specifed speed in KHz
    // when leaveing the other parameters blank it uses the most common transfer mode
    static void init(eBaudrate bd, bool cpol = true, bool cpha = true, bool lsbfirst = false)
    {
      SpiNC::init(bd, cpol, cpha, lsbfirst);
      ChipSelect_t csel;
      csel.mode(true, true);
      csel.set();
    }

    // read sizeof(datatype) * count bytes into the data buffer
    template <typename T>
    static void read(T *data, uint8_t count = 1)
    {
      count = count * sizeof(T);
      ChipSelect_t csel;
      csel.clear();
      SpiNC::read(data, count);
      csel.set();
    }

    // write sizeof(datatype) * count bytes from the data buffer
    template <typename T>
    static void write(const T *data, uint8_t count = 1)
    {
      count = count * sizeof(T);
      ChipSelect_t csel;
      csel.clear();
      SpiNC::write(data, count);
      csel.set();
    }

    // write sizeof(datatype) * count bytes of the buffer and read back
    // the same ammount back into the buffer
    template <typename T>
    static void transceive(T *data, uint8_t count = 1)
    {
      count = count * sizeof(T);
      ChipSelect_t csel;
      csel.clear();
      SpiNC::transceive(data, count);
      csel.set();
    }
  };
#else
  class SpiSlave
  {
  public:
    typedef uint8_t (*tx_handler_t)(uint8_t count);
    typedef void (*rx_handler_t)(uint8_t count, uint8_t indata);

    static void init(tx_handler_t txhandler, rx_handler_t rxhandler, bool cpol = true, bool cpha = true, bool lsbfirst = false)
    {
      sTxHandler = txhandler;
      sRxHandler = rxhandler;
      SPI->CR1 = 0;
      SPI->CR2 = SPI_CR2_SSM;
      uint8_t cr1 = 0;
      if (cpol)
        cr1 |= SPI_CR1_CPOL;
      if (cpha)
        cr1 |= SPI_CR1_CPHA;
      if (lsbfirst)
        cr1 |= SPI_CR1_LSBFIRST;
      SPI->CR1 = cr1 | SPI_CR1_SPE;
      Port<GPIOC_BaseAddress> portc;
      portc.mode(5, false, true); // SCK
      portc.mode(6, false, true); // MOSI
      portc.mode(7, false, true); // MISO
    }

    // INTERRUPT_HANDLER(EXTI_PORTA, 3) {}
    // INTERRUPT_HANDLER(EXTI_PORTB, 4) {}
    // INTERRUPT_HANDLER(EXTI_PORTC, 5) {}
    // INTERRUPT_HANDLER(EXTI_PORTD, 6) {}
    // INTERRUPT_HANDLER(EXTI_PORTE, 7) {}
    template <typename Csel_t>
    static void setupExti(Csel_t csel)
    {
      csel.mode(false, true);
      GpioBase::enableExti(csel, GpioBase::Exti_edge_fall);
    }

    // INTERRUPT_HANDLER(EXTI_PORTA, 3) {}
    // INTERRUPT_HANDLER(EXTI_PORTB, 4) {}
    // INTERRUPT_HANDLER(EXTI_PORTC, 5) {}
    // INTERRUPT_HANDLER(EXTI_PORTD, 6) {}
    // INTERRUPT_HANDLER(EXTI_PORTE, 7) {}
    template <typename TxHandler, typename RxHandler, typename Csel>
    static void onSlaveSelect()
    {
      // enable Slave Select Bit to make tell hardware to turn on
      SPI->CR2 |= SPI_CR2_SSI;
      // write the first byte of buffer to the TX
      SPI->DR = TxHandler(0);
      uint8_t count = 0;
      while (!Csel().read())
      {
        // wait for tx to be empty
        if (SPI->SR & SPI_SR_TXE)
        {
          // and write another byte
          SPI->DR = TxHandler(count + 1);
        }
        if (SPI->SR & SPI_SR_RXNE)
        {
          RxHandler(count, SPI->DR);
          ++count;
        }
      }
      // clear the SlaveSelect
      SPI->CR2 &= ~SPI_CR2_SSI;
    }
  };
#endif
#endif
  // need to check if this is allowed, errata sheet says to keep cpu
  // activity to a minimum because it causes glitches when transceiving data.
  // Should call the WFI instruction and set the AL no return from isr bit
  // until transmission is completed to avoid glitches
  class I2c
  {
  public:
    // initialize the device as master that has no address in either
    // fast or slow mode
    static void init(bool fastmode = true)
    {
      // reset the device because glitches at startup
      I2C->CR2 = I2C_CR2_SWRST;
      Port<GPIOB_BaseAddress> portb;
      portb.mode(4, true, false); // scl output open drain
      portb.mode(5, true, false); // sda output open drain
      portb.set(0x30);
      I2C->CR2 = 0;
      I2C->FREQR = 16; // 16 MHz
      if (fastmode)
      {
        I2C->CCRH = I2C_CCRH_FS;
        I2C->CCRL = 0x0E; // 380 kHz
        I2C->TRISER = 5;  // max rise time 300 ns
      }
      else
      {
        I2C->CCRH = 0x00;
        I2C->CCRL = 0x50; // 100 kHz
        I2C->TRISER = 17; // max rise time 1000 ns
      }
      // set address configuration complete, has to be done
      I2C->OARL = 0;
      I2C->OARH = I2C_OARH_ADDCONF;
      // raise i2c interrupt to highest level because errata sheet
      ITC->ISPR5 |= 0xC0;
      // enable the interrupts
      I2C->ITR = I2C_ITR_ITBUFEN | I2C_ITR_ITEVTEN | I2C_ITR_ITERREN;
    }

    // check if any transmission is on going
    static bool busy()
    {
      return I2C->SR3 & I2C_SR3_BUSY;
    }

    // returns 0xFF in case the last transmission had an error
    // returns 0x00 in case the last transmission was succesfull
    // returns any other number if transmission is still ongoing
    static uint8_t state()
    {
      return sCount;
    }

    // Check if the Address specified is ACKed (true) or NACKed (false)
    // bus error also returns false
    static bool checkSlave(uint8_t address)
    {
      while (busy())
        ;
      // dissable the interrupts
      I2C->ITR = 0;
      // enable the device
      I2C->CR1 = I2C_CR1_PE;
      // generate a start condition and enter a polling loop
      I2C->CR2 = I2C_CR2_START;
      bool slaveanswer = false;
      while (true)
      {
        // any error before even transmitting the address
        // means we couldnt even access the bus
        if (I2C->SR2 != 0)
          break;
        if (I2C->SR1 & I2C_SR1_SB)
        {
          // start condition generated, transmitt address
          I2C->DR = address;
          while (true)
          {
            uint8_t state = I2C->SR2;
            // slave address not acknowledged
            if (state & I2C_SR2_AF)
              break;
            // any other error
            if (state != 0)
              break;
            // address acknowledged
            if (I2C->SR1 & I2C_SR1_ADDR)
            {
              // generate a stop condition
              I2C->CR2 = I2C_CR2_STOP;
              slaveanswer = true;
              break;
            }
          }
          break;
        }
      }
      // disable the device to clean up any flags that might have been set
      I2C->CR1 = 0;
      // enable the interrupts again
      I2C->ITR = I2C_ITR_ITBUFEN | I2C_ITR_ITEVTEN | I2C_ITR_ITERREN;
      return slaveanswer;
    }

    // read the ammount of data asynchronely specified by count. data needs to be at least that size
    // the address of the slave is expected to be located in data[0] and will be overwritten
    // by the data returned from slave
    template <typename T>
    static void read_async(T *data, uint8_t count = 1)
    {
      while (busy())
        ;
      *data |= 0x01; // set lsb of address to enter receiver mode
      sData = (uint8_t *)data;
      count = count * sizeof(T);
      sCount = count;
      // enable the device
      I2C->CR1 = I2C_CR1_PE;
      if (count > 2)
      {
        I2C->CR2 = I2C_CR2_ACK | I2C_CR2_START;
      }
      else if (count == 2)
      {
        // set ACK and POS to nack the 2nd byte
        I2C->CR2 = I2C_CR2_POS | I2C_CR2_ACK | I2C_CR2_START;
      }
      else
      {
        // dont set the ACK bit if only one byte will be read
        I2C->CR2 = I2C_CR2_START;
      }
    }

    // read the ammount of data specified by count. data needs to be at least that size
    // the address of the slave is expected to be located in data[0] and will be overwritten
    // by the data returned from slave
    // returns true on completition and false in case of an error
    template <typename T>
    static bool read(T *data, uint8_t count = 1)
    {
      read_async(data, count);
      while (true)
      {
        if (state() == 0xFF)
          return false;
        if (state() == 0x00)
          return true;
      }
    }

    // write the ammount of data specified by count - 1 asynchronely.
    // data needs to be at least of the size count
    // the address of the slave is expected to be located in data[0].
    template <typename T>
    static void write_async(const T *data, uint8_t count)
    {
      while (busy())
        ;
      sData = (uint8_t *)data;
      // when transmitting only one byte, set count to zero to increase isr speed
      sCount = count * sizeof(T) - 2;
      // enable the device
      I2C->CR1 = I2C_CR1_PE;
      I2C->CR2 = I2C_CR2_START;
    }

    // write the ammount of data specified by count - 1.
    // data needs to be at least of the size count
    // the address of the slave is expected to be located in data[0].
    // returns true on completition and false in case of an error
    template <typename T>
    static bool write(T *data, uint8_t count = 1)
    {
      write_async(data, count);
      while (true)
      {
        if (state() == 0xFF)
          return false;
        if (state() == 0x00)
          return true;
      }
    }

    // Isr serving the data transfer. Don't call this manually
    static void isr()
    {
      uint8_t state = I2C->SR2;
      if (state != 0)
      {
        // a wild error has appeared
        sCount = 0xFF;
        //I2C->SR2 = 0;
        I2C->CR2 = I2C_CR2_STOP;
        // disable the device again
        I2C->CR1 = 0;
      }
      else
      {
        state = I2C->SR1;
        if (state & I2C_SR1_ADDR)
        {
          // address ACKed, read SR3 to clear the state
          state = I2C->SR3;
          // if transmitter mode, we need to increment the data pointer to send the next correct byte and fill the buffer
          // else receiver mode, we overwrite the address that was put in first byte of the data buffer
          if (state & I2C_SR3_TRA)
          {
            ++sData;
            // transmitter empty, put data
            I2C->DR = *sData;
            if (sCount == 0)
            {
              // on last byte transmitted, generate a stop condition
              I2C->CR2 = I2C_CR2_STOP;
            }
            else
            {
              // more than one byte, increment data pointer again
              ++sData;
            }
          }
          else
          {
            uint8_t count = sCount;
            if (count == 2)
            {
              // if receiver mode with exactly 2 data bytes. clear ACK to NACK 2nd byte
              I2C->CR2 = I2C_CR2_POS;
            }
            else if (count == 1)
            {
              // if only one byte is to received set a stop instead. The stop will come after the
              // allready ongoing transmission
              I2C->CR2 = I2C_CR2_STOP;
            }
          }
        }
        else if (state & I2C_SR1_RXNE)
        {
          *sData++ = I2C->DR;
          if (--sCount == 1)
          {
            // on 2nd last byte, clear ACK to nack last and send a stop after transmission is complete
            I2C->CR2 = I2C_CR2_STOP;
          }
        }
        else if (state & I2C_SR1_TXE)
        {
          // on last byte, instead of putting data, generate a stop condition
          if (sCount == 0)
          {
            I2C->CR2 = I2C_CR2_STOP;
          }
          else
          {
            // transmitter empty, put data
            I2C->DR = *sData++;
            --sCount;
          }
        }
        else if (state & I2C_SR1_SB)
        {
          // start condition send, transmit address
          I2C->DR = *sData;
        }
      }
    }

  private:
    static uint8_t *sData;
    static volatile uint8_t sCount;
  };

#ifdef SYN_HAL_I2C_SLAVE
  class I2cSlave
  {
  public:
    typedef void (*slaveTxHandle_t)(uint8_t);
    typedef void (*slaveRxHandle_t)(uint8_t, uint8_t);

    static void init(uint8_t ownAddress, slaveTxHandle_t txHandle, slaveRxHandle_t rxHandle)
    {
      I2c::init();
      I2C->OARH = 0;
      I2C->OARL = ownAddress;
      I2C->OARH = I2C_OARH_ADDCONF;
      sTxHandle = txHandle;
      sRxHandle = rxHandle;
      I2C->CR1 = I2C_CR1_PE;
      setACK(true);
    }

    static void writeToMaster(uint8_t data)
    {
      I2C->DR = data;
    }

    static uint8_t readFromMaster()
    {
      return I2C->DR;
    }

    static void setACK(bool enable)
    {
      if (enable)
        I2C->CR2 = I2C_CR2_ACK;
      else
        I2C->CR2 = 0;
    }

    static void isr()
    {
      uint8_t status = I2C->SR1;
      if (status & I2C_SR1_ADDR)
      {
        // address matched and ACKed, clear by reading SR3
        status = I2C->SR3;
        if (status & I2C_SR3_TRA)
        {
          // Slave transmitter mode
          sTxHandle(0);
        }
        // reset the byte counter since a new transmission had started
        sCurrentByte = 0;
      }
      else if (status & I2C_SR1_RXNE)
      {
        sRxHandle(sCurrentByte, I2C->DR);
        ++sCurrentByte;
      }
      else if (status & I2C_SR1_TXE)
      {
        sTxHandle(++sCurrentByte);
      }
      else if (status & I2C_SR1_STOPF)
      {
        I2C->CR1 = 0;
        I2C->CR1 = I2C_CR1_PE;
        setACK(true);
      }
      else
      {
        status = I2C->SR2;
        if (status != 0)
        {
          // error or transmission finished turn device off and on again
          I2C->CR2 = I2C_CR2_STOP;
          I2C->CR1 = 0;
          I2C->CR1 = I2C_CR1_PE;
          setACK(true);
        }
      }
    }

  private:
    static slaveTxHandle_t sTxHandle;
    static slaveRxHandle_t sRxHandle;
    static uint8_t sCurrentByte;
  };
#endif

  class Uart
  {
  public:
    enum eBaudrate
    {
      bd2400,
      bd9600,
      bd19200,
      bd57600,
      bd115200,
      bd232400,
      bd460800,
      bd921600
    };
    enum eStopbits
    {
      sb1 = 0,
      sb1_5 = 3,
      sb2 = 10
    };
    enum eParity
    {
      none,
      odd,
      even
    };

    // initialize the UART interface with the given settings
    // when leaving the parameters blank it is set to the common 8N1
    static void init(eBaudrate bd, eStopbits sb = sb1, eParity pr = none)
    {
      Port<GPIOD_BaseAddress> portd;
      portd.mode(5, true, true);   // tx fast output push pull
      portd.mode(6, false, false); // rx input opendrain
      switch (bd)
      {
      case bd2400:
        UART1->BRR2 = 0x1b;
        UART1->BRR1 = 0xA0;
        break;
      case bd9600:
        UART1->BRR2 = 0x03;
        UART1->BRR1 = 0x68;
        break;
      case bd19200:
        UART1->BRR2 = 0x01;
        UART1->BRR1 = 0x34;
        break;
      case bd57600:
        UART1->BRR2 = 0x06;
        UART1->BRR1 = 0x11;
        break;
      case bd115200:
        UART1->BRR2 = 0x0B;
        UART1->BRR1 = 0x08;
        break;
      case bd232400:
        UART1->BRR2 = 0x05;
        UART1->BRR1 = 0x04;
        break;
      case bd460800:
        UART1->BRR2 = 0x03;
        UART1->BRR1 = 0x02;
        break;
      case bd921600:
        UART1->BRR2 = 0x01;
        UART1->BRR1 = 0x01;
        break;
      }
      // select the parity
      switch (pr)
      {
      case none:
        UART1->CR1 = 0;
        break;
      case odd:
        UART1->CR1 = UART1_CR1_PCEN | UART1_CR1_PS;
        break;
      case even:
        UART1->CR1 = UART1_CR1_PCEN;
        break;
      }
      // set the stop bits
      UART1->CR3 = (sb << 4);
      // enable receiver and transmitter
      UART1->CR2 = UART1_CR2_REN | UART1_CR2_TEN;
    }

    // check whether the last async write is completed or not
    static bool tx_busy()
    {
      return (UART1->SR & UART1_SR_TC) == 0;
    }

    // check whether the last async read is completed or not
    static bool rx_busy()
    {
      return sRxCount != 0;
    }

    // check the ammount of bytes from last async read are still not read
    static uint8_t rx_remaining()
    {
      return sRxCount;
    }

    // abort the current async read
    static void rx_abort()
    {
      sim();
      UART1->CR2 &= ~UART1_CR2_RIEN;
      sRxCount = 0;
      rim();
    }

    // read sizeof(datatype) * count bytes into the buffer data without blocking
    template <typename T>
    static void read_async(T *data, uint8_t count = 1)
    {
      // wait for any other ongoing transmission to complete
      while (rx_busy())
        ;
      sRxData = (uint8_t *)data;
      sRxCount = count * sizeof(T);
      // clear the receiver not empty flag
      UART1->SR &= ~UART1_SR_RXNE;
      // enable Uart interrupt to take care of all transmitting
      UART1->CR2 |= UART1_CR2_RIEN;
    }

    // read sizeof(datatype) * count bytes into the buffer data
    template <typename T>
    static void read(T *data, uint8_t count = 1)
    {
      read_async(data, count);
      // block until transmission is finished
      while (rx_busy())
        ;
    }

    // write sizeof(datatype) * count bytes from the buffer data without blocking
    template <typename T>
    static void write_async(const T *data, uint8_t count = 1)
    {
      // wait for any ongoing transmission to complete
      while (tx_busy())
        ;
      sTxData = (const uint8_t *)data;
      sTxCount = count * sizeof(T);
      // enable Uart interrupt to take care of all transmitting
      UART1->CR2 |= UART1_CR2_TIEN;
    }

    // write sizeof(datatype) * count bytes from the buffer data
    template <typename T>
    static void write(T *data, uint8_t count = 1)
    {
      write_async(data, count);
      // block until transmission is finished
      while (tx_busy())
        ;
    }

    // transmitter isr, don't call manually
    static void tx_isr()
    {
      UART1->DR = *sTxData++;
      if (--sTxCount == 0)
      {
        // when all bytes are transmitted, disable the interrupt again
        UART1->CR2 &= ~UART1_CR2_TIEN;
      }
    }

    // receiver isr, don't call manually
    static void rx_isr()
    {
      *sRxData++ = UART1->DR;
      if (--sRxCount == 0)
      {
        // when all bytes are read, disable the interrupt again
        UART1->CR2 &= ~UART1_CR2_RIEN;
      }
    }

  private:
    static const uint8_t *sTxData;
    static uint8_t *sRxData;
    static volatile uint8_t sTxCount;
    static volatile uint8_t sRxCount;
  };

  // Basic usage:
  // 1. select appropriate init();
  // 2. turnOn() / convert();
  // 3. read() / read8() / read(channel);
  // or just
  // readSingle(channel);
  // which will setup the pin, make one coversion and return the signal.
  // blocks during conversion instead of leaving the ADC running in the background.
  class Adc
  {
  public:
    // enable the Adc
    static void turnOn()
    {
      ADC1->CR1 = ADC1_CR1_ADON;
      System::delay(2);
      convert();
      while (busy())
        ;
    }

    // disable the Adc
    static void turnOff()
    {
      ADC1->CR1 = 0;
      ADC1->TDRL = 0;
    }

    // start a conversion of any kind after having initialized the adc or turn adc on
    static void convert()
    {
      ADC1->CR1 |= ADC1_CR1_ADON;
    }

    // check wether the converion is done or not
    static bool busy()
    {
      return (ADC1->CSR & ADC1_CSR_EOC) == 0;
    }

    // read the last valid 10 bit value in single or continued single channel mode
    static uint16_t read()
    {
      return *((uint16_t *)&(ADC1->DRH));
    }

    // read the last valid 8 bit value in single or continued single channel mode
    static uint8_t read8()
    {
      return ADC1->DRH;
    }

    // read the last valid buffered 10 bit sample of the channel in scan mode
    static uint16_t read(uint8_t channel)
    {
      sim();
      uint16_t data = *((uint16_t *)(&(ADC1->DB0RH) + 2 * channel));
      if (data != *((uint16_t *)(&(ADC1->DB0RH) + 2 * channel)))
        data = *((uint16_t *)(&(ADC1->DB0RH) + 2 * channel));
      rim();
      return data;
    }

    // read the last valid buffered 8 bit sample of the channel in scan mode
    static uint8_t read8(uint8_t channel)
    {
      volatile uint8_t *data = &(ADC1->DB0RH) + 2 * channel;
      return *data;
    }

    // perform the entire Sequence to read a single channel once with 10 bit accuracy
    static uint16_t readSingle(uint8_t channel)
    {
      initSingle(channel);
      convert();
      while (busy())
        ;
      return read();
    }

    // perform the entire Sequence to read a single channel once with 8 bit accuracy
    static uint8_t readSingle8(uint8_t channel)
    {
      initSingle8(channel);
      convert();
      while (busy())
        ;
      return read8();
    }

    // set the ADC to perform a single 10 bit conversion of the specified channel
    // if the ADC is not already running, convert() has to be called once
    // to start the conversations
    static void initSingle(uint8_t channel)
    {
      configurepin(channel);
      ADC1->CR3 = 0;
      ADC1->CR2 = ADC1_CR2_ALIGN;
      ADC1->CSR = channel;
      ADC1->CR1 = ADC1_CR1_ADON;
    }

    // use 8 bit versions of ADC if thats enough precision
    // different alignment in the data register.
    // if the ADC is not already running, convert() has to be called once
    // to start the conversations
    static void initSingle8(uint8_t channel)
    {
      configurepin(channel);
      ADC1->CR3 = 0;
      ADC1->CR2 = 0;
      ADC1->CSR = channel;
      ADC1->CR1 = ADC1_CR1_ADON;
    }

    // set the adc to perform a continous 10 bit precission conversion
    // can not be mixed with single conversions
    // if the ADC is not already running, convert() has to be called once
    // to start the conversations
    static void initContinuous(uint8_t channel)
    {
      configurepin(channel);
      ADC1->CR3 = 0;
      ADC1->CR2 = ADC1_CR2_ALIGN;
      ADC1->CSR = channel;
      ADC1->CR1 = ADC1_CR1_CONT | ADC1_CR1_ADON;
    }

    // set the adc to perform a continous 8 bit precission conversion
    // can not be mixed with single conversions
    // if the ADC is not already running, convert() has to be called once
    // to start the conversations
    static void initContinuous8(uint8_t channel)
    {
      configurepin(channel);
      ADC1->CR3 = 0;
      ADC1->CR2 = 0;
      ADC1->CSR = channel;
      ADC1->CR1 = ADC1_CR1_CONT | ADC1_CR1_ADON;
    }

    // measure all channels from 0 to channel. Any channel int he sequence can't
    // be used as an output pin. 10 bit precision
    // if the ADC is not already running, convert() has to be called once
    // to start the conversations
    static void initScan(uint8_t channel)
    {
      for (uint8_t ch = 2; ch <= channel; ++ch)
        configurepin(ch);
      ADC1->CSR = channel;
      ADC1->CR3 = ADC1_CR3_DBUF;
      ADC1->CR2 = ADC1_CR2_ALIGN | ADC1_CR2_SCAN;
      ADC1->CR1 = ADC1_CR1_CONT | ADC1_CR1_ADON;
    }

    // measure all channels from 0 to channel. Any channel int he sequence can't
    // be used as an output pin. 8 bit precision
    // if the ADC is not already running, convert() has to be called once
    // to start the conversations
    static void initScan8(uint8_t channel)
    {
#ifndef SYN_HAL_32_PIN_DEVICE
      for (uint8_t ch = 2; ch <= channel; ++ch)
      {
#else
      for (uint8_t ch = 0; ch <= channel; ++ch)
      {
#endif
        configurepin(ch);
      }
      ADC1->CSR = channel;
      ADC1->CR3 = ADC1_CR3_DBUF;
      ADC1->CR2 = ADC1_CR2_SCAN;
      ADC1->CR1 = ADC1_CR1_CONT | ADC1_CR1_ADON;
    }

    // stop continous or scan (scan after the last conversion)
    static void stop()
    {
      ADC1->CR1 &= ~ADC1_CR1_CONT;
    }

    // disable a channel for use with the Adc again enabling its
    // Schmitt-Trigger part of the input again
    static void deinit(uint8_t channel)
    {
      // clear the schmitt trigger disable bit again
      ADC1->TDRL &= ~(1 << channel);
    }

    /*
    enable the analog watchdog to trigger an interrupt if the min or max
    value is breached on any channel converted with Adc class methods.

    INTERRUPT_HANDLER(ADC1_ISR, 22){
    // your code here
    // clear irq flag
    Adc::clearAWD();
    }
    */
    static void enableAWD(uint8_t channelmask, uint16_t min, uint16_t max)
    {
      ADC1->HTRL = max & 0x03;
      ADC1->HTRH = max >> 2;
      ADC1->LTRL = min & 0x03;
      ADC1->LTRH = min >> 2;
      // scan only the specified channels
      ADC1->AWCRL = channelmask;
      // clear any pending interrupt before enabling the irq
      clearAWD();
      // enable the analog Watchdog intterrupt
      ADC1->CSR |= ADC1_CSR_AWDIE;
    }

    // returns a mask of channels that all generated a AWD event
    static uint8_t statusAWD()
    {
      return ADC1->AWSRL;
    }

    // clears analog watchdog irq status, has to be called in isr
    static void clearAWD()
    {
      ADC1->AWSRL = 0;
      ADC1->CSR &= ~ADC1_CSR_AWD;
    }

    // stops generating analog watchdog events
    static void disableAWD()
    {
      ADC1->CSR &= ~ADC1_CSR_AWDIE;
    }

  private:
    static void configurepin(uint8_t channel)
    {
#ifndef SYN_HAL_32_PIN_DEVICE
      Port<GPIOD_BaseAddress> portd;
      switch (channel)
      {
      case 2:
      {
        Pin<GPIOC_BaseAddress, 4> pin;
        pin.mode(false, false);
        ADC1->TDRL |= (1 << 2);
      }
      break;
      case 3:
        portd.mode(2, false, false);
        ADC1->TDRL |= (1 << 3);
        break;
      case 4:
        portd.mode(3, false, false);
        ADC1->TDRL |= (1 << 4);
        break;
      case 5:
        portd.mode(5, false, false);
        ADC1->TDRL |= (1 << 5);
        break;
      case 6:
        portd.mode(6, false, false);
        ADC1->TDRL |= (1 << 6);
        break;
      }
#else
      Port<GPIOB_BaseAddress> portb;
      portb.mode(channel, false, false);
      ADC1->TDRL |= (1 << channel);
#endif
    }
  };
} // namespace syn
#endif /* SYN_STM8S_HAL */