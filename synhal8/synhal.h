#ifndef SYN_STM8S_HAL
#define SYN_STM8S_HAL
#include "stm8s.h"
#include "../../src/synhal_cfg.h"

namespace syn
{
  class Utility
  {
  public:
    // swap upper and lower byte of 16 bit value
    inline void swap_bytes(uint16_t &value)
    {
      uint8_t *b1, *b2;
      b1 = (uint8_t *)&value;
      b2 = ((uint8_t *)&value) + 1;
      uint8_t tmp = *b1;
      *b1 = *b2;
      *b2 = tmp;
    }

    // block for the specified ammount using an estimation
    static void udelay(uint16_t micros);

    // return number of characters excluding trailing zero
    static uint8_t strlen(const char *str);
    // copys the string until trailing zero and returns pointer to next write position
    // doesn't add a trailing zero and doesn't check for anything really
    static char *strcpy(char *dst, const char *str);
    // write signed integer as asci to the outbuffer, returns pointer to next write position
    static char *sprint_i16(char *dst, int16_t value);
    // write unsigned integer as asci to the outbuffer, returns pointer to next write position
    static char *sprint_u16(char *dst, uint16_t value);
    // write unsigned integer as asci hex to the outbuffer, returns pointer to next write position
    static char *sprint_hex(char *dst, uint8_t value);
    // compare two arrays for equality
    static bool memcmp(const uint8_t *a1, const uint8_t *a2, uint8_t len);
    // copy array and return pointer dst + count (not copied location)
    static uint8_t* memcpy(uint8_t* dst, const uint8_t *src, uint8_t count);
    // zero array
    static void clear_array(uint8_t* data, uint8_t count);
  };

  class GpioBase
  {
  public:
    static GPIO_TypeDef *get_port_address(char port)
    {
#ifdef DEBUG
      while (port < 'A' || port > 'D')
        ;
#endif
      return (GPIO_TypeDef *)(GPIOA_BaseAddress + ((port - 'A') * 0x05));
    }

    static void pushpull(GPIO_TypeDef *port, uint8_t mask);
    static void opendrain(GPIO_TypeDef *port, uint8_t mask);
    static void input_pullup(GPIO_TypeDef *port, uint8_t mask);
    static void floating(GPIO_TypeDef *port, uint8_t mask);
    static void high_speed(GPIO_TypeDef *port, uint8_t mask);

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

  class Gpio
  {
  public:
    Gpio()
    {
    }

    Gpio(char port, uint8_t pinnum)
    {
      init(port, pinnum);
    }

    Gpio &init(char port, uint8_t pinnum)
    {
      _port = GpioBase::get_port_address(port);
      _pinmask = 1 << pinnum;
      return *this;
    }

    Gpio &init_multi(char port, uint8_t pinmask)
    {
      _port = GpioBase::get_port_address(port);
      _pinmask = pinmask;
      return *this;
    }

    Gpio &change_pin_mask(uint8_t new_mask)
    {
      _pinmask = new_mask;
      return *this;
    }

    Gpio &change_pin_mask_by_number(uint8_t new_pinnum)
    {
      _pinmask = 1 << new_pinnum;
      return *this;
    }

    Gpio &pushpull();
    Gpio &opendrain();
    Gpio &input_pullup();
    Gpio &floating();
    // WARNING! in input mode it enables EXTI
    // should not be used unless in peripheral initialization
    Gpio &high_speed();

    // used to quickly toggle between pushpull<->input_pullup or opendrain<->floating
    // use cases when bitbanging, check datasheets!
    void clear_direction_bit()
    {
      _port->DDR &= ~_pinmask;
    }

    void set_direction_bit()
    {
      _port->DDR |= _pinmask;
    }

    void set()
    {
      _port->ODR |= _pinmask;
    }

    void clear()
    {
      _port->ODR &= ~_pinmask;
    }

    bool read()
    {
      return _port->IDR & _pinmask;
    }

  private:
    GPIO_TypeDef *_port;
    uint8_t _pinmask;
  };

  class DebouncedGpio
  {
    void initDebounce()
    {
      uint8_t count = 0;
      for (uint8_t i = 0; i < 255; ++i)
      {
        if (_pin.read())
        {
          ++count;
        }
        Utility::udelay(4);
      }
      if (count > 190)
      {
        _debounce = 0xA0; // set debounce to highest
      }
      else
      {
        _debounce = 0; // or to lowest
      }
    }

    void setDebounce(bool value)
    { // set debounce start valaue to high or low
      if (value)
        _debounce = 0xA0;
      else
        _debounce = 0;
    }

    bool debounced()
    {
      int8_t value = (int8_t)(_debounce & 0x7F); // dont read debounce bit
      if (_pin.read())
      {
        if (++value >= 0x20)
        {
          _debounce = 0x80 | 0x20;
          return true;
        }
      }
      else
      {
        if (--value <= 0)
        {
          _debounce = 0;
          return false;
        }
      }
      if (_debounce & 0x80)
      {
        _debounce = 0x80 | value;
        return true;
      }
      else
      {
        _debounce = value;
        return false;
      }
    }

  private:
    Gpio _pin;
    uint8_t _debounce;
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
      //while (!(CLK->ICKR & 0x12)) // HSI is ready after reset, already in use
      //  ;
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
        Gpio ccopin;
#ifndef SYN_HAL_32_PIN_DEVICE
        ccopin.init('C', 4);
#else
        Gpio ccopin;
        if (alternate() & (1 << 5))
          ccopin = Gpio('D', 0);
        else
          ccopin = Gpio('C', 4);
#endif
        ccopin.pushpull();
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
      return _millis;
    }

    // return the lower 8 bit of the millis counter
    static uint8_t millis8()
    {
      return *(((uint8_t *)&_millis) + 1);
    }

    // block for the specidifed ammount of millis using systick
    static void delay(uint16_t millis);

#ifdef SYN_SYSTEM_SECONDS_SUPPORT
    static uint32_t *seconds()
    {
      return &_seconds;
    }
#endif

    // Systick ISR, don't call manually
    static void _systick_isr()
    {
      TIM4->SR1 = 0; // clear irq flag
      uint16_t tmp = ++_millis;
#ifdef SYN_SYSTEM_SECONDS_SUPPORT
      if ((tmp % 1000) == 0)
      {
        ++_seconds;
      }
#endif
    }

  private:
    static void systick_init()
    {
      TIM4->PSCR = 0x6;        // 16MHz / 64 -> 250KHz
      TIM4->ARR = 249;         // 125KHz / 250 -> 1000Hz
      TIM4->EGR = TIM4_EGR_UG; // load the values from shadow registers
      TIM4->IER = TIM4_IER_UIE;
      TIM4->CR1 = TIM4_CR1_URS | TIM4_CR1_CEN;
    }

    static volatile uint16_t _millis;
#ifdef SYN_SYSTEM_SECONDS_SUPPORT
    static uint32_t _seconds;
#endif
  };

  // capture the current time during reset, than allows querries whether a specific ammount
  // of milliseconds have passed or not. However due to timer roll over in the 16 bit timer,
  // the running out can only be detected during the window of (UINT16_MAX - COMPARE_VALUE)
  // the timer rolls over every 65 seconds.
  class DeadlineTimer
  {
  public:
    // reset needs to be called to prime the timer before comapring
    void reset()
    {
      _reset_time = System::millis();
    }

    bool expired(uint16_t compare_value) const
    {
      // http://www.thetaeng.com/designIdeas/TimerWrap.html
      return (System::millis() - _reset_time) >= compare_value;
    }

  private:
    uint16_t _reset_time;
  };

  class MinuteDeadlineTimer
  {
  public:
    void reset(uint8_t offset = 0)
    {
      _deadline.reset();
      _minutes_past = offset;
    }

    // compare if compare_value minutes have past since last reset
    // has to be called at least every 5 seconds, better faster.
    bool expired(uint8_t compare_value)
    {
      if (_deadline.expired(60000))
      {
        _deadline.reset();
        ++_minutes_past;
      }
      return _minutes_past >= compare_value;
    }

  private:
    DeadlineTimer _deadline;
    uint8_t _minutes_past;
  };

#ifndef SYN_HAL_32_PIN_DEVICE
  // The led is connected to the chip at pin B5 with GND, so a low level will make it shine
  // That pin is also a true open drain that doesn't have push capability and is used for sda
  // of the i2c interface. Therefore configuring it with output opendrain and when i2c is active
  // it will probably overwrite the state the was put in by this class anyway. It glows nicely
  // when there is action on the i2c bus thou
  class Led
  {
  public:
    // configure the pin the LED is connected to
    static void init()
    {
      Gpio p('B', 5);
      p.opendrain();
    }

    // turn the LED on or off
    // without specifing a the parameter, the led will be turned on
    static void set(bool poweron = true)
    {
      Gpio p('B', 5);
      if (poweron)
        p.clear();
      else
        p.set();
    }

    // turn the LED off
    static void clear()
    {
      Gpio p('B', 5);
      p.set();
    }

    // switch the LED on or off depending on current state
    static void toggle()
    {
      Gpio p('B', 5);
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
#ifndef SYN_HAL_AUTO_WAKEUP_UNIT
      // need to enable the AWU to actually use it..
      return;
#else
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
#endif
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
      Gpio tmp_pin;
      tmp_pin.init('D', 4);
      tmp_pin.pushpull();
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
    // set up the Watchdog to cause a device reset after the given amount of milliseconds
    // a value of 0 is forbidden. If the parameter slow is set true, one LSB of the milliseconds
    // parameter amounts to 4 milliseconds. Keep in mind that the Watchdog can not be turned
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
    // Radio Controll Device conforming PWM signal with 50 Hz
    // to provide 1000 steps between 1ms and 2ms signal length
    static void init_rcpwm()
    {
      // divide pclk by 2^4 (16) -> 1MHz
      TIM1->PSCRH = 0;
      TIM1->PSCRL = 15;
      // reload at 19999 -> 50 Hz
      TIM1->ARRH = 0x4E;
      TIM1->ARRL = 0x1F;
      TIM1->CR1 = TIM1_CR1_CEN;
    }

    // arduino like pwm, around 500Hz and duty cycle 0 to 255 (see analogWrite)
    static void init_arduinopwm()
    {
      // divide by 128 to get roughly an update at 125KHz
      TIM1->PSCRH = 0;
      TIM1->PSCRL = 127;
      // reload at 254 -> ~490Hz
      TIM1->ARRH = 0;
      TIM1->ARRL = 0xFE;
      TIM1->CR1 = TIM1_CR1_CEN;
    }

    // initialize to run on 25khz
    // minimum value is zero, if that works for the fans
    // 100% duity cycle is at value 128 or above
    static void init_pc_fan_pwm()
    {
      // divide pclk by 5 -> 3.2 MHz -> 312.5 nanosec per tick
      TIM1->PSCRH = 0;
      TIM1->PSCRL = 4;
      // 40usec (25KHz) / 312.5 nanosec = 128 parts aka 100% duty.
      TIM1->ARRH = 0;
      TIM1->ARRL = 127;
      TIM1->CR1 = TIM1_CR1_CEN;
    }

    // enable channel 1 and 2 mapped to encoder interface
    static void init_encoder()
    {
      TIM1->PSCRH = 0;
      TIM1->PSCRL = 0;
      TIM1->ARRH = 0xFF;
      TIM1->ARRL = 0xFF;
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
      Gpio temp_pin('C', 6);
      if (channelmask & 0x01)
      {
        // enable PWM Mode 1 with preload
        TIM1->CCMR1 = (0x6 << 4) | TIM1_CCMR_OCxPE;
        TIM1->CCER1 |= TIM1_CCER1_CC1E;
        TIM1->CCR1H = 0;
        TIM1->CCR1L = 0;
#ifndef SYN_HAL_32_PIN_DEVICE
        if (!System::alternate() & (1 << 0))
        {
          while (true)
            ;
        }
#else
        temp_pin.change_pin_mask(0x01);
#endif
        temp_pin.pushpull();
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
        {
          temp_pin.init('C', 7);
        }
        else
          while (true)
            ;
#else
        temp_pin.change_pin_mask(0x02);
#endif
        temp_pin.pushpull();
      }
      if (channelmask & 0x04)
      {
        // enable PWM Mode 1 with preload
        TIM1->CCMR3 = (0x6 << 4) | TIM1_CCMR_OCxPE;
        TIM1->CCER2 |= TIM1_CCER2_CC3E;
        TIM1->CCR3H = 0;
        TIM1->CCR3L = 0;
        temp_pin.init('C', 3);
        temp_pin.pushpull();
      }
      if (channelmask & 0x08)
      {
        // enable PWM Mode 1 with preload
        TIM1->CCMR4 = (0x6 << 4) | TIM1_CCMR_OCxPE;
        TIM1->CCER2 |= TIM1_CCER2_CC4E;
        TIM1->CCR4H = 0;
        TIM1->CCR4L = 0;
#ifndef SYN_HAL_32_PIN_DEVICE
        temp_pin.init('C', 4);
        temp_pin.pushpull();
#else
        if (System::alternate() & (1 << 6))
        {
          temp_pin.init('D', 7);
          temp_pin.pushpull();
        }
        else
        {
          temp_pin.init('C', 4);
          temp_pin.pushpull();
        }
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
      uint8_t *ccr_high = (uint8_t *)(&TIM1->CCR1H) + channel * 2;
      // in the manual it says, the LDW instruction is forbidden, and we need to write
      // the MSB and than the LSB. LDW instruction writes the LSB first, so, just do it twice
      // LDW first, than LD (8 bit)
      *((uint16_t *)ccr_high) = duty;
      ++ccr_high; // point to ccr_low
      *ccr_high = duty & 0xFF;
    }

    // enable encoder mode for channel 1 and 2 till rolover as specified by the cfg
    static void enableEncoder()
    {
#ifndef SYN_HAL_32_PIN_DEVICE
      if (System::alternate() & (1 << 0))
      {
        Gpio temp_pin;
        temp_pin.init('C', 6);
        temp_pin.input_pullup();
        temp_pin.init('C', 7);
        temp_pin.input_pullup();
      }
#else
      Gpio temp_pin;
      temp_pin.init('C', 1);
      temp_pin.input_pullup();
      temp_pin.init('C', 2);
      temp_pin.input_pullup();
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
      // see reference page 142 for reading 16 bit counter correctly
      // MSB first, than LSB. LDW reads LSB first
      union retunion
      {
        int16_t sig16;
        uint8_t bytes[2];
      };
      retunion ret;
      ret.bytes[0] = TIM1->CNTRH;
      ret.bytes[1] = TIM1->CNTRL;

      if (ret.sig16 < 0)
      {
        ret.sig16 = -ret.sig16;
        if (ret.sig16 > minval)
        {
          *((int16_t *)(&TIM1->CNTRH)) = 0;
          ret.sig16 /= minval;
          ret.sig16 = -ret.sig16;
        }
        else
        {
          ret.sig16 = 0;
        }
      }
      else if (ret.sig16 > minval)
      {
        *((int16_t *)(&TIM1->CNTRH)) = 0;
        ret.sig16 /= minval;
      }
      else
      {
        ret.sig16 = 0;
      }
      return ret.sig16;
    }
  };

  class Timer2
  {
  public:
    // Radio Controll Device conforming PWM signal with 50 Hz
    // to provide 1000 steps between 1ms and 2ms signal length
    static void init_rcpwm()
    {
      // divide pclk by 2^4 (16) -> 1MHz
      TIM2->PSCR = 4;
      // reload at 19999 -> 50 Hz
      TIM2->ARRH = 0x4E;
      TIM2->ARRL = 0x1F;
      TIM2->CR1 = TIM2_CR1_CEN;
    }

    // arduino like pwm, around 500Hz and duty cycle 0 to 255 (see analogWrite)
    static void init_arduinopwm()
    {
      // divide by 128 to get roughly an update at 125KHz
      TIM2->PSCR = 7;
      // reload at 254 -> ~490Hz
      TIM2->ARRH = 0;
      TIM2->ARRL = 0xFE;
      TIM2->CR1 = TIM2_CR1_CEN;
    }

    // enable PWM output on the channels specified by channelmask
    // bit 0 -> channel 1
    // bit 1 -> channel 2
    // bit 2 -> channel 3
    static void enablePWM(uint8_t channelmask)
    {
      Gpio temp_pin;
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
        {
          temp_pin.init('C', 5);
          temp_pin.pushpull();
        }
        else
        {
          temp_pin.init('D', 4);
          temp_pin.pushpull();
        }
#else
        temp_pin.init('D', 4);
        temp_pin.pushpull();
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
        temp_pin.init('D', 3);
        temp_pin.pushpull();
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
        {
          temp_pin.init('D', 2);
          temp_pin.pushpull();
        }
        else
        {
          temp_pin.init('A', 3);
          temp_pin.pushpull();
        }
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
      // channel -= 1;
      // //*((&TIM1->CCR1H) + channel * 2) = duty >> 8;
      // // try useing a trick instead of shifting 8 times
      // // this writes the lower register first and than the high register
      // // which doesn't really update the shadow register, but if we write the
      // // low register again, it should be ok
      // *((uint16_t *)((&TIM2->CCR1H) + channel * 2)) = duty;
      // *((&TIM2->CCR1L) + channel * 2) = duty;

      channel -= 1;
      uint8_t *ccr_high = (uint8_t *)(&TIM2->CCR1H) + channel * 2;
      // in the manual it says, the LDW instruction is forbidden, and we need to write
      // the MSB and than the LSB. LDW instruction writes the LSB first, so, just do it twice
      // LDW first, than LD (8 bit)
      *((uint16_t *)ccr_high) = duty;
      ++ccr_high; // point to ccr_low
      *ccr_high = duty & 0xFF;
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
      Gpio pins;
      pins.init_multi('C', 0x60) // pin 5, 6 SCK / MOSI
          .pushpull()
          .high_speed();
      // set CLK idle according to cpol
      if (cpol)
        pins.set();
      else
        pins.clear();
      pins.change_pin_mask_by_number(7)
          .floating();
    }

    // write count bytes from the constant data buffer, but discard incoming data
    // before the data buffer is written, a command byte is injected into the stream
    static void write_command(uint8_t command, const uint8_t *data, uint8_t count);

    // write count bytes of the buffer and read back
    // the same ammount back into the buffer
    static void transceive(uint8_t *data, uint8_t count);
    // read and write 1 byte
    static uint8_t transceive1(uint8_t data);
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

    // write sizeof(datatype) * count bytes from the data buffer
    // template <typename T>
    // static void write(const T *data, uint8_t count = 1)
    // {
    //   count = count * sizeof(T);
    //   ChipSelect_t csel;
    //   csel.clear();
    //   SpiNC::write(data, count);
    //   csel.set();
    // }

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
    enum Speed
    {
      slow,
      medium,
      high
    };
    // initialize the device as master that has no address in either
    // slow, medium or fast mode (100kHz, 380kHz)
    static void init(Speed speed = high);

    // check if any transmission is on going
    static bool busy()
    {
      //return sCount != 0;
      return I2C->SR3 & I2C_SR3_BUSY;
    }

    static void busy_loop(void(*functor)(void))
    {
      while(busy())
        functor();
    }

    // returns 0xFF in case the last transmission had an error
    // returns 0x00 in case the last transmission was successful
    // returns any other number if transmission is still ongoing
    static uint8_t state()
    {
      return sCount;
    }

    // in TX mode, there will be one byte injected before actually
    // transmitting the tx buffer. This is useful for command bytes
    // common to i2c protocols
    // when using async methods, be careful to check for not busy
    static void set_inject(uint8_t inject_byte)
    {
      sTxInjectByte = inject_byte;
    }

    // Check if the Address specified is ACKed (true) or NACKed (false)
    // bus error also returns false
    static bool checkSlave(uint8_t address);

    // read the amount of data specified by count.
    // data needs to be at least that size
    // use state() to check result of the transaction
    static void read_async(uint8_t address, uint8_t *data, uint8_t count);

    // read the amount of data specified by count.
    // data needs to be at least that size
    // returns true on completion and false in case of an error
    static bool read(uint8_t address, uint8_t *data, uint8_t count);

    // write the amount of data specified by count.
    // data needs to be at least of the size count
    // use state() to check result of the transaction
    static void write_async(uint8_t address, const uint8_t *data, uint8_t count);

    // write the amount of data specified by count.
    // data needs to be at least of the size count
    // returns true on completion and false in case of an error
    static bool write(uint8_t address, const uint8_t *data, uint8_t count);

    // ISR serving the data transfer. Don't call this manually
    static void _isr();

  private:
    static uint8_t *sData;
    static volatile uint8_t sCount;
    static uint8_t sAddress;
    static uint8_t sTxInjectByte;
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
    static void init(eBaudrate bd = bd115200, eStopbits sb = sb1, eParity pr = none)
    {
      Gpio pin;
      pin.init('D', 5)
          .pushpull();
      pin.change_pin_mask_by_number(6)
          .floating();
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

    // check the amount of bytes from last async read are still not read
    static uint8_t rx_avail();
    // enable the receiver ISR to actually receive stuff
    static void rx_start();
    // stop the receiver ISR, does not delete already received data
    static void rx_stop();
    // read up to count bytes into the buffer data
    static uint8_t read(uint8_t *data, uint8_t count);

    // reset the receiver buffer back to zero
    static void rx_flush();

    // check if there was a buffer overrun since the last time this method was called
    static bool rx_overrun();
    
    static void putc(uint8_t c)
    {
      while (tx_busy())
        ;
      UART1->DR = c;
    }

    // write count bytes from the buffer data without blocking
    static void write_async(const uint8_t *data, uint8_t count)
    {
      // wait for any ongoing transmission to complete
      while (tx_busy())
        ;
      sTxData = data;
      sTxCount = count;
      // enable Uart interrupt to take care of all transmitting
      UART1->CR2 |= UART1_CR2_TIEN;
    }

    // write count bytes from the buffer data
    static void write(const uint8_t *data, uint8_t count)
    {
      while (count != 0)
      {
        putc(*data++);
        --count;
      }
    }

    // transmitter isr, don't call manually
    static void tx_isr();

    // receiver isr, don't call manually
    static void rx_isr();
  private:
    static const uint8_t *sTxData;
    static volatile uint8_t sTxCount;
  };

  // Basic usage:
  // 1. select appropriate init();
  // 2. turnOn() / convert();
  // 3. read() / read8() / read(channel);
  // or just
  // readSingle(channel);
  // which will setup the pin, make one coversion and return the signal.
  // blocks during conversion instead of leaving the ADC running in the background.
  //
  // In general, one conversion takes 28 master clock cycles.
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
      return *((volatile uint16_t *)&(ADC1->DRH));
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
      convert();
      Utility::udelay(3);
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

    // use adc to generate random values
    // fills the buffer until count bytes are reached
    // works with either a floating pin, or a port without pin (for example 10)
    // but floating pin seems to be working better
    static void random(uint8_t *buff, uint8_t count, uint8_t channel);

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
  static void configurepin(uint8_t channel);
  };
} // namespace syn
#endif /* SYN_STM8S_HAL */