#pragma once

#ifdef STM32F103xB
#include "../embos/stm32f103c8/stm32f10x.h"
#endif
#ifdef STM32F401xC
#include "../embos/stm32f401ccu6/stm32f4xx.h"
#endif
#ifdef STM32G030xx
#include "../embos/stm32g030f6p6/stm32g0xx.h"
#endif

#include "../../src/synhal_cfg.h"
#include "mtl.h"

#include "../embos/common/RTOS.h"

namespace syn
{
  class Atomic
  {
  public:
    // disables all interrupts
    Atomic()
    {
      OS_INT_PreserveAndDisableAll(&_state);
    }

    ~Atomic()
    {
      OS_INT_RestoreAll(&_state);
    }

    template <typename Functor, typename Retval>
    static Retval action(Functor fn)
    {
      Atomic a;
      return fn();
    }

    template <typename Functor, typename Retval, typename ParamA>
    static Retval action(Functor fn, ParamA pa)
    {
      Atomic a;
      return fn(pa);
    }

    template <typename Functor, typename Retval, typename ParamA, typename ParamB>
    static Retval action(Functor fn, ParamA pa, ParamB pb)
    {
      Atomic a;
      return fn(pa, pb);
    }

  private:
    unsigned long _state;
  };

  class AtomicWeak
  {
  public:
    // disables all OS interrupts,
    // highest priority, non OS interrupts can still take place
    AtomicWeak()
    {
      OS_INT_Preserve(&_state);
      OS_INT_Disable();
    }

    ~AtomicWeak()
    {
      OS_INT_Restore(&_state);
    }

    template <typename Functor, typename Retval>
    static Retval action(Functor fn)
    {
      AtomicWeak a;
      return fn();
    }

    template <typename Functor, typename Retval, typename ParamA>
    static Retval action(Functor fn, ParamA pa)
    {
      AtomicWeak a;
      return fn(pa);
    }

    template <typename Functor, typename Retval, typename ParamA, typename ParamB>
    static Retval action(Functor fn, ParamA pa, ParamB pb)
    {
      AtomicWeak a;
      return fn(pa, pb);
    }

  private:
    unsigned long _state;
  };

  class Region
  {
  public:
    // disables task switching
    Region()
    {
      OS_EnterRegion();
    }

    ~Region()
    {
      OS_LeaveRegion();
    }

    template <typename Functor, typename Retval>
    static Retval action(Functor fn)
    {
      Region r;
      return fn();
    }

    template <typename Functor, typename Retval, typename ParamA>
    static Retval action(Functor fn, ParamA pa)
    {
      Region r;
      return fn(pa);
    }

    template <typename Functor, typename Retval, typename ParamA, typename ParamB>
    static Retval action(Functor fn, ParamA pa, ParamB pb)
    {
      Region r;
      return fn(pa, pb);
    }
  };

  class Core
  {
  public:
    // priority shall be between 0 and 15 (4 nvic prio bits)
    // lower value means higher priority
    // interrupts with level between 0 and 7 are prohibited to call OS functions
    // the will also nest each other depending on hardware priority, if not disabling irqs
    // OS-Aware interrupts will not nest after the call to Core::enter_isr()
    static void enable_isr(IRQn_Type irq, uint16_t priority)
    {
      OS_ASSERT(priority < 16, ERR_IMPOSSIBRU);
      __NVIC_SetPriority(irq, priority);
      __NVIC_EnableIRQ(irq);
    }

    static void disable_isr(IRQn_Type irq)
    {
      __NVIC_DisableIRQ(irq);
    }

    static void enter_isr()
    {
      OS_INT_Enter();
    }

    static void leave_isr()
    {
      OS_INT_Leave();
    }
  };

  class Mutex
  {
  public:
    void init()
    {
      OS_MUTEX_Create(&_handle);
    }

    void lock()
    {
      OS_MUTEX_LockBlocked(&_handle);
    }

    // returns true if the mutex was acquired
    bool lock(OS_TIME millisec)
    {
      return OS_MUTEX_LockTimed(&_handle, millisec) != 0;
    }

    // returns true if the mutex was acquired
    bool try_lock()
    {
      return OS_MUTEX_Lock(&_handle) != 0;
    }

    void unlock()
    {
      OS_MUTEX_Unlock(&_handle);
    }

  protected:
    OS_MUTEX _handle;
  };

  template <typename Mutex_t>
  class LockGuard
  {
  public:
    LockGuard(Mutex_t &m) : _m(m)
    {
      _m.lock();
    }

    ~LockGuard()
    {
      _m.unlock();
    }

  private:
    Mutex_t &_m;
  };

  class Semaphore
  {
  public:
    void init(uint16_t initial = 0)
    {
      OS_SEMAPHORE_Create(&_handle, initial);
    }

    void take()
    {
      OS_SEMAPHORE_TakeBlocked(&_handle);
    }

    // try to take the semaphore, block for maximum timeout milliseconds
    // return true if we got the semaphore
    bool take(OS_TIME timeout)
    {
      return OS_SEMAPHORE_TakeTimed(&_handle, timeout) != 0;
    }

    // try to take the semaphore, but never block.
    // return true if we got the semaphore
    bool try_take()
    {
      return OS_SEMAPHORE_Take(&_handle) != 0;
    }

    // increase the semaphore
    void give()
    {
      OS_SEMAPHORE_Give(&_handle);
    }

    // increase the semaphore
    void give_max(uint32_t maximum)
    {
      OS_SEMAPHORE_GiveMax(&_handle, maximum);
    }

    // check the current value of the semaphore
    int32_t count() const
    {
      return OS_SEMAPHORE_GetValue(&_handle);
    }

  protected:
    OS_SEMAPHORE _handle;
  };

  // Semaphore as a lightweight signaling device
  // only one task with highest priority will be woken
  // in contrast, Event will wake every task waiting and
  // offers some control regarding the clearing of the event
  class Signal
  {
  public:
    void init()
    {
      OS_SEMAPHORE_Create(&_handle, 0);
    }

    void wait()
    {
      OS_SEMAPHORE_TakeBlocked(&_handle);
    }

    // returns false if the signal didn't fire
    bool wait(OS_TIME timeout)
    {
      return OS_SEMAPHORE_TakeTimed(&_handle, timeout) != 0;
    }

    // test if the signal signal is ready and accept it
    bool try_wait()
    {
      return OS_SEMAPHORE_Take(&_handle) != 0;
    }

    // assert the signal
    void set()
    {
      OS_SEMAPHORE_GiveMax(&_handle, 1);
    }

    // assert the signal from another thread
    // returns false if the signal was already set
    bool set_check()
    {
      Region r;
      bool ret = OS_SEMAPHORE_GetValue(&_handle) != 1;
      set();
      return ret;
    }

    // assert the signal from an interrupt
    // returns false if the the signal was already set
    bool set_check_isr()
    {
      AtomicWeak a;
      bool ret = OS_SEMAPHORE_GetValue(&_handle) != 1;
      set();
      return ret;
    }

  protected:
    OS_SEMAPHORE _handle;
  };

  class Event
  {
  public:
    typedef enum
    {
      bits_manual_clear = OS_EVENT_RESET_MODE_MANUAL,
      bits_auto_clear = OS_EVENT_RESET_MODE_AUTO,
    } Mode;

    void init(Mode mode)
    {
      OS_EVENT_CreateEx(&_handle, mode);
    }

    // mark the event for deletion
    void deinit()
    {
      OS_EVENT_Delete(&_handle);
    }

    // returns if the event is set or deinitialized
    void wait()
    {
      OS_EVENT_GetBlocked(&_handle);
    }

    // returns true if the event is set within the timeout
    bool wait(OS_TIME timeout)
    {
      return OS_EVENT_GetTimed(&_handle, timeout) == 0;
    }

    // returns true if the event is set, but never clears it
    bool try_wait()
    {
      return OS_EVENT_Get(&_handle) != 0;
    }

    // resume any task waiting on the event
    void set()
    {
      OS_EVENT_Set(&_handle);
    }

    // resume any task and instantly reset the event again
    // only needed in manual mode
    void pulse()
    {
      OS_EVENT_Pulse(&_handle);
    }

    // clear the event, only needed in manual mode
    void clear()
    {
      OS_EVENT_Reset(&_handle);
    }

  protected:
    OS_EVENT _handle;
  };

  class SoftTimer
  {
  public:
    SoftTimer()
    {
    }

    virtual ~SoftTimer()
    {
      OS_TIMER_DeleteEx(&_handle);
    }

    void init(OS_TIME period, bool autoreload)
    {
      if (autoreload)
        OS_TIMER_CreateEx(&_handle, reinterpret_cast<OS_TIMER_EX_ROUTINE *>(&SoftTimer::_reload), period, this);
      else
        OS_TIMER_CreateEx(&_handle, reinterpret_cast<OS_TIMER_EX_ROUTINE *>(&SoftTimer::_oneshot), period, this);
    }

    // start the timer and speciy wehter it should restart automatically
    void start()
    {
      OS_TIMER_StartEx(&_handle);
    }

    // resets the timer and starts it
    void restart()
    {
      OS_TIMER_RestartEx(&_handle);
    }

    // return the remaing ticks until fireing
    OS_TIME remaining()
    {
      return OS_TIMER_GetRemainingPeriodEx(&_handle);
    }

    // ends the timer and triggers callback
    void trigger()
    {
      OS_TIMER_TriggerEx(&_handle);
    }

    // stops the timer, can be started again
    void stop()
    {
      OS_TIMER_StopEx(&_handle);
    }

  protected:
    virtual void execute() = 0;
    static void _oneshot(SoftTimer *this_timer);
    static void _reload(SoftTimer *this_timer);

    OS_TIMER_EX _handle;
  };

  template <typename OsType>
  class NamedObj : public OsType
  {
  public:
    void set_name(const char *name)
    {
      OS_DEBUG_SetObjName(&_name, &this->_handle, name);
    }

  private:
    OS_OBJNAME _name;
  };

  // similar to ROS rate, tries to keep the owner at that Hertz
  template <uint32_t Hertz>
  class Rate
  {
  public:
    Rate()
    {
      _lastwake = OS_TIME_GetTicks();
    }

    void sleep()
    {
      _lastwake += SYN_SYSTICK_HERTZ / Hertz;
      OS_TASK_DelayUntil(_lastwake);
    }

  private:
    OS_TIME _lastwake;
  };

  class Thread : private OS_TASK
  {
    typedef void (*thread_run)(void *);

  public:
    // when using a dynamic stack, make sure that the threads dont get terminated
    // else you end up with a memory leak.
    // static stack threads however can just terminate themself if they want
    Thread(const char *name, OS_PRIO priority = 100, uint32_t stacksize = 192,
           uint32_t *pstack = 0, uint8_t timeslice = 20)
    {
      pStack = (OS_REGS OS_STACKPTR *)pstack;
      // use BasePrio field to store stacksize until start() is called
      BasePrio = stacksize;
      Priority = priority;
      TimeSliceReload = timeslice;
      sName = name;
    }

    // creates the task, don't call twice. use resume after suspend instead
    void start()
    {
      uint32_t stacksize = BasePrio;
      BasePrio = Priority;
      if (pStack == 0)
      {
        pStack = (OS_REGS OS_STACKPTR *)new int32_t[stacksize];
      }
      OS_CreateTaskEx(
          this,
          sName,
          Priority,
          (thread_run)&Thread::runner,
          pStack,
          stacksize * sizeof(int32_t), // stacksize is in byte, not in registers
          TimeSliceReload,
          (void *)this);
    }

    ~Thread()
    {
      OS_TASK_Terminate(this);
    }

    void suspend()
    {
      OS_TASK_Suspend(this);
    }

    void resume()
    {
      OS_TASK_Resume(this);
    }

    // this is dangerous for dynamic allocated stacks, 気をつけてね
    void terminate()
    {
      OS_TASK_Terminate(this);
    }

    // send any number for event bits on this thread
    void notify(OS_TASKEVENT bitmask)
    {
      OS_TASKEVENT_Set(this, bitmask);
    }

    // get the events set, but don't clear them
    OS_TASKEVENT checkNotify() const
    {
      return OS_TASKEVENT_Get(this);
    }

    void clearNotify()
    {
      OS_TASKEVENT_Clear(this);
    }

    // suspends always the calling thread!
    static void sleep(OS_TIME millisec)
    {
      if (millisec == 0)
        millisec = 1;
      OS_TASK_Delay(millisec);
    }

    static void usleep(uint16_t usec)
    {
      OS_Delayus(usec);
    }

    // wait for any event
    OS_TASKEVENT waitNotify()
    {
      return OS_TASKEVENT_GetBlocked(0xFFFFFFFF);
    }

    // wait for any event, with a timeout
    OS_TASKEVENT waitNotify(OS_TIME timeout)
    {
      return OS_TASKEVENT_GetTimed(0xFFFFFFFF, timeout);
    }

  private:
    virtual void run() = 0;
    static void runner(Thread *this_thread);
  };

  template <typename Message_t, uint32_t Size>
  class MailBox_Embos
  {
  public:
    void init()
    {
      OS_MAILBOX_Create(&_handle, sizeof(Message_t), Size, &_msgs);
    }

    uint32_t count() const
    {
      return OS_MAILBOX_GetMessageCnt(&_handle);
    }

    // this function is allowed by interrupts
    // returns false if the message was not put (the mailbox full)
    bool try_put(const Message_t &msg)
    {
      if (sizeof(Message_t) > 1)
      {
        return OS_MAILBOX_Put(&_handle, (void *)&msg) == 0;
      }
      else
      {
        return OS_MAILBOX_Put1(&_handle, (const char *)&msg) == 0;
      }
    }

    // this function is only allowed by tasks
    // will block until the message was put into the box
    void put(const Message_t &msg)
    {
      if (sizeof(Message_t) > 1)
      {
        OS_MAILBOX_PutBlocked(&_handle, (void *)&msg);
      }
      else
      {
        OS_MAILBOX_PutBlocked1(&_handle, (const char *)&msg);
      }
    }

    // this function is only allowed by tasks
    // will block until the message was put into the box
    // or return without putting if the timer runs out
    // returns true if the message was placed
    bool put(const Message_t &msg, OS_TIME timeout)
    {
      if (sizeof(Message_t) > 1)
      {
        return OS_MAILBOX_PutTimed(&_handle, (void *)&msg, timeout) == 0;
      }
      else
      {
        return OS_MAILBOX_PutTimed1(&_handle, (const char *)&msg, timeout) == 0;
      }
    }

    // this function is allowed by interrupts
    // puts the message on the front of the queue, not back
    // returns false if the message was not put (the mailbox full)
    bool try_put_front(const Message_t &msg)
    {
      if (sizeof(Message_t) > 1)
      {
        return OS_MAILBOX_PutFront(&_handle, (void *)&msg) == 0;
      }
      else
      {
        return OS_MAILBOX_Put1(&_handle, (const char *)&msg) == 0;
      }
    }

    // this function is only allowed by tasks
    // puts the message on the front of the queue, not back
    // will block until the message was put into the box
    void put_front(const Message_t &msg)
    {
      if (sizeof(Message_t) > 1)
      {
        OS_MAILBOX_PutFrontBlocked(&_handle, (void *)&msg);
      }
      else
      {
        OS_MAILBOX_PutFrontBlocked1(&_handle, (const char *)&msg);
      }
    }

    // returns true if a message was retrieved
    // can be called by interrupts, too
    bool try_get(Message_t &msg)
    {
      if (sizeof(Message_t) > 1)
      {
        return OS_MAILBOX_Get(&_handle, (void *)&msg) == 0;
      }
      else
      {
        return OS_MAILBOX_Get1(&_handle, (char *)&msg) == 0;
      }
    }

    // blocks until a message was received
    void get(Message_t &msg)
    {
      if (sizeof(Message_t) > 1)
      {
        OS_MAILBOX_GetBlocked(&_handle, (void *)&msg);
      }
      else
      {
        OS_MAILBOX_GetBlocked1(&_handle, (char *)&msg);
      }
    }

    // blocks until a message was received
    // or the timeout was reached
    // returns true if a message could be retrieved
    bool get(Message_t &msg, OS_TIME timeout)
    {
      if (sizeof(Message_t) > 1)
      {
        return OS_MAILBOX_GetTimed(&_handle, (void *)&msg, timeout) == 0;
      }
      else
      {
        return OS_MAILBOX_GetTimed1(&_handle, (char *)&msg, timeout) == 0;
      }
    }

    // peek at the first message, if a message is in the buffer
    // this is allowed to be called by interrupts
    // copies the message!
    bool peek(Message_t &msg)
    {
      return OS_MAILBOX_Peek(&_handle, (void *)&msg) == 0;
    }

    // retrieve message without copy, call purge after processing
    // allowed by interrupts
    // true if a message was available
    bool try_get_inplace(Message_t *&pMsg)
    {
      return OS_MAILBOX_GetPtr(&_handle, (void **)&pMsg) == 0;
    }

    // retrieve message without copy, call purge after processing
    // blocks until a message was retrieved
    void get_inplace(Message_t *&pMsg)
    {
      OS_MAILBOX_GetPtrBlocked(&_handle, (void **)&pMsg);
    }

    // has to be called after retrieving an inplace message
    // signals the mailbox that processingis finished and will
    // remove the message from them queue. can be called by interrupts
    void purge()
    {
      OS_MAILBOX_Purge(&_handle);
    }

  private:
    OS_MAILBOX _handle;
    Message_t _msgs[Size];
  };

  template <typename Mail_t, uint32_t Size>
  class MailBox
  {
  public:
    void init()
    {
      _sig_write.init(Size);
      _sig_read.init(0);
      _pread = _pwrite = _mails;
    }

    bool is_full()
    {
      return _sig_write.count() == 0;
    }

    bool try_push(const Mail_t &mail)
    {
      Atomic a;
      bool ret = false;
      if (_sig_write.try_take())
      {
        _write(mail);
        _sig_read.give();
        ret = true;
      }
      return ret;
    }

    void push(const Mail_t &mail)
    {
      _sig_write.take();
      {
        Atomic a;
        _write(mail);
        _sig_read.give();
      }
    }

    bool try_pop(Mail_t &mail)
    {
      Atomic a;
      bool ret = false;
      if (_sig_read.try_take())
      {
        _read(mail);
        _sig_write.give();
        ret = true;
      }
      return ret;
    }

    void pop(Mail_t &mail)
    {
      _sig_read.take();
      {
        Atomic a;
        _read(mail);
        _sig_write.give();
      }
    }

    // alternative API for zero-copy operation
    // however, only safe for single producer & single consumer!

    bool try_reserve(Mail_t **mail)
    {
      OS_ASSERT(write_dirty == false, ERR_FORBIDDEN);
      bool ret = false;
      if (_sig_write.try_take())
      {
        OS_ASSERT(write_dirty = true, ERR_FORBIDDEN);
        *mail = _pwrite;
        ret = true;
      }
      return ret;
    }

    void reserve(Mail_t **mail)
    {
      OS_ASSERT(write_dirty == false, ERR_FORBIDDEN);
      _sig_write.take();
      OS_ASSERT(write_dirty = true, ERR_FORBIDDEN);
      *mail = _pwrite;
    }

    void release()
    {
      OS_ASSERT(write_dirty == true, ERR_FORBIDDEN);
      Atomic a;
      if (++_pwrite == &_mails[Size])
        _pwrite = _mails;
      OS_ASSERT((write_dirty = false) == false, ERR_FORBIDDEN);
      _sig_read.give();
    }

    bool try_peek(Mail_t **mail)
    {
      OS_ASSERT(read_dirty == false, ERR_FORBIDDEN);
      bool ret = false;
      if (_sig_read.try_take())
      {
        OS_ASSERT(read_dirty = true, ERR_FORBIDDEN);
        *mail = _pread;
        ret = true;
      }
      return ret;
    }

    void peek(Mail_t **mail)
    {
      OS_ASSERT(read_dirty == false, ERR_FORBIDDEN);
      _sig_read.take();
      OS_ASSERT(read_dirty = true, ERR_FORBIDDEN);
      *mail = _pread;
    }

    void purge()
    {
      OS_ASSERT(read_dirty == true, ERR_FORBIDDEN);
      Atomic a;
      if (++_pread == &_mails[Size])
        _pread = _mails;
      OS_ASSERT((read_dirty = false) == false, ERR_FORBIDDEN);
      _sig_write.give();
    }

  private:
    void _write(const Mail_t &mail)
    {
      *_pwrite++ = mail;
      if (_pwrite == &_mails[Size])
        _pwrite = _mails;
    }

    void _read(Mail_t &mail)
    {
      mail = *_pread++;
      if (_pread == &_mails[Size])
        _pread = _mails;
    }

    Mail_t *_pread;
    Mail_t *_pwrite;
    Semaphore _sig_read;
    Semaphore _sig_write;
    Mail_t _mails[Size];
#ifdef DEBUG
    bool read_dirty, write_dirty;
#endif
  };

  class System
  {
  public:
    static void init();

    static void nop()
    {
      __asm volatile("nop" ::
                         : "memory");
    }

    // get system tick counter aka milliseconds since start
    // overflow after 24 days, will roll over to negative number?
    static uint32_t milliseconds()
    {
      return OS_TIME_GetTicks();
    }

    // calculate current microseconds, overflow after 71 minutes
    static uint32_t microseconds()
    {
      return OS_TIME_Getus();
    }

    static uint64_t milliseconds64()
    {
      return OS_TIME_ConvertTicks2ms(milliseconds());
    }

    // calculate curent microseconds as 64 bit variable
    // overflow after 584 thousand years
    static uint64_t microseconds64()
    {
      return OS_TIME_Getus64();
    }

    static void delay(uint32_t millis)
    {
      uint32_t tickstart = milliseconds();
      /* Add a freq to guarantee minimum wait */
      ++millis;
      while ((milliseconds() - tickstart) < millis)
        ;
    }

    static void udelay(uint16_t usec)
    {
      OS_Delayus(usec);
    }

    // start operating system core and dont return
    static void spin()
    {
      OS_Start();
    }
  };

  class StopWatch
  {
  public:
    // start a microsecond time measurement
    void start()
    {
      OS_TIME_StartMeasurement(&_time);
    }

    // stop and return result
    uint32_t stop()
    {
      OS_TIME_StopMeasurement(&_time);
      return OS_TIME_GetResultus(&_time);
    }

  private:
    long unsigned int _time;
  };

  /*
  *   START OF HARDWARE DEFINITIONS
  */

  class Gpio
  {
  public:
    Gpio()
    {
      _pPort = 0;
    }

#ifdef STM32G030xx
    // enable /disable gpio ports accodring to mask
    // bit 0 -> port a
    // bit 1 -> port b etc
    static void enable_ports(uint16_t mask)
    {
#ifdef STM32G030xx
      RCC->IOPENR = mask;
#else
      (void)mask;
#endif
    }
#endif

    // port shall be 'A' 'B' or 'C'
    // pin is a number beteween and including 0 and 15
    Gpio(int8_t port, uint8_t pin)
    {
      init(port, pin);
    }

    void init(int8_t port, uint8_t pin)
    {
      OS_ASSERT(pin < 16, ERR_BAD_INDEX);
      _port = port;
      _pin = pin;
      _bitmask = 1 << pin;
      switch (port)
      {
      case 'A':
      case 'a':
        _pPort = GPIOA;
        break;
      case 'B':
      case 'b':
        _pPort = GPIOB;
        break;
      case 'C':
      case 'c':
        _pPort = GPIOC;
        break;
      default:
        OS_ASSERT(true == false, ERR_BAD_PORT_NAME);
      }
    }

    enum Mode
    {
      in_analog = 0x0,
      in_floating = 0x4,
      in_pulldown = 0x8,
      in_pullup = 0xC,
      out_push_pull = 0x1,
      out_open_drain = 0x5,
      out_alt_push_pull = 0x9,
      out_alt_open_drain = 0xD,
    };

    enum Speed
    {
      Input = 0x0,
      MHz_10 = 0x1,
      MHz_2 = 0x2,
      MHz_50 = 0x3,
      MHz_100 = 0x4
    };
#if defined(STM32G030xx)
    enum Alternate
    {
      AF0 = 0,
      AF1 = 1,
      AF2 = 2,
      AF3 = 3,
      AF4 = 4,
      AF5 = 5,
      AF6 = 6,
      AF7 = 7,
      Nop = 8
    };
#elif defined(STM32F103xB)
    enum Alternate
    {
      Nop = 0x0,
      Timer_1_2 = 0x1,
      Timer_3_4_5 = 0x2,
      Timer_9_10_11 = 0x3,
      I2C_1 = 0x4,
      SPI = 0x5,
      USART_1_2 = 0x7,
      USART_6 = 0x8,
      I2C_2 = 0x9,
      OTG_FS = 0xA,
      SDIO_ = 0xC,
      EVENTOUT = 0xF,
      CAN = 0x1,
    };
#elif defined(STM32F401xC)
    enum Alternate
    {
      Nop = 0x0,
      Timer_1_2 = 0x1,
      Timer_3_4_5 = 0x2,
      Timer_9_10_11 = 0x3,
      I2C_1 = 0x4,
      SPI = 0x5,
      USART_1_2 = 0x7,
      USART_6 = 0x8,
      I2C_2 = 0x9,
      OTG_FS = 0xA,
      SDIO_ = 0xC,
      EVENTOUT = 0xF
    };
#endif

    // defaults to floating input (reset state)
    // if set any output mode and speed is not set, defaults to 10MHz
    // if an alternate mode is specified, the alternate parameter needs to be set
    void mode(Mode m, Speed s = Input, Alternate a = Nop)
    {
      if (m == out_alt_open_drain || m == out_alt_push_pull)
      {
        OS_ASSERT(a != Nop, ERR_FORBIDDEN);
      }
      if (m & 0x3 && s == Input)
      {
        s = MHz_10;
      }
#ifdef STM32F103xB
      if (s == MHz_100)
      {
        s = MHz_50;
      }
      if (m == in_pullup)
      {
        set();
        m = in_pulldown;
      }
      else if (m == in_pulldown)
      {
        clear();
      }
      uint16_t cfg = ((uint16_t)m & 0xC) | (uint16_t)s;
      if (_pin < 8)
      {
        uint16_t p = (_pin) << 2;
        _pPort->CRL &= ~(0xF << p);
        _pPort->CRL |= (cfg << p);
      }
      else
      {
        uint16_t p = (_pin - 8) << 2;
        _pPort->CRH &= ~(0xF << p);
        _pPort->CRH |= (cfg << p);
      }
#elif defined(STM32F401xC)
      _pPort->MODER &= ~(0x3 << (_pin * 2));
      _pPort->OSPEEDR &= ~(0x3 << (_pin * 2));
      _pPort->PUPDR &= ~(0x3 << (_pin * 2));
      volatile uint32_t *pAfr;
      uint32_t afr_shift;
      if (_pin < 8)
      {
        pAfr = &_pPort->AFR[0];
        afr_shift = _pin * 4;
      }
      else
      {
        pAfr = &_pPort->AFR[1];
        afr_shift = (_pin - 8) * 4;
      }
      *pAfr &= ~(0xF << afr_shift);
      switch (m)
      {
      case in_analog:
        _pPort->MODER |= (0x3 << (_pin * 2));
        break;
      case out_push_pull:
        _pPort->MODER |= (0x1 << (_pin * 2));
        _pPort->OTYPER &= ~(0x1 << _pin);
        break;
      case in_floating:
        break;
      case out_open_drain:
        _pPort->MODER |= (0x1 << (_pin * 2));
        _pPort->OTYPER |= (0x1 << _pin);
        break;
      case in_pullup:
        _pPort->PUPDR |= (0x1 << (_pin * 2));
        break;
      case in_pulldown:
        _pPort->PUPDR |= (0x2 << (_pin * 2));
        break;
      case out_alt_push_pull:
        _pPort->MODER |= (0x2 << (_pin * 2));
        _pPort->OTYPER &= ~(0x1 << _pin);
        *pAfr |= (a << afr_shift);
        break;
      case out_alt_open_drain:
        _pPort->MODER |= (0x2 << (_pin * 2));
        _pPort->OTYPER |= (0x1 << _pin);
        *pAfr |= (a << afr_shift);
        break;
      }
      switch (s)
      {
      case MHz_10:
        _pPort->OSPEEDR |= (0x1 << (_pin * 2));
        break;
      case MHz_50:
        _pPort->OSPEEDR |= (0x2 << (_pin * 2));
        break;
      case MHz_100:
        _pPort->OSPEEDR |= (0x3 << (_pin * 2));
      case Input:
      case MHz_2:
        break;
      }
#elif defined(STM32G030xx)
      _pPort->MODER &= ~(0x3 << (_pin * 2));
      _pPort->OSPEEDR &= ~(0x3 << (_pin * 2));
      _pPort->PUPDR &= ~(0x3 << (_pin * 2));
      volatile uint32_t *pAfr;
      uint32_t afr_shift;
      if (_pin < 8)
      {
        pAfr = &_pPort->AFR[0];
        afr_shift = _pin * 4;
      }
      else
      {
        pAfr = &_pPort->AFR[1];
        afr_shift = (_pin - 8) * 4;
      }
      *pAfr &= ~(0xF << afr_shift);
      switch (m)
      {
      case in_analog:
        _pPort->MODER |= (0x3 << (_pin * 2));
        break;
      case out_push_pull:
        _pPort->MODER |= (0x1 << (_pin * 2));
        _pPort->OTYPER &= ~(0x1 << _pin);
        break;
      case in_floating:
        break;
      case out_open_drain:
        _pPort->MODER |= (0x1 << (_pin * 2));
        _pPort->OTYPER |= (0x1 << _pin);
        break;
      case in_pullup:
        _pPort->PUPDR |= (0x1 << (_pin * 2));
        break;
      case in_pulldown:
        _pPort->PUPDR |= (0x2 << (_pin * 2));
        break;
      case out_alt_push_pull:
        _pPort->MODER |= (0x2 << (_pin * 2));
        _pPort->OTYPER &= ~(0x1 << _pin);
        *pAfr |= (a << afr_shift);
        break;
      case out_alt_open_drain:
        _pPort->MODER |= (0x2 << (_pin * 2));
        _pPort->OTYPER |= (0x1 << _pin);
        *pAfr |= (a << afr_shift);
        break;
      }
      switch (s)
      {
      case MHz_10:
        _pPort->OSPEEDR |= (0x1 << (_pin * 2));
        break;
      case MHz_50:
        _pPort->OSPEEDR |= (0x2 << (_pin * 2));
        break;
      case MHz_100:
        _pPort->OSPEEDR |= (0x3 << (_pin * 2));
      case Input:
      case MHz_2:
        break;
      }
#else
#error "Unknown chip!"
#endif
    }

    void setWeakPullUpDown(bool pull_up, bool pull_down)
    {
#ifdef STM32F103xB
      (void)pull_up;
      (void)pull_down;
#else
      _pPort->PUPDR &= ~(0x3 << (_pin * 2));
      if(pull_up)
        _pPort->PUPDR |= (0x1 << (_pin * 2));
      if(pull_down)
        _pPort->PUPDR |= (0x2 << (_pin * 2));
#endif
    }

    bool read() const
    {
      return _pPort->IDR & _bitmask;
    }

    void set(bool val = true)
    {
      if(val)
        _pPort->BSRR = _bitmask;
      else
        clear();
    }

    void clear()
    {
#ifdef STM32F103xB
      _pPort->BRR = _bitmask;
#elif defined(STM32F401xC)
      _pPort->BSRR = uint32_t(_bitmask) << 16;
#elif defined(STM32G030xx)
      _pPort->BSRR = uint32_t(_bitmask) << 16;
#else
#error "Unknown chip!"
#endif
    }

    void toggle()
    {
      read() ? clear() : set();
    }

    enum Remap
    {
      spi1_clk_pb3_miso_pb4_mosi_pb5 = 0x0001,
      i2c1_scl_pb8_sda_pb9 = 0x0002,
      uart1_tx_pb6_rx_pb7 = 0x0004,
      uart2_tx_pd5_rx_pd6 = 0x0008,
      uart3_tx_c10_rx_c11 = 0x0010,
      uart3_tx_d8_rx_d9 = 0x0030,
      tim1_remap_part = 0x0040,
      tim1_remap_full = 0x00C0,
      tim2_ch1_pa15_ch2_pb3 = 0x0100,
      tim2_ch3_pb10_ch4_pb11 = 0x0200,
      tim2_ch1_pa15_ch2_pb3_ch3_pb10_ch4_pb11 = 0x0300,
      tim3_remap_part = 0x0800,
      tim3_remap_full = 0x0C00,
      tim4_remap = 0x1000,
      can_rx_pb8_tx_pb9 = 0x4000,
      can_rx_pd0_tx_pd1 = 0x6000,
      osc_inout_pd0_pd1 = 0x8000,
      tim5_lsi_ch4 = 0x10000,
      adc1_etr_inj_tim8_ch4 = 0x20000,
      adc1_etr_reg_tim8_trgo = 0x40000,
      adc2_etr_inh_tim8_ch4 = 0x80000,
      adc2_etr_reg_tim8_trgo = 0x100000,
      swj_no_nrst = 0x1000000,
      swj_jtag_disable = 0x2000000,
      swj_all_disable = 0x4000000
    };

    static void remap(Remap map)
    {
#ifdef STM32F103xB
      AFIO->MAPR |= (uint32_t)map;
#elif defined(STM32F401xC)
      (void)(map);
#elif defined(STM32G030xx)
      (void)(map);
#else
#error "Unknown chip!"
#endif
    }

    static void clear_remap(Remap map)
    {
#ifdef STM32F103xB
      AFIO->MAPR &= ~map;
#elif defined(STM32F401xC)
      (void)(map);
#elif defined(STM32G030xx)
      (void)(map);
#else
#error "Unknown chip!"
#endif
    }

    static bool is_remapped(Remap map)
    {
#ifdef STM32F103xB
      return AFIO->MAPR & map;
#elif defined(STM32F401xC)
      (void)(map);
      OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
      return false;
#elif defined(STM32G030xx)
      (void)(map);
      OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
      return false;
#else
#error "Unknown chip!"
#endif
    }

  private:
    GPIO_TypeDef *_pPort;
    uint16_t _bitmask;
    int8_t _port;
    uint8_t _pin;
  };

  class Led
  {
  public:
    Led() : _led('C', 13)
    {
      _led.mode(syn::Gpio::out_push_pull, syn::Gpio::MHz_2);
    }

    Led(char port, uint8_t pin) : _led(port, pin)
    {
      _led.mode(syn::Gpio::out_push_pull, syn::Gpio::MHz_2);
    }

    void on()
    {
      _led.clear();
    }

    void off()
    {
      _led.set();
    }

    void toggle()
    {
      _led.toggle();
    }

  private:
    Gpio _led;
  };

  class Exti
  {
  public:
    // line is equivalent to port pin number
    // port shall be 'A' 'B' or 'C'
    // priority shall be between 0 and 15
    // lower value means higher priority
    // interrupts with level between 0 and 7 are prohibited to call OS functions
    // select the correct exti type in the synhal_cfg
    static void enable(uint16_t line, char port, bool rising, bool falling, uint32_t priority = 8)
    {
      OS_ASSERT('A' <= port && port <= 'C', ERR_BAD_PORT_NAME);
      uint8_t extiselector = port - 'A';
      uint16_t extiafionum = (line % 4) * 4;
      uint16_t extiafioreg = line / 4;
#ifdef STM32F103xB
      AFIO->EXTICR[extiafioreg] &= ~(0xF << extiafionum);
      AFIO->EXTICR[extiafioreg] |= (extiselector << extiafionum);
      {
        Atomic a;
        EXTI->IMR |= (1 << line);
        if (rising)
          EXTI->RTSR |= (1 << line);
        if (falling)
          EXTI->FTSR |= (1 << line);
      }
      if (line == 0)
      {
        Core::enable_isr(EXTI0_IRQn, priority);
      }
      else if (line == 1)
      {
        Core::enable_isr(EXTI1_IRQn, priority);
      }
      else if (line == 2)
      {
        Core::enable_isr(EXTI2_IRQn, priority);
      }
      else if (line == 3)
      {
        Core::enable_isr(EXTI3_IRQn, priority);
      }
      else if (line == 4)
      {
        Core::enable_isr(EXTI4_IRQn, priority);
      }
      else if (line < 10)
      {
        Core::enable_isr(EXTI9_5_IRQn, priority);
      }
      else if (line < 16)
      {
        Core::enable_isr(EXTI15_10_IRQn, priority);
      }
      else
      {
        OS_ASSERT(true == false, ERR_BAD_INDEX);
      }
#elif defined(STM32F401xC)
      SYSCFG->EXTICR[extiafioreg] &= ~(0xF << extiafionum);
      SYSCFG->EXTICR[extiafioreg] |= (extiselector << extiafionum);
      {
        Atomic a;
        EXTI->IMR |= (1 << line);
        if (rising)
          EXTI->RTSR |= (1 << line);
        if (falling)
          EXTI->FTSR |= (1 << line);
      }
      if (line == 0)
      {
        Core::enable_isr(EXTI0_IRQn, priority);
      }
      else if (line == 1)
      {
        Core::enable_isr(EXTI1_IRQn, priority);
      }
      else if (line == 2)
      {
        Core::enable_isr(EXTI2_IRQn, priority);
      }
      else if (line == 3)
      {
        Core::enable_isr(EXTI3_IRQn, priority);
      }
      else if (line == 4)
      {
        Core::enable_isr(EXTI4_IRQn, priority);
      }
      else if (line < 10)
      {
        Core::enable_isr(EXTI9_5_IRQn, priority);
      }
      else if (line < 16)
      {
        Core::enable_isr(EXTI15_10_IRQn, priority);
      }
      else
      {
        OS_ASSERT(true == false, ERR_BAD_INDEX);
      }
#elif defined(STM32G030xx)
      EXTI->EXTICR[extiafioreg] &= ~(0xF << extiafionum);
      EXTI->EXTICR[extiafioreg] |= (extiselector << extiafionum);
      {
        Atomic a;
        EXTI->IMR1 |= (1 << line);
        if (rising)
          EXTI->RTSR1 |= (1 << line);
        if (falling)
          EXTI->FTSR1 |= (1 << line);
      }
      if (line == 0 || line == 1)
      {
        Core::enable_isr(EXTI0_1_IRQn, priority);
      }
      else if (line == 2 || line == 3)
      {
        Core::enable_isr(EXTI2_3_IRQn, priority);
      }
      else if (line < 16)
      {
        Core::enable_isr(EXTI4_15_IRQn, priority);
      }
      else
      {
        OS_ASSERT(true == false, ERR_BAD_INDEX);
      }
#else
#error "Unknown chip"
#endif
    }

    static void disable(uint16_t line)
    {
      Atomic a;
#if defined(STM32G030xx)
      EXTI->IMR1 &= ~(1 << line);
      EXTI->RTSR1 &= ~(1 << line);
      EXTI->FTSR1 &= ~(1 << line);
#else 
      EXTI->IMR &= ~(1 << line);
      EXTI->RTSR &= ~(1 << line);
      EXTI->FTSR &= ~(1 << line);
#endif
    }

    // generate a software interrupt on the line
    static void sw_trigger(uint16_t line)
    {
#if defined(STM32G030xx)
      EXTI->SWIER1 = (1 << line);
#else
      EXTI->SWIER = (1 << line);
#endif
    }

    // check wether this exti got triggered
    // returns value != 0 if triggered
    // on rising / falling edge seperate devices, returns 1 for falling
    // 2 for rising or 3 for both edges
    static uint16_t is_set(uint16_t line)
    {
      uint16_t ret = 0;
#if defined(STM32G030xx)
      uint16_t mask = (1 << line);
      if (EXTI->FPR1 & mask )
      {
        ret = 1;
      }
      if (EXTI->RPR1 & mask)
      {
        ret |= 2;
      }
#else
      if(EXTI->PR & (1 << line))
      {
        return 1;
      }
#endif
      return ret;
    }

    // clear the pending bit for the irq line
    // for falling and rising edge devices, the line to clear
    // should be passed as the same value obtained by is_set function
    // val = 1 falling, val = 2 rising, val = 3 both edges
    static void clear(uint16_t line, uint16_t val=1)
    {
      uint16_t mask = (1 << line);
#if defined(STM32G030xx)
      if(val & 0x1)
      {
        EXTI->FPR1 = mask;
      }
      if(val & 0x2)
      {
        EXTI->RPR1 = (1 << line);
      }
#else
      if(val & 0x1)
      {
        EXTI->PR = mask;
      }
#endif
    }
  };

  class Adc
  {
    static const uint32_t ADC_CHANNEL_COUNT = 10;

  public:
    // initialize and start the ADC, will read channels 0 to 9 in a loop
    // and update the static channel save accordingly. one rotation takes about
    // 45 micro seconds
    // uses DMA_channel_1 to copy values
    static void init();

    // set the pin to analog reading mode
    // channel can be any number between and including 0 and 9
    // 0 .. 7 unlocks pins of Port A 0 .. 7
    // 8 & 9 unlocks pins 0 & 1 of Port B
    static void enable(uint16_t channel)
    {
      OS_ASSERT(channel < ADC_CHANNEL_COUNT, ERR_BAD_INDEX);
      if (channel < 8)
      {
        Gpio pin('A', channel);
        pin.mode(Gpio::in_analog, Gpio::Input);
      }
      else
      {
        Gpio pin('B', channel - 8);
        pin.mode(Gpio::in_analog, Gpio::Input);
      }
    }

    // return the last measured value of the specified channel
    static uint16_t read(uint16_t channel)
    {
      OS_ASSERT(channel < ADC_CHANNEL_COUNT, ERR_BAD_INDEX);
      return _channels[channel];
    }

  private:
    static uint16_t _channels[ADC_CHANNEL_COUNT];
  };

  class Dma
  {
  public:
    void init(uint16_t stream)
    {
#ifdef STM32F103xB
      --stream;
      OS_ASSERT(stream < 7, ERR_BAD_INDEX);
      _pChannel = (DMA_Channel_TypeDef*)((uint32_t*)DMA1_Channel1 + stream * 5);
#endif
#ifdef STM32F401xC
      OS_ASSERT(stream < 16, ERR_BAD_INDEX);
      _number = stream;
      if (stream < 8)
      {
        _pStream = DMA1_Stream0 + stream;
      }
      else
      {
        stream -= 8;
        _pStream = DMA2_Stream0 + stream;
      }
#endif
    }

    // stop operation of the dma
    void stop()
    {
#ifdef STM32F103xB
      _pChannel->CCR &= ~DMA_CCR1_EN;
#endif
#ifdef STM32F401xC
      _pStream->CR &= ~DMA_SxCR_EN;
#endif
    }

    // the channel attribute is for stm32f401
    // it refers to channel of the specific stream to use
    void start(uint32_t channel = 0)
    {
#ifdef STM32F103xB
      (void)channel;
      _pChannel->CCR |= DMA_CCR1_EN;
#endif
#ifdef STM32F401xC
      OS_ASSERT(channel < 8, ERR_BAD_INDEX);
      _pStream->CR |= (channel << 25) | DMA_SxCR_EN;
#endif
    }

    // cylcic reading from a peripheral to memory. periheral stays the same, memory gets incremented
    // count is the number of transfers, not the number of bytes!
    template <typename Peri_t, typename Mem_t>
    void cyclicP2M(Peri_t *src, Mem_t *dst, uint16_t count)
    {
#ifdef STM32F103xB
      _pChannel->CCR = 0; // stop the dma before setting anything
      uint16_t psize = sizeof(Peri_t) >> 1;
      uint16_t msize = sizeof(Mem_t) >> 1;
      _pChannel->CCR = (msize << 10) | (psize << 8) | DMA_CCR1_MINC | DMA_CCR1_CIRC;
      _pChannel->CNDTR = count;
      _pChannel->CMAR = (uint32_t)dst;
      _pChannel->CPAR = (uint32_t)src;
#endif
#ifdef STM32F401xC
      _pStream->CR = 0;
      _pStream->FCR = 0;
      _pStream->NDTR = count;
      _pStream->PAR = (uint32_t)src;
      _pStream->M0AR = (uint32_t)dst;
      uint16_t psize = sizeof(Peri_t) >> 1;
      uint16_t msize = sizeof(Mem_t) >> 1;
      _pStream->CR = (msize << 13) | (psize << 11) | DMA_SxCR_MINC | DMA_SxCR_CIRC;
#endif
    }

    // cylcic reading from memory to peripheral. periheral stays the same, memory gets incremented
    // count is the number of transfers, not the number of bytes!
    template <typename Peri_t, typename Mem_t>
    void cyclicM2P(Mem_t* src, Peri_t* dst, uint16_t count)
    {
#ifdef STM32F103xB
      _pChannel->CCR = 0; // stop the dma before setting anything
      uint16_t psize = sizeof(Peri_t) >> 1;
      uint16_t msize = sizeof(Mem_t) >> 1;
      _pChannel->CCR = (msize << 10) | (psize << 8) | DMA_CCR1_MINC | DMA_CCR1_CIRC;
      _pChannel->CNDTR = count;
      _pChannel->CMAR = (uint32_t)src;
      _pChannel->CPAR = (uint32_t)dst;
#endif
#ifdef STM32F401xC
      _pStream->CR = 0;
      _pStream->FCR = 0;
      _pStream->NDTR = count;
      _pStream->PAR = (uint32_t)src;
      _pStream->M0AR = (uint32_t)dst;
      uint16_t psize = sizeof(Peri_t) >> 1;
      uint16_t msize = sizeof(Mem_t) >> 1;
      _pStream->CR = (msize << 13) | (psize << 11) | DMA_SxCR_MINC | DMA_SxCR_CIRC;
#endif
    }

    // oneshot memory to peripheral.
    // count is the number of transfers, not the number of bytes!
    template <typename Mem_t, typename Peri_t>
    void oneshotM2P(Mem_t *src, Peri_t *dst, uint16_t count)
    {
#ifdef STM32F103xB
      _pChannel->CCR &= (DMA_CCR1_TEIE | DMA_CCR1_HTIE | DMA_CCR1_TCIE); // stop the dma before setting anything, keep interrupts
      uint16_t psize = sizeof(Peri_t) >> 1;
      uint16_t msize = sizeof(Mem_t) >> 1;
      _pChannel->CCR |= (msize << 10) | (psize << 8) | DMA_CCR1_MINC | DMA_CCR1_DIR;
      _pChannel->CNDTR = count;
      _pChannel->CMAR = (uint32_t)src;
      _pChannel->CPAR = (uint32_t)dst;
#endif
    }

    uint16_t count()
    {
      return _pChannel->CNDTR;
    }

#ifdef STM32F401xC
    enum BurstSize
    {
      Single_Transfer = 0,
      Burst_of_4_beats = 1,
      Burst_of_8_beats = 2,
      Burst_of_16_beats = 3
    };

    enum FifoSize
    {
      FIFO_1_4th = 0,
      FIFO_HALF = 1,
      FIFO_3_4th = 2,
      FIFO_FULL = 3
    };

    // the fifo can contain up to 4 words (16 bytes)
    // care has to be taken to only set possible values
    void enableBurstMode(BurstSize memory, BurstSize peripheral, FifoSize size)
    {
      _pStream->CR |= (memory << 23) | (peripheral << 21);
      _pStream->FCR = DMA_SxFCR_DMDIS | size;
    }
#endif

#ifdef STM32F103xB
    static const uint16_t IRQ_STATUS_ERROR = 0x4;
    static const uint16_t IRQ_STATUS_HALF = 0x2;
    static const uint16_t IRQ_STATUS_FULL = 0x1;
#endif
#ifdef STM32F401xC
    static const uint16_t IRQ_STATUS_FULL = 0x20;
    static const uint16_t IRQ_STATUS_HALF = 0x10;
    static const uint16_t IRQ_STATUS_ERROR = 0x08;
    static const uint16_t IRQ_STATUS_DIRECT_ERR = 0x04;
    static const uint16_t IRQ_STATUS_FIFO_ERR = 0x01;
#endif
    void enableIrq(uint16_t irq_status_mask, uint16_t priority = 8);

  private:
#ifdef STM32F103xB
    DMA_Channel_TypeDef *_pChannel;
#endif
#ifdef STM32F401xC
    DMA_Stream_TypeDef *_pStream;
    uint16_t _number;
#endif
  };

  class Timer
  {
  public:
    void init(uint16_t number)
    {
      _tclk = 0;
      _number = number;
      switch (number)
      {
#ifdef STM32F103xB
      case 1:
        _pTimer = TIM1;
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        break;
      case 2:
        _pTimer = TIM2;
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        break;
      case 3:
        _pTimer = TIM3;
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        break;
      case 4:
        _pTimer = TIM4;
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        break;
#endif
#ifdef STM32F401xC
      case 1:
        _pTimer = TIM1;
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        break;
      case 2:
        _pTimer = TIM2;
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        break;
      case 3:
        _pTimer = TIM3;
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        break;
      case 4:
        _pTimer = TIM4;
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        break;

      case 5:
        _pTimer = TIM5;
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        break;
#endif
      default:
        OS_ASSERT(true == false, ERR_BAD_PORT_NAME);
      }
    }

    void clear_arr()
    {
      _pTimer->ARR = 0;
    }

    uint16_t value() const
    {
      return _pTimer->CNT;
    }

    uint16_t arr() const
    {
      return _pTimer->ARR;
    }

    uint32_t fupdate() const
    {
      return _tclk / arr();
    }

    uint16_t capture_1() const
    {
      return _pTimer->CCR1;
    }

    uint16_t capture_2() const
    {
      return _pTimer->CCR2;
    }

    uint16_t capture_3() const
    {
      return _pTimer->CCR3;
    }

    uint16_t capture_4() const
    {
      return _pTimer->CCR4;
    }

    void ramp(uint32_t target_hz, uint16_t delta);

    uint32_t setStepperHz(uint32_t hz)
    {
      if(hz == 0)
      {
        _pTimer->ARR = 0;
        _pTimer->EGR |= TIM_EGR_UG;
        return 0;
      }

      uint32_t arr = (_tclk / 2) / hz;
      if(arr > UINT16_MAX)
      {
        arr = UINT16_MAX;
      }
      else if (arr == 0)
      {
        arr = 1;
      }
      
      uint32_t prevarr = _pTimer->ARR;
      _pTimer->ARR = arr;
      if(prevarr == 0)
      {
        // generate an update event to push the values from shadow registers in real registers
        // this is necessary because if the ARR is zero, the timers are stopped and wont generate
        // the event that loads the new ARR value from shadow registers
        _pTimer->EGR |= TIM_EGR_UG;
      }
      return (_tclk / 2) / arr;
    }

    void configStepper();

    // configure the timer for pwm out by setting the prescaler, reload and pwm compare startvalue
    // Timers are running with 72MHz on stm32f103 and 84MHz on stm32f401
    // the prescalers minimum value is 1, which is added internally (e.g. setting a value of 1 will result in 36MHz timer ticks (divison by 2))
    void configPwm(uint16_t prescaler, uint16_t reload, uint16_t startvalue);

    // configures the timer to enbale 50Hz rc pwm output on the channels
    void configRcPwm()
    {
      // PWM should be 50 Hz. Resolution for RC PWM will be 600 to 2400.
      // 0 = 0 us; 1000 = 1000 usec; -> max pulselength 20ms = 20000
      // the prescaler needs to bring the 72MHz down to 50Hz together with the reload register
      // we divide the counter by 72. So it runs at 1MHz
      // set the startvalue to 1500 for center pwm
      configPwm((SystemCoreClock / 1000000) - 1, 20000, 1500);
    }

    // enables the corresponding pin to perfom pwm output
    // lower speed at the gpio is desierable for some reason (less jittery / stronger signal)
    void enablePwm(int8_t port, uint8_t pinnum, uint16_t channel, Gpio::Speed speed = Gpio::MHz_2);
    void enablePwm(syn::Gpio& pin, uint16_t channel, Gpio::Speed speed = Gpio::MHz_2);

    // check if the pwm channel output is enabled
    bool is_pwm_out(uint16_t channel)
    {
      --channel;
      OS_ASSERT(channel < 4, ERR_BAD_INDEX);
      channel *= 4;
      return _pTimer->CCER & (1 << channel);
    }

    // set the corresponding output compare register
    void setPwm(uint16_t channel, uint16_t value)
    {
      --channel;
      OS_ASSERT(channel < 4, ERR_BAD_INDEX);
      uint16_t *reg = (uint16_t *)(((uint32_t*)(&(_pTimer->CCR1))) + channel);
      *reg = value;
    }

    // change the reload value on the fly
    void setReload(uint16_t value)
    {
      _pTimer->ARR = value;
    }

    uint16_t getReload() const
    {
      return _pTimer->ARR;
    }

    // value is the time in microseconds
    // rc signals typically work between 1000 and 2000 us
    // the function enforces a mamximum of 2400 and minimum of 600
    void setRcPwm(uint16_t channel, uint16_t value)
    {
      if (value > 2399)
        value = 2399;
      else if (value < 600)
        value = 600;
      setPwm(channel, value);
    }

    enum InputFilter
    {
      Clk_1 = 0x00,
      Clk_2 = 0x01,
      Clk_4 = 0x02,
      Clk_8 = 0x03
    };

    // configure simple input capturing
    void configInputCapture(uint16_t prescaler, uint16_t reload, InputFilter filter);

    // configure pwm input capturing
    // defaults to ch1 & ch3 capture rising, ch2 & ch4 capture falling
    // if mapping is true, instead of capturing on ch1 and ch3 input signals,
    // the timer will use ch2 and ch4 as it's inputs
    void configPwmCapture(uint16_t prescaler, uint16_t reload, InputFilter filter, bool mapping);

    // setup pin for input capture
    // port shall be 'A' 'B' or 'C'
    // pin is a number beteween and including 0 and 15
    void enableInput(int8_t port, uint8_t pinnum, bool pulldown, bool pullup);

    // enable a callback for the timer
    // triggers at each update event
    // and return void upon completition
    void enableCallback(uint16_t priority = 8);

    uint32_t irq_status() const
    {
      return _pTimer->SR;
    }

    // clear interrupt status register. If its not done, will be an interrupt loop
    void clear_irq(uint32_t mask = 0xFFFF)
    {
      _pTimer->SR &= ~mask; 
    }

    // enable DMA request at Update Event
    // also set from which register to start (offset)
    // and the burst count for how many registers to transfers
    // returns the dma read register to set the dma
    // calculate the base reg by offset / 4 (as in the register map)
    // the first CCR register is number 13 (offset 0x34)
    volatile uint16_t *enableDmaUpdate(uint16_t base_reg, uint16_t burst_count);

    // start the timer again after stopping, better call only after having initialized the timers
    void start()
    {
      _pTimer->CR1 |= TIM_CR1_CEN;
    }

    // stop the timer from running
    void stop()
    {
      _pTimer->CR1 &= !TIM_CR1_CEN;
    }

    // stop timer when debugging, careful with RC pwm
    void stopForDebug();

  private:
    TIM_TypeDef *_pTimer;
    uint32_t _tclk;
    uint16_t _number;
  };

  class Usart
  {
    typedef  void(*usart_write_t)(const uint8_t* pdata, uint16_t count);
    typedef  uint16_t(*usart_read_t)(uint8_t* pdata, uint16_t count, uint32_t timeout);
    typedef  uint16_t(*usart_avail_t)();
  public:
    enum eBaudrate {
      b9600,
      b19200,
      b57600,
      b115200,
      b230400,
      b460800,
      b921600
    };
    void init(uint16_t dev, eBaudrate baudrate, bool halfduplex);

    void write(const uint8_t* pdata, uint16_t count)
    {
      _write(pdata, count);
    }

    uint16_t available()
    {
      return _avail();
    }

    uint16_t read(uint8_t* data, uint16_t count, uint32_t timeout = 0)
    {
      return _read(data, count, timeout);
    }
  private:
    usart_write_t _write;
    usart_read_t _read;
    usart_avail_t _avail;
  };


  class SpiMaster
  {
  public:
    // port -> Spi Number 1, 2, 3
    // frequency -> in MHz
    // clock_polarity -> false = idle_low
    // clock_phase -> false = capture on first edge, true = capture on second edge
    // transfer_size -> false = 8bit, true 16bit
    // hardware_slave_sel -> false = user takes care of nss pin
    void init(uint16_t port, uint32_t frequency, bool clock_polarity, bool clock_phase, bool transfer_size, bool hardware_slave_sel);

    bool busy_tx(const uint8_t *pbuffer, uint16_t size);
    bool busy_tx(const uint16_t *pbuffer, uint16_t size);

    bool busy_bidi(uint8_t *pbuffer, uint16_t size);

    // writes start address to spi device, than continues to read into buffer for size
    bool busy_read_regs(uint8_t startaddress, uint8_t* pbuffer, uint16_t size);

  private:
    SPI_TypeDef *_pSpi;
  };

  class I2cMaster
  {
  public:
    I2cMaster();
    I2cMaster(uint16_t port, uint8_t address);

    void init(uint16_t port, uint8_t address, bool remap=true);
    bool write(uint8_t *data, uint16_t size);
    bool read(uint8_t *data, uint16_t size);
    
  private:
    static void init_runtime_remap_i2c1();
    static void runtime_remap_i2c1(bool remap);

    void *_pdev;
    uint8_t _address;
    uint8_t _remapped;
  };

  class UsbRpc
  {
  public:
    class Packet;
    class Handler;

    static void init();

    static bool write(const uint8_t *data, uint16_t size, uint32_t timeout = 0);

  private:
    static void _start_usb();
    static void _enable_rx();
  };

  class VirtualEeprom
  {
  public:
    static const uint32_t VE_FLASH_BASE = 0x8000000;
    static const uint32_t VE_PAGE_SIZE = 0x400;
    static const uint32_t VE_BANK_HIGHEST = 62;
    static const uint32_t VE_BANK_COUNT = 8;

  private:
    class Bank
    {
    public:
      Bank(uint16_t number);
      bool read(uint16_t vaddress, uint8_t *value, uint16_t size);
      bool write(uint16_t vaddress, const uint8_t *value, uint16_t size);

    private:
      static void initPage(uint16_t *pageaddress, bool use_lock = true);
      // initialize this virtual eeprom bank
      void format();
      // copy relevant data from one page to the other, than erase the source
      void copy(uint16_t *src, uint16_t *dst, bool use_lock = true);
      // update the write pointer
      void updateWritePointer();

      class Entry
      {
      public:
        const uint16_t vaddress;
        const uint8_t loc_prev;
        const uint8_t size_this;
        const uint16_t data; // this can be more than one data field, but at least one

        Entry *getPrev();
        Entry *getNext();
        bool write(Entry *previous, uint16_t vaddress, const uint8_t *value, uint16_t size, bool use_lock);
        void read(uint8_t *value, uint16_t size);
      };

      Mutex _mutex;
      uint16_t *_pages[2];
      uint16_t *_active;
      Entry *_wpointer;
    };

    class Flash
    {
    public:
      // write to specific address and check for success
      static bool write(uint16_t *address, uint16_t value, bool use_lock = true);
      // erase the entire page
      static void erasePage(uint16_t *pageaddress, bool use_lock = true);
      // protected bank creator
      static Bank *getBank(uint16_t number);

    private:
      Flash();
      static Flash _instance;
      // storage for the available banks
      Bank *_banks[VE_BANK_COUNT];
    };

  public:
    // bank can be any value between 0 to BANK_COUNT
    // we only allow to use the last 16kb of the flash for memory
    // one bank consists of 2 pages with 1kb each. half of the space is needed
    // for managing the virtual adresses. Leading to up to 4kb of actual storage
    VirtualEeprom(uint16_t bank)
    {
      OS_ASSERT(bank < VE_BANK_COUNT, ERR_BAD_INDEX);
      _pbank = Flash::getBank(bank);
      OS_ASSERT(_pbank, ERR_NULL_POINTER);
    }

    template <typename T>
    bool read(uint16_t vaddress, T &value)
    {
      return _pbank->read(vaddress, (uint8_t *)&value, sizeof(T));
    }

    template <typename T>
    bool write(uint16_t vaddress, const T &value)
    {
      return _pbank->write(vaddress, (const uint8_t *)&value, sizeof(T));
    }

  private:
    // bank of this eeprom
    Bank *_pbank;
  };

  class CANopenNode {
  public:
    // initialize CAN interface and CANopenNode stack
    static int32_t init(uint8_t desired_id, uint16_t baudrate_k);
    // slow process messages, can be in a loop with other code
    static void process(syn::Gpio &led_green, syn::Gpio& led_red);
  private:
    // restart can hadware and node
    static int32_t reset_com();
    // Process that runs elevated from tasks every millisecond
    class CANopenSYNC : public syn::SoftTimer
    {
    public:
      void execute();
    };

    static CANopenSYNC _synctimer;
  };
} // namespace syn
