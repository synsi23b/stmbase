#pragma once

#ifdef STM32F103xB
#include "../embos/stm32f103c8/stm32f103xb.h"
#define EMBOS_IRQN(x) x + 16
#endif

#include "../../synhal_cfg.h"
#include "mtl.h"

#include "../embos/common/RTOS.h"

namespace syn
{
  class Atomic
  {
  public:
    // disables all OS interrupts,
    // highest priority, non OS interrupts can still take place
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
    // priority shall be between 0 and 255
    // lower value means higher priority
    // interrupts with level between 0 and 127 are prohibited to call OS functions
    static void enable_isr(IRQn_Type irq, uint16_t priority)
    {
      OS_ARM_ISRSetPrio(EMBOS_IRQN(irq), priority);
      OS_ARM_EnableISR(EMBOS_IRQN(irq));
    }

    static void disable_isr(IRQn_Type irq)
    {
      OS_ARM_DisableISR(EMBOS_IRQN(irq));
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
      OS_MUTEX_CREATE(&_handle);
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
    void init()
    {
      OS_SEMAPHORE_Create(&_handle, 0);
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
    int32_t check() const
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
      bits_manual_clear_logic_or = OS_EVENT_RESET_MODE_MANUAL,
      bits_auto_clear_logic_or = OS_EVENT_RESET_MODE_AUTO,
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
    // create the timer and speciy wehter it should restart automatically
    SoftTimer(OS_TIME period, bool autoreload)
    {
      if (autoreload)
        OS_TIMER_CreateEx(&_handle, reinterpret_cast<OS_TIMER_EX_ROUTINE *>(&SoftTimer::_reload), period, this);
      else
        OS_TIMER_CreateEx(&_handle, reinterpret_cast<OS_TIMER_EX_ROUTINE *>(&SoftTimer::_oneshot), period, this);
    }

    virtual ~SoftTimer()
    {
      OS_TIMER_DeleteEx(&_handle);
    }

    // start the timer
    void start()
    {
      OS_TIMER_StartEx(&_handle);
    }

    // resets the timer and starts it
    void restart()
    {
      OS_TIMER_RestartEx(&_handle);
    }

    // return the remaing ticks before fireing
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

  class Thread
  {
    typedef void (*thread_run)(void *);

  public:
    // when using a dynamic stack, make sure that the threads dont get terminated
    // else you end up with a memory leak.
    // static stack threads however can just terminate themself if they want
    Thread(const char *name, OS_PRIO priority = 100, uint16_t stacksize = 512,
           OS_REGS *pstack = 0, uint8_t timeslice = 20)
    {
      _handle.pStack = pstack;
      _handle.Timeout = stacksize;
      _handle.Priority = priority;
      _handle.TimeSliceReload = timeslice;
      _handle.Name = name;
    }

    // creates the task, don't call twice. use resume after suspend instead
    void start()
    {
      OS_REGS *pstack = _handle.pStack;
      if (pstack == 0)
      {
        pstack = new OS_REGS[_handle.Timeout];
      }
      OS_CreateTaskEx(
          &_handle,
          _handle.Name,
          _handle.Priority,
          reinterpret_cast<thread_run>(Thread::runner),
          pstack, _handle.Timeout,
          _handle.TimeSliceReload,
          this);
    }

    virtual ~Thread()
    {
      OS_TASK_Terminate(&_handle);
    }

    void suspend()
    {
      OS_TASK_Suspend(&_handle);
    }

    void resume()
    {
      OS_TASK_Resume(&_handle);
    }

    // this is dangerous for dynamic allocated stacks, 気をつけてね
    void terminate()
    {
      OS_TASK_Terminate(&_handle);
    }

    // send any number for event bits on this thread
    void notify(OS_TASKEVENT bitmask)
    {
      OS_TASKEVENT_Set(&_handle, bitmask);
    }

    // get the events set, but don't clear them
    OS_TASKEVENT checkNotify() const
    {
      return OS_TASKEVENT_Get(&_handle);
    }

    void clearNotify()
    {
      OS_TASKEVENT_Clear(&_handle);
    }

    // disabled task switching but not interrupts
    static void prevent_taskswitch()
    {
      OS_EnterRegion();
    }

    static void restore_taskswitch()
    {
    }

  protected:
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
    static void runner(const void *this_thread);

    OS_TASK _handle;
  };

  template <typename Message_t, uint32_t Size>
  class MailBox
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

  template <typename Value_t, uint32_t Size, typename Synchro_Primitive_t>
  class Ringbuffer
  {
  public:
    uint16_t in_avail() const
    {
      return _buffer.in_avail();
    }

    uint16_t remainingSpace() const
    {
      return _buffer.remainingSpace();
    }

    bool full() const
    {
      return _buffer.full();
    }

    // look at some fancy values in the future
    bool peek(Value_t &val, uint32_t offset = 0)
    {
      return _buffer.peek(val, offset);
    }

    // remove up to count values from the buffer
    void flush(uint32_t count = Size)
    {
      Synchro_Primitive_t a;
      _buffer.flush(count);
    }

    // returns false if the buffer was empty when attempting to read
    bool pop(Value_t &v)
    {
      Synchro_Primitive_t a;
      bool ret = _buffer.pop(v);
      return ret;
    }

    // returns number of popped elements up to size
    uint16_t batchPop(Value_t *vs, uint16_t size)
    {
      Synchro_Primitive_t a;
      uint16_t ret = _buffer.batchPop(vs, size);
      return ret;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool push(const Value_t &v, bool overwrite = true)
    {
      Synchro_Primitive_t a;
      bool ret = _buffer.push(v, overwrite);
      return ret;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool batchPush(const Value_t *vs, uint16_t size, bool overwrite = true)
    {
      Synchro_Primitive_t a;
      bool ret = _buffer.batchPush(vs, size, overwrite);
      return ret;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool push_isr(const Value_t &v, bool overwrite = true)
    {
      return _buffer.push(v, overwrite);
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool batchPush_isr(const Value_t *vs, uint16_t size, bool overwrite = true)
    {
      return _buffer.batchPush(vs, size, overwrite);
    }

  private:
    mtl::Ringbuffer<Value_t, Size> _buffer;
  };

  template <typename Value_t, typename Synchro_Primitive_t>
  class DynRingbuffer
  {
  public:
    DynRingbuffer(uint16_t size) : _buffer(size)
    {
    }

    uint16_t in_avail() const
    {
      return _buffer.in_avail();
    }

    bool full() const
    {
      return _buffer.full();
    }

    // returns false if the buffer was empty when attempting to read
    bool pop(Value_t &v)
    {
      Synchro_Primitive_t a;
      bool ret = _buffer.pop(v);
      return ret;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool push(Value_t &v, bool overwrite = true)
    {
      Synchro_Primitive_t a;
      bool ret = _buffer.push(v, overwrite);
      return ret;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool push_isr(Value_t &v, bool overwrite = true)
    {
      return _buffer.push(v, overwrite);
    }

  private:
    mtl::DynRingbuffer<Value_t> _buffer;
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
    static int32_t milliseconds()
    {
      return OS_TIME_GetTicks();
    }

    // calculate current microseconds, overflow after 71 minutes
    static int32_t microseconds()
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
    Gpio(int8_t port, uint8_t pin)
    {
      OS_ASSERT(pin < 16, ERR_BAD_INDEX);
      _port = port;
      _pin = pin;
      _bitmask = 1 << pin;
      switch (port)
      {
      case 'A':
        _pPort = GPIOA;
        break;
      case 'B':
        _pPort = GPIOB;
        break;
      case 'C':
        _pPort = GPIOC;
        break;
      default:
        OS_ASSERT(true == false, ERR_BAD_PORT_NAME);
      }
    }

    enum Mode
    {
      out_push_pull = 0x0,
      out_open_drain = 0x1,
      out_alt_push_pull = 0x2,
      out_alt_open_drain = 0x3,
      in_analog = 0x0,
      in_floating = 0x1,
      in_pullup_pulldown = 0x2
    };

    enum Speed
    {
      Input = 0x0,
      MHz_10 = 0x1,
      MHz_2 = 0x2,
      MHz_50 = 0x3
    };

    void mode(Mode m, Speed s)
    {
      uint16_t cfg = (uint16_t)m << 2 | (uint16_t)s;
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
    }

    bool read()
    {
      return _pPort->IDR & _bitmask;
    }

    void set()
    {
      _pPort->BSRR = _bitmask;
    }

    void clear()
    {
      _pPort->BRR = _bitmask;
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
      tim2_remap_part_1 = 0x0100,
      tim2_remap_part_2 = 0x0200,
      tim2_remap_full = 0x0300,
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
      AFIO->MAPR |= (uint32_t)map;
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
    static void enable(uint16_t line, char port, bool rising, bool falling, uint32_t priority = 200)
    {
      OS_ASSERT('A' <= port && port >= 'C', ERR_BAD_PORT_NAME);
      uint8_t extiselector = port - 'A';
      uint16_t extiafionum = (line % 4) * 4;
      uint16_t extiafioreg = line / 4;
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
    }

    static void disable(uint16_t line)
    {
      Atomic a;
      EXTI->IMR &= ~(1 << line);
      EXTI->RTSR &= ~(1 << line);
      EXTI->FTSR &= ~(1 << line);
    }

    // generate a software interrupt on the line
    static void sw_trigger(uint16_t line)
    {
      EXTI->SWIER = (1 << line);
    }

    // check wether this exti got triggered
    static bool is_set(uint16_t line)
    {
      return EXTI->PR & (1 << line);
    }

    // clear the pending bit for the irq line
    static void clear(uint16_t line)
    {
      EXTI->PR = (1 << line);
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

  class Timer
  {
  public:
    Timer(uint16_t number)
    {
      switch (number)
      {
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
      default:
        OS_ASSERT(true == false, ERR_BAD_PORT_NAME);
      }
    }

    uint16_t value() const
    {
      return _pTimer->CNT;
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

    // configure the timer for pwm out by setting the prescaler, reload and pwm compare startvalue
    // Timers are running with 72MHz
    // the prescalers minimum value is 1, which is added internally (e.g. setting a value of 1 will result in 36MHz timer ticks)
    void configPwm(uint16_t prescaler, uint16_t reload, uint16_t startvalue);

    // configures the timer to enbale 50Hz rc pwm output on the channels
    void configRcPwm()
    {
      // PWM should be 50 Hz. Resolution for RC PWM will be 600 to 2400.
      // 0 = 0 us; 1000 = 1000 usec; -> max pulselength 20ms = 20000
      // the prescaler needs to bring the 72MHz down to 50Hz together with the reload register
      // we divide the counter by 72. So it runs at 1MHz
      // set the startvalue to 155 for center pwm
      configPwm(71, 20000, 1500);
    }

    // enables the corresponding pin to perfom pwm output
    // lower speed at the gpio is desierable for some reason (less jittery / stronger signal)
    void enablePwm(uint16_t channel, Gpio::Speed speed = Gpio::MHz_2);

    // set the corresponding output compare register
    void setPwm(uint16_t channel, uint16_t value)
    {
      --channel;
      OS_ASSERT(channel < 3, ERR_BAD_INDEX);
      uint16_t *reg = (uint16_t *)((&(_pTimer->CCR1)) + channel);
      *reg = value;
    }

    // change the reload value on the fly
    void setReload(uint16_t value)
    {
      _pTimer->ARR = value;
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

    // setup corresponding pin as input for the timer
    void enableInput(uint16_t channel, bool rising_edge = true, bool pulldow = true);

    // enable a callback for the timer
    // triggers at each update event
    // the callback shall accept an uint32 as parameter, its the timer status register
    // and return void upon completition
    // set priority to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1 for higher than average, but dont call FreeRtos Stuff
    void enableCallback(uint32_t priority = 200);

  private:
    uint16_t getTimernum();

    TIM_TypeDef *_pTimer;
  };

  class I2cMaster
  {
  public:
    I2cMaster(uint16_t port, uint8_t address);

    void init();
    bool write(uint8_t *data, uint16_t size);
    bool read(uint8_t *data, uint16_t size);

  private:
    void *_pdev;
    uint8_t _address;
  };

  class UsbCdc
  {
  public:
    static void init();
    // returns the number of bytes in the inbuffer
    static uint16_t in_avail();
    // look at some fancy values in the future
    static bool peek(uint8_t &val, uint32_t offset = 0);
    // remove up to count values from the buffer
    static void flush(uint32_t count = SYN_USBCDC_BUFFSIZE);
    // wait for a change in received data, returns false if it timed out
    static bool waitData(OS_TIME timeout = INT32_MAX);
    // reads as much as possible
    static uint16_t read(uint8_t *data, uint16_t size);
    // read until "\n" is detected, including the "\n" blocks up to timeout.
    // if timed out, nothing will be written to buffer and returns 0
    // returns -1 if the buffer was to small to read the line
    static int32_t readline(uint8_t *data, uint16_t size, OS_TIME timeout = INT32_MAX);
    // blocks until the buffer was written
    static void write(const uint8_t *data, uint16_t size);
    template <typename T>
    static void write(T *data, uint16_t size)
    {
      write((const uint8_t *)data, size);
    }

    template <typename T>
    static void write(T &obj)
    {
      write((const uint8_t *)&obj, sizeof(T));
    }
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
} // namespace syn

extern "C"
{
  void EXTI0_IRQHandler();
  void EXTI1_IRQHandler();
  void EXTI2_IRQHandler();
  void EXTI3_IRQHandler();
  void EXTI4_IRQHandler();
  void DMA1_Channel1_IRQHandler();
  void DMA1_Channel2_IRQHandler();
  void DMA1_Channel3_IRQHandler();
  void DMA1_Channel4_IRQHandler();
  void DMA1_Channel5_IRQHandler();
  void DMA1_Channel6_IRQHandler();
  void DMA1_Channel7_IRQHandler();
  void ADC1_2_IRQHandler();
  void USB_HP_CAN1_TX_IRQHandler();
  void USB_LP_CAN1_RX0_IRQHandler();
  void CAN1_RX1_IRQHandler();
  void CAN1_SCE_IRQHandler();
  void EXTI9_5_IRQHandler();
  void TIM1_BRK_IRQHandler();
  void TIM1_UP_IRQHandler();
  void TIM1_TRG_COM_IRQHandler();
  void TIM1_CC_IRQHandler();
  void TIM2_IRQHandler();
  void TIM3_IRQHandler();
  void TIM4_IRQHandler();
  void I2C1_EV_IRQHandler();
  void I2C1_ER_IRQHandler();
  void I2C2_EV_IRQHandler();
  void I2C2_ER_IRQHandler();
  void SPI1_IRQHandler();
  void SPI2_IRQHandler();
  void USART1_IRQHandler();
  void USART2_IRQHandler();
  void USART3_IRQHandler();
  void EXTI15_10_IRQHandler();
  void RTCAlarm_IRQHandler();
  void USBWakeUp_IRQHandler();
}
