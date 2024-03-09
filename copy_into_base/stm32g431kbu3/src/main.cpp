#include <synhal.h>

// uncomment when using usb-synrpc
//#include "synrpc_usbcon.h"
//syn::UsbRpc::Handler packethandler;

#include <stdio.h>


syn::Mutex mut;


static void _Write(char const *s, int i)
{
  mut.lock();
  printf(s, i);
  mut.unlock();
}


class HPTask : public syn::Thread
{
public:
  HPTask() : syn::Thread("HP_Task", 100, 256, stack)
  {
  }

  void run()
  {
    int i = 0;
    while (1)
    {
      _Write("HPTask %d\n", i++);
      sleep(50);
    }
  }

  uint32_t stack[256];
};


class LPTask : public syn::Thread
{
public:
  LPTask() : syn::Thread("LP_Task", 50, 256, 0)
  {
  }

  void run()
  {
    int i = 0;
    while (1)
    {
      _Write("LPTask %d\n", i++);
      sleep(100);
    }
  }

  //uint32_t stack[256];
};


HPTask hptask;
LPTask lptask;

int main(void)
{
  syn::System::init();
  // also start the usb-synrpc handler
  //packethandler.start();

  hptask.start();
  lptask.start();

  mut.init();
  /* Start scheduler */
  syn::System::spin();
  /* We should never get here as control is now taken by the scheduler */
  for (;;)
    ;
}
