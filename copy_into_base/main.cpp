#include <synhal.h>

// uncomment when using usb-synrpc
//#include <synrpc_usbcon.h>
//syn::PacketHandler packethandler;

int main(void)
{
  syn::System::init();
  // also start the usb-synrpc handler
  //packethandler.start();


  /* Start scheduler */
  syn::System::spin();
  /* We should never get here as control is now taken by the scheduler */
  for (;;)
    ;
}
