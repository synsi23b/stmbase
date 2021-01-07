#include <synos.h>

//#include "../stmbase/drivers/tm1638.h"
//#include "../stmbase/drivers/nrf24/rf24node.h"
//#include "../stmbase/drivers/ds18b20/ds18b20.h"

//#include <stdio.h>
//#define printf(x, y) (x, y)
//#define printf(x) (x)

syn::Led alive_led;

//DS18B20Manager temp_manager;

const char *dest = "LOGR";

void check_input_routine(uint16_t arg)
{
  // uses port b5 (I2C)
  alive_led.init();

  while(1)
  {
    alive_led.toggle();

    // RF24Node::Message *pmsg = RF24Node::try_reserve(RF24_NODE_LOGGER, RF24Node::log_key_value);
    // if (pmsg != 0)
    // {
    //   //pmsg->init(dest, RF24Node::log_key_value);
    //   pmsg->log_u8("Foo", 23);
    //   pmsg->log_i16("Bar", -1024);
    //   pmsg->log_str("YOL", "Whatud!");
    //   pmsg->send(RF24Node::unreliable);
    //   pmsg = 0;
    // }
    syn::Routine::sleep(1000);
  }
}

void thermometer_routine(uint16_t arg)
{
  //uint8_t count = temp_manager.init('D', 4);

  while (1)
  {
    // temp_manager.start_convert_all();
    // syn::Routine::sleep(1000);
    
    // int16_t temp = temp_manager.get_selected_device()->read_raw();

    // RF24Node::Message *pmsg = RF24Node::try_reserve();
    // if(pmsg)
    // {
    //   pmsg->init(dest, RF24Node::temperature_value);
    //   pmsg->temperature(temp_manager.get_selected_device()->get_address(), temp);
    //   pmsg->send();
    // }

    syn::Routine::sleep(1000);
  }
}

void tim_x()
{
  // run every 10 milliseconds on mainstack during systick interrupt
}

int main()
{
  syn::Kernel::init();

  // IMPORTANT !!!
  // the count of routines has to exactly match the amount of routines used
  // the routine index given during initialization has to match

  // routine index 0, round_robin_time 10 millis
  // stack allocated automatically
  syn::Routine::init((void *)&check_input_routine, 0, 128);

  // routine index 1, round_robin_time 10 millis, managing the nrf24 module
  //syn::Routine::init<1, 10>(&RF24Node::message_handler_routine, RF24_NODE_FOLD_ADDR('S', 'M', 0), 128);

  syn::Routine::init((void *)&thermometer_routine, 0, 128);

  syn::SysTickHook::init<0, 10>(tim_x);
  //syn::SysTickHook::init<1, 10>(tim_y);

  // never returns
  syn::Kernel::spin();
  return 0;
}
