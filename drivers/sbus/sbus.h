#pragma once

#ifdef STM8S103
#include "../../synos8/synos.h"
#endif

class SbusReader {
public:
  SbusReader()
  {}

  // setup uart and start receiving frames
  // the uart buffer gets checked in calls to "available"
  void start();

  // return true if a new frame is ready
  bool available();

  // return true if in failsafe mode
  bool failsafe() const;

  // return the data for the given channel.
  // data can range from 0 to 2047 (11 bit)
  uint16_t channel_1() const;
  uint16_t channel_2() const;
  uint16_t channel_3() const;
  //uint16_t channel_4() const;
private:
  uint8_t _last_frame[25];
};