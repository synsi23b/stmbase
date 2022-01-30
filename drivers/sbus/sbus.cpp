#include "sbus.h"

using namespace syn;

// setup uart and start receiving frames
// the uart buffer gets checked in calls to "available"
void SbusReader::start()
{
    Uart::init(Uart::bd100000, Uart::sb2, Uart::even);
    Uart::rx_start();
}

// return true if a new frame is ready
bool SbusReader::available()
{
    uint8_t avail = Uart::rx_avail();
    if (avail >= 25)
    {
        uint8_t frame_start = 0;
        while(frame_start < (avail - 24))
        {
            if(Uart::peek(frame_start) == 0x0F)
            {
                // sbus frame possible start found
                uint8_t footer = frame_start + 24;
                if(footer < avail)
                {
                    // footer still in bounds
                    footer = Uart::peek(footer);
                    if(footer == 0x00 || (footer & 0x0F == 0x04))
                    {
                        // footer is correct, parse the frame
                        while(frame_start != 0)
                        {
                            uint8_t to_clear = frame_start < 25 ? frame_start : 25;
                            // clear garbage that might be in front
                            frame_start -= Uart::read(_last_frame, frame_start);
                        }
                        Uart::read(_last_frame, 25);
                        return true;
                    }
                }
            }
            ++frame_start;
        }
    }
    return false;
}

// return true if in failsafe mode
bool SbusReader::failsafe()
{
    return _last_frame[23] & 0x08;
}

// return the data for the given channel. available channel numbers are 0 .. 15
// data can range from 0 to 2047 (11 bit)
uint16_t SbusReader::channel(uint8_t number)
{
    uint16_t ret = 1024;
    switch (number)
    {
    case 0:
        ret = static_cast<uint16_t>(_last_frame[1] | _last_frame[2] << 8);
        break;
    case 1:
        ret = static_cast<uint16_t>(_last_frame[2] >> 3 | _last_frame[3] << 5);
        break;
    case 2:
        ret = static_cast<uint16_t>(_last_frame[3] >> 6 | _last_frame[4] << 2 | _last_frame[5] << 10);
        break;
    case 3:
        ret = static_cast<uint16_t>(_last_frame[5] >> 1 | _last_frame[6] << 7);
        break;
    case 4:
        ret = static_cast<uint16_t>(_last_frame[6] >> 4 | _last_frame[7] << 4);
        break;
    case 5:
        ret = static_cast<uint16_t>(_last_frame[7] >> 7 | _last_frame[8] << 1 | _last_frame[9] << 9);
        break;
    case 6:
        ret = static_cast<uint16_t>(_last_frame[9] >> 2 | _last_frame[10] << 6);
        break;
    case 7:
        ret = static_cast<uint16_t>(_last_frame[10] >> 5 | _last_frame[11] << 3);
        break;
    case 8:
        ret = static_cast<uint16_t>(_last_frame[12] | _last_frame[13] << 8);
        break;
    case 9:
        ret = static_cast<uint16_t>(_last_frame[13] >> 3 | _last_frame[14] << 5);
        break;
    case 10:
        ret = static_cast<uint16_t>(_last_frame[14] >> 6 | _last_frame[15] << 2 | _last_frame[16] << 10);
        break;
    case 11:
        ret = static_cast<uint16_t>(_last_frame[16] >> 1 | _last_frame[17] << 7);
        break;
    case 12:
        ret = static_cast<uint16_t>(_last_frame[17] >> 4 | _last_frame[18] << 4);
        break;
    case 13:
        ret = static_cast<uint16_t>(_last_frame[18] >> 7 | _last_frame[19] << 1 | _last_frame[20] << 9);
        break;
    case 14:
        ret = static_cast<uint16_t>(_last_frame[20] >> 2 | _last_frame[21] << 6);
        break;
    case 15:
        ret = static_cast<uint16_t>(_last_frame[21] >> 5 | _last_frame[22] << 3);
        break;
    }
    return ret & 0x3FF;
}
