#pragma once

#include "../synhal32/synhal.h"

#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_W_REGISTER_MEAS 0x74
#define BMP280_W_REGISTER_CONFIG 0x75
#define BMP280_REGISTER_PRESS 0xF7
#define BMP280_REGISTER_TEMP 0xFA
#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E

class BMP280
{
public:
  void init(uint16_t port)
  {
    _spi.init(port, 4 * 1000 * 1000, true, true, false, true);
    uint8_t inicommand[4] = {
        BMP280_W_REGISTER_CONFIG,
        // IIR filter off, stanby off (0.5ms)
        0b00000000,
        BMP280_W_REGISTER_MEAS,
        // Force Mode, Sampling x1
        0b00100101};
    _spi.busy_tx(inicommand, 4);
    _spi.busy_read_regs(BMP280_REGISTER_DIG_T1, (uint8_t*)&_calib, sizeof(_calib));
  }

  uint8_t read_status()
  {
    uint8_t stat;
    _spi.busy_read_regs(BMP280_REGISTER_STATUS, &stat, 1);
    return stat;
  }

  void start_measure()
  {
    // Force Mode, Sampling x1
    uint8_t inicommand[2] = { BMP280_W_REGISTER_MEAS, 0b00100101 };
    _spi.busy_tx(inicommand, 2);
  }

  int32_t read_temp()
  {
    int32_t var1, var2, T;
    T = BMP280_REGISTER_TEMP;
    _spi.busy_bidi((uint8_t*)&T, 4);
    T >>= 4;
    var1 = ((((T>>3) - ((int32_t)_calib.dig_T1<<1))) * ((int32_t)_calib.dig_T2)) >> 11;
    var2 = (((((T>>4) - ((int32_t)_calib.dig_T1)) * ((T>>4) - ((int32_t)_calib.dig_T1))) >> 12) * ((int32_t)_calib.dig_T3)) >> 14;
    _t_fine = var1 + var2;
    T = (_t_fine * 5 + 128) >> 8;
    return T;
  }

private:
  typedef struct
  {
    uint16_t dig_T1; /**< dig_T1 cal register. */
    int16_t dig_T2;  /**<  dig_T2 cal register. */
    int16_t dig_T3;  /**< dig_T3 cal register. */

    uint16_t dig_P1; /**< dig_P1 cal register. */
    int16_t dig_P2;  /**< dig_P2 cal register. */
    int16_t dig_P3;  /**< dig_P3 cal register. */
    int16_t dig_P4;  /**< dig_P4 cal register. */
    int16_t dig_P5;  /**< dig_P5 cal register. */
    int16_t dig_P6;  /**< dig_P6 cal register. */
    int16_t dig_P7;  /**< dig_P7 cal register. */
    int16_t dig_P8;  /**< dig_P8 cal register. */
    int16_t dig_P9;  /**< dig_P9 cal register. */
  } bmp280_calib_data;

  syn::SpiMaster _spi;
  bmp280_calib_data _calib;
  int32_t _t_fine;
};