/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "config.h"
#include "f38x_flash.h"
#include <string.h>

#if (CONFIG_ADDRESS % FLASH_PAGESIZE) != 0
#error "Variable(config) must be aligned at flash page boundary."
#endif

volatile __code __at(CONFIG_ADDRESS) config_t config = {
  { // baudrate
    115200, // gps
    9600,}, // telemeter
  { // gps
    { // rate
      200,  // measurement_ms : 200ms = 5Hz
      1,},  // navigation_cycles
    { // message
      {0x01, 0x02, 1},  // NAV-POSLLH   // 28 + 8 = 36 bytes
      {0x01, 0x03, 5},  // NAV-STATUS   // 16 + 8 = 24 bytes
      {0x01, 0x04, 5},  // NAV-DOP      // 18 + 8 = 26 bytes
      {0x01, 0x06, 1},  // NAV-SOL      // 52 + 8 = 60 bytes
      {0x01, 0x12, 1},  // NAV-VELNED   // 36 + 8 = 44 bytes
      {0x01, 0x20, 20}, // NAV-TIMEGPS  // 16 + 8 = 24 bytes
      {0x01, 0x21, 20}, // NAV-TIMEUTC  // 20 + 8 = 28 bytes
      {0x01, 0x30, 10}, // NAV-SVINFO   // (8 + 12 * x) + 8 = 112 bytes (@8)
      {0x02, 0x10, 1},  // RXM-RAW      // (8 + 24 * x) + 8 = 208 bytes (@8)
      {0x02, 0x11, 1},  // RXM-SFRB     // 42 + 8 = 50 bytes
    },},
  { // inertial
    (3 << 3), // gyro_config(MPU-6000/9250) : FS_SEL = 3 (2000dps)
    (2 << 3), // accel_config(MPU-6000/9250) : FS_SEL = 2 (8G)
  },
  { // telemetry_truncate
    20,     // a_page : approximately 5 Hz
    2,      // p_page : approximately 1 Hz
    2,      // m_page : approximately 1 Hz
    {{ // g_page.items
      {0x01, 0x06, 5}, // NAV-SOL: approximately 1 Hz
    },},
  },
  {{ // position upper
    1800000000, 900000000, 100000000, // E180, N90, 100km
  }},
  {{ // position lower
    -1800000000, -900000000, -100000000, // W180, S90, -100km
  }},
};

// temporary buffer for renewal at the same address of USB FIFO
#define CONFIG_RENEWAL_BUFFER_XDATA_ADDRESS 0x0400

__xdata config_t *config_clone(){
  __xdata u8 *buf = (__xdata u8 *)CONFIG_RENEWAL_BUFFER_XDATA_ADDRESS;
  memcpy(buf, &config, FLASH_PAGESIZE);
  return buf;
}

void config_renew(config_t *new_one){
  __xdata u8 *buf = (__xdata u8 *)CONFIG_RENEWAL_BUFFER_XDATA_ADDRESS;
  if((u8 *)new_one == buf){
    // perform renewal without update check
  }else if(memcmp(&config, new_one, sizeof(config_t)) != 0){
    // perform automatic clone of area located behind config
    memcpy(buf, new_one, sizeof(config_t));
    memcpy(buf + sizeof(config_t), (u8 *)&config + sizeof(config_t),
        FLASH_PAGESIZE - sizeof(config_t));

  }else{
    return;
  }
  flash_renew_page((flash_address_t)(&config), buf, FLASH_PAGESIZE);
}
