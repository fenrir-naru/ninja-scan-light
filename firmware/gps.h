/*
 * Copyright (c) 2013, M.Naruoka (fenrir)
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

#ifndef __GPS_H__
#define __GPS_H__

/*
 * u-block GPS => Little Endian!!
 * sdcc => Little Endian
 * 
 */

#include <time.h>
#include "type.h"

#define MAX_SAT 16
#define USE_GPS_STD_TIME 0
#define GPS_TIME_FROM_RAW_DATA 0

void gps_sleep();
void gps_wakeup();
void gps_init();
void gps_polling();
#if USE_GPS_STD_TIME
time_t gps_std_time(time_t *timeptr);
#endif

extern volatile __bit gps_time_modified;
typedef struct {
  s16 wn;
  u32 itow_ms;
} gps_time_t;
extern __xdata gps_time_t gps_time;

typedef enum {
  GPS_FIX_NO = 0,
  GPS_FIX_DEAD_RECKONING_ONLY = 1,
  GPS_FIX_2D = 2,
  GPS_FIX_3D = 3,
  GPS_FIX_DEAD_RECKONING_COMBINED = 4,
  GPS_FIX_TIME_ONLY = 5,
} gps_fix_type_t;
typedef enum {
  GPS_POS_ACC_BAD = 0,
  GPS_POS_ACC_1KM = (1 << 3),
  GPS_POS_ACC_500M = (2 << 3),
  GPS_POS_ACC_100M = (3 << 3),
} gps_pos_accuracy_t; // larger value has better accuracy, opposite to error!
extern __xdata u8 gps_fix_info; // bit[0..2] = fix_type, bit[3..4] = pos_accuracy
#define gps_fix_type ((gps_fix_type_t)(gps_fix_info & 0x07))
#define gps_pos_accuracy ((gps_pos_accuracy_t)(gps_fix_info & 0x18))

extern __xdata u8 gps_num_of_sat;

extern __bit gps_utc_valid;
extern __xdata struct tm gps_utc;

typedef union {
  struct {
    s32 lng; // 1E-7 [deg]; (-180, 180]
    s32 lat; // 1E-7 [deg]; [-90, 90]
    s32 alt; // 1E-3 [m];
  } s;
  s32 v[3];
} gps_pos_t;
extern __xdata void (*gps_position_monitor)(__xdata gps_pos_t *);

typedef struct {
  u8 msg_class;
  u8 msg_id;
  u8 rate;
} ubx_cfg_t;

#endif /* __GPS_H__ */
