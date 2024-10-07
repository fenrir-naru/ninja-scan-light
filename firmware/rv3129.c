/*
 * Copyright (c) 2024, M.Naruoka (fenrir)
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

#include "c8051f380.h"

#include "rv3129.h"
#include "main.h"
#include "gps.h"
#include "util.h"

#include "f38x_i2c.h"

#include "data_hub.h"
#include "ff.h"

#include <stdio.h>

#define I2C_ADDRESS_RW (0x56 << 1)

typedef enum {
  // Control page
  ADDR_Control_1 = 0,
  ADDR_Control_INT,
  ADDR_Control_INT_Flag,
  ADDR_Control_Status,
  ADDR_Control_Reset,

  // Clock page
  ADDR_Seconds = 0x08,
  ADDR_Minutes,
  ADDR_Hours,
  ADDR_Days,
  ADDR_Weekdays,
  ADDR_Months,
  ADDR_Years,

  // Alarm page
  ADDR_Second_Alarm = 0x10,
  ADDR_Minute_Alarm,
  ADDR_Hour_Alarm,
  ADDR_Days_Alarm,
  ADDR_Weekday_Alarm,
  ADDR_Months_Alarm,
  ADDR_Year_Alarm,

  // Timer page
  ADDR_Timer_Low = 0x18,
  ADDR_Timer_High,

  // Temperature page
  ADDR_Temperature = 0x20,

  // EEPROM User
  ADDR_EEPROM_User0 = 0x28,
  ADDR_EEPROM_User1,

  // EEPROM Control page
  ADDR_EEPROM_Contr = 0x30,
  ADDR_Xtal_Offset,
  ADDR_Xtal_Coef,
  ADDR_Xtal_T0,

  // RAM page
  //User RAM = 0x38,
} address_t;

static void read_data(u8 addr, u8 *buf, u8 buf_size){
  i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), &addr, 1);
  i2c0_read_write(I2C_READ(I2C_ADDRESS_RW), buf, buf_size);
}

static void update_register(u8 addr, u8 v, u8 v_mask){
  u8 buf[2] = {addr};
  read_data(addr, &buf[1], 1);
  buf[1] &= ~v_mask;
  buf[1] |= (v & v_mask);
  i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), buf, 2);
}

static void start_watch(){
  update_register(ADDR_Control_1, 0x01, 0x01); // WE = 1
}
static void stop_watch(){
  update_register(ADDR_Control_1, 0x00, 0x01); // WE = 0
}
static void start_timer(){
  update_register(ADDR_Control_1, 0x02, 0x02); // TE = 1
}
static void stop_timer(){
  update_register(ADDR_Control_1, 0x00, 0x02); // TE = 0
}

#define BCD2NUM(bcd) ((((bcd & 0xF0) >> 4) * 10) + (bcd & 0x0F))
#define NUM2BCD(num) ((((num) / 10) << 4) + (num % 10))

static void get_watch(struct tm *t){
  u8 buf[7];
  read_data(ADDR_Seconds, buf, sizeof(buf));
  t->tm_sec = BCD2NUM(buf[0]);        // Seconds [0-60]
  t->tm_min = BCD2NUM(buf[1]);        // Minutes [0-59]
  t->tm_hour = BCD2NUM(buf[2]);       // Hours   [0-23]
  t->tm_mday = BCD2NUM(buf[3]);       // Day     [1-31]
  t->tm_wday = BCD2NUM(buf[4]);       // Weekday [0:Sunday-6]
  t->tm_mon = BCD2NUM(buf[5]) - 1;    // Month [1-12] -> [0-11]
  t->tm_year = BCD2NUM(buf[6]) + 100; // Year since 2000 -> 1900
}

static void set_watch(struct tm *t){
  u8 mon = t->tm_mon + 1; // Month [0-11] -> [1-12]
  u8 year = t->tm_year - 100; // Year since 1900 -> 2000
  update_wday(t); // Recalculate weekday
  {
    u8 buf[] = {
      ADDR_Seconds,
      NUM2BCD(t->tm_sec),     // Seconds [0-60]
      NUM2BCD(t->tm_min),     // Minutes [0-59]
      NUM2BCD(t->tm_hour),    // Hours   [0-23]
      NUM2BCD(t->tm_mday),    // Day     [1-31]
      NUM2BCD(t->tm_wday),    // weekday [0:Sunday-6]
      NUM2BCD(mon),
      NUM2BCD(year),
    };
    i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), buf, sizeof(buf));
  }
}

static void get_watch2(u32 *tow_ms, struct tm *t_buf){
  struct tm t;
  if(!t_buf){t_buf = &t;}
  get_watch(t_buf);
  *tow_ms = t_buf->tm_wday;    // weekday [0:Sunday-6]
  (*tow_ms) *= 24;
  (*tow_ms) += t_buf->tm_hour; // Hours   [0-23]
  (*tow_ms) *= 60;
  (*tow_ms) += t_buf->tm_min;  // Minutes [0-59]
  (*tow_ms) *= 60;
  (*tow_ms) += t_buf->tm_sec;  // Seconds [0-60]
  (*tow_ms) *= 1000;
}

static void set_watch2(u32 tow_ms, struct tm *t_ref){
  struct tm t = {0};
  tow_ms = ((tow_ms + 999) / 1000) + 1; // round up
  t.tm_sec = tow_ms % 60;         // Seconds [0-60]
  t.tm_min = (tow_ms /= 60) % 60; // Minutes [0-59]
  t.tm_hour = (tow_ms /= 60) % 24;// Hours   [0-23]
  t.tm_wday = tow_ms / 24;        // weekday [0:Sunday-6]
  if(t_ref && (t.tm_wday == t_ref->tm_wday) && (t_ref->tm_year >= 100)){
    // Note: (RV3129 year) in [2000, 2079]
    t.tm_mday = t_ref->tm_mday;
    t.tm_mon = t_ref->tm_mon;
    t.tm_year = t_ref->tm_year;
  }else{
    t.tm_mday = 9 + t.tm_wday;    // Day     [1-31]
    t.tm_mon = 0; // 2000/1/9 = Sunday
    t.tm_year = 100;
  }
  set_watch(&t);
}

static u16 get_timer(){
  WORD_t v;
  read_data(ADDR_Timer_Low, v.c, sizeof(v));
  return v.i;
}

static void set_timer(u16 v){
  u8 buf[] = {
    ADDR_Timer_Low, (u8)(v & 0xFF), (u8)(v >> 8)
  };
  i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), buf, sizeof(buf));
}

// Load milliseconds of week described in RTC.CFG.
// After loaded, file name is changed to RTC.OLD to preventfrom multiple load.
// Appropriate time can be obtained online such as https://gnsscalc.com/
static const char config_fname[] = "RTC.CFG";
static void set_rtc_config(FIL *f){
  long tow_ms_new = data_hub_read_long(f);
  if(tow_ms_new < 0){return;}
  stop_watch();
  set_watch2(tow_ms_new, NULL);
  start_watch();
#if _FS_MINIMIZE < 1
  f_close(f);
  f_rename(config_fname, "RTC.OLD");
#else // set illegal size for overwrite prevention
#define push_str(f, str) { \
  UINT buf_filled; \
  f_write(f, str, (sizeof(str) - 1), &buf_filled); \
}
  push_str(f, "-1");
#undef push_str
#endif
}

void rv3129_init(){
  u8 buf;
  data_hub_load_config(config_fname, set_rtc_config);
  read_data(ADDR_Control_1, &buf, 1);
  if(buf & 0x01){ // watch started
    get_watch2(&(gps_time.itow_ms), &gps_utc);
    gps_time_modified = TRUE;
    IE0 = 1; // invoke -INT0
  }
}

static __xdata u8 watch_updated = 0;

void rv3129_polling(){
  if(!gps_utc_valid){return;}
  if((watch_updated++ & 0x0F) != 0){return;}
  stop_watch();
  set_watch2(gps_time.itow_ms, &gps_utc);
  start_watch();
}
