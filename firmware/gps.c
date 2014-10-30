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

/**
 * u-blox GPS support routines.
 *  by fenrir.
 */

#include "c8051f380.h"
#include <string.h>

#include "main.h"
#include "f38x_uart0.h"
#include "data_hub.h"
#include "gps.h"
#include "util.h"

void gps_write(char *buf, int size){
  int written = 0;
  while(TRUE){
    written = uart0_write(buf, size);
    size -= written;
    if(size == 0){break;}
    buf += written;
  }
}

static void checksum(unsigned char ck[2], 
                        unsigned char *packet, 
                        int size){
  u8 a, b;
  a = b = 0x00;
  while(size-- > 0){
    a += *(packet++);
    b += a;
  }
  ck[0] = a;
  ck[1] = b;
}

static void gps_packet_write(char *packet, int packet_size){
  unsigned char ck[2];
  checksum(ck, &packet[2], packet_size - 2);
  gps_write(packet, packet_size);
  gps_write(ck, sizeof(ck));
}

#define swap(x) (\
(((x) & 0x80) ? 0x01 : 0x00) | \
(((x) & 0x40) ? 0x02 : 0x00) | \
(((x) & 0x20) ? 0x04 : 0x00) | \
(((x) & 0x10) ? 0x08 : 0x00) | \
(((x) & 0x08) ? 0x10 : 0x00) | \
(((x) & 0x04) ? 0x20 : 0x00) | \
(((x) & 0x02) ? 0x40 : 0x00) | \
(((x) & 0x01) ? 0x80 : 0x00) \
)

#define expand_16(x) \
((u16)(x) & 0xFF), \
((u16)(x) >> 8) & 0xFF

#define expand_32(x) \
((u32)(x) & 0xFF), \
((u32)(x) >> 8) & 0xFF, \
((u32)(x) >> 16) & 0xFF, \
((u32)(x) >> 24) & 0xFF

#define UBX_CFG_RATE_MEAS 200 // 5Hz
#define UBX_CFG_RATE_NAV  1
#define UBX_CFG_RAET_TIME 0

static void set_ubx_cfg_rate(){
  static const __code unsigned char packet[6 + 6] = {
    0xB5, // packet[0]
    0x62, // packet[1]
    0x06, // packet[2]
    0x08, // packet[3]
    expand_16(sizeof(packet) - 6),   // packet[4 + 0]
    expand_16(UBX_CFG_RATE_MEAS),     // packet[6 + 0]
    expand_16(UBX_CFG_RATE_NAV),      // packet[6 + 2]
    expand_16(UBX_CFG_RAET_TIME),     // packet[6 + 4]
  };
  gps_packet_write(packet, sizeof(packet));
}

#define UBX_CFG_TP_INTERVAL    1000000
#define UBX_CFG_TP_LENGTH      1000
#define UBX_CFG_TP_STATUS      -1
#define UBX_CFG_TP_TIMEREF     0
#define UBX_CFG_TP_CABLE_DELAY 50
#define UBX_CFG_TP_RF_DELAY    820
#define UBX_CFG_TP_USER_DELAY  0

static void set_ubx_cfg_tp(){
  static const __code unsigned char packet[20 + 6] = {
    0xB5, // packet[0] 
    0x62, // packet[1] 
    0x06, // packet[2] 
    0x07, // packet[3] 
    expand_16(sizeof(packet) - 6),       // packet[4 + 0]    
    expand_32(UBX_CFG_TP_INTERVAL),       // packet[6 + 0]  
    expand_32(UBX_CFG_TP_LENGTH),         // packet[6 + 4]  
    (UBX_CFG_TP_STATUS & 0xFF),           // packet[6 + 8]  
    (UBX_CFG_TP_TIMEREF & 0xFF),          // packet[6 + 9]  
    0x00,                                 // packet[6 + 10] 
    0x00,                                 // packet[6 + 11] 
    expand_16(UBX_CFG_TP_CABLE_DELAY),    // packet[6 + 12]  
    expand_16(UBX_CFG_TP_RF_DELAY),       // packet[6 + 14]  
    expand_32(UBX_CFG_TP_USER_DELAY),     // packet[6 + 16] 
  };
  gps_packet_write(packet, sizeof(packet));
}

static void set_ubx_cfg_sbas(){
  static const __code unsigned char packet[8 + 6] = {
    0xB5, // packet[0] 
    0x62, // packet[1] 
    0x06, // packet[2] 
    0x16, // packet[3] 
    expand_16(sizeof(packet) - 6), // packet[4 + 0]
    0, // packet[6 + 0]; mode, SBAS Disabled(bit0 = 0)
    0, // packet[6 + 1]; usage
    0, // packet[6 + 2]; maxsbas
    0, // packet[6 + 3]; reserved
    expand_32(0), // packet[6 + 4], scanmode
  };
  gps_packet_write(packet, sizeof(packet));
}

#define UBX_CFG_PRT_BAUDRATE  115200

static void set_ubx_cfg_prt(){
  // UBX
#if (defined(SDCC) && (SDCC < 270)) // lower than sdcc-2.7.0 ?
  unsigned char packet[20 + 6] = {
#else
  unsigned char packet[20 + 6];
  static const __code unsigned char _packet[sizeof(packet)] = {
#endif
    0xB5, // packet[0] 
    0x62, // packet[1] 
    0x06, // packet[2] 
    0x00, // packet[3] 
    expand_16(sizeof(packet) - 6), // packet[4 + 0]  
    0x00, // packet[6 + 0]   // Port.NO
    0x00, // packet[6 + 1]   // Res0
    0x00, // packet[6 + 2]   // Res1
    0x00, // packet[6 + 3]  
    0xC0, // packet[6 + 4]   // (M)11000000(L)
    0x08, // packet[6 + 5]   // (M)00001000(L)
    0x00, // packet[6 + 6]   // (M)00000000(L)
    0x00, // packet[6 + 7]   //
    expand_32(UBX_CFG_PRT_BAUDRATE), // packet[6 + 8]  
    0x07, // packet[6 + 12]  // in - UBX,NMEA,RAW
    0x00, // packet[6 + 13] 
    0x01, // packet[6 + 14]  // out - UBX
    0x00, // packet[6 + 15] 
    0x00, // packet[6 + 16] 
    0x00, // packet[6 + 17] 
    0x00, // packet[6 + 18] 
    0x00, // packet[6 + 19] 
  };
#if !(defined(SDCC) && (SDCC < 270)) // Not lower than sdcc-2.7.0?
  memcpy(packet, _packet, sizeof(packet));
#endif
  
  {
    u8 i;
    for(i = 0; i < 3; i++){
      packet[6 + 0]  = i;
      gps_packet_write(packet, sizeof(packet));
    }
  }
  
  // NMEA
  /*const char code *str1 = "$PUBX,41,0,0007,0003,115200,1*1A\r\n";
  const char code *str2 = "$PUBX,41,1,0007,0003,115200,1*1B\r\n";
  const char code *str3 = "$PUBX,41,2,0007,0003,115200,1*18\r\n";  
  gps_write(str1, strlen(str1));
  gps_write(str2, strlen(str2));  
  gps_write(str3, strlen(str3));*/
}

static void set_ubx_cfg_msg(u8 _class, u8 id, u8 rate){
  unsigned char packet[6 + 3] = { // 3 byte payload CFG-MSG, which will change message rate for the current port.
    0xB5, // packet[0] 
    0x62, // packet[1] 
    0x06, // packet[2] 
    0x01, // packet[3] 
    expand_16(sizeof(packet) - 6), // packet[4 + 0]  
    _class,   // packet[6 + 0]
    id,       // packet[6 + 1]  
    rate,     // packet[6 + 2]
  };
  gps_packet_write(packet, sizeof(packet));
}

#if GPS_DIRECT
#include "data_hub.h"
#include "usb_cdc.h"

static void gps_direct(){
  char buf[32];
  while(1){
    cdc_tx(buf, (u16)uart0_read(buf, sizeof(buf)));
    uart0_write(buf, (u8)cdc_rx(buf, sizeof(buf)));
  }
}

static void gps_direct_init(FIL *f){
  cdc_force = TRUE;
  main_loop_prologue = gps_direct;
}

#endif

static void additional_config(FIL *f){
  char buf[8];
  u16 buf_filled;
  while(f_read(f, buf, sizeof(buf), &buf_filled) == FR_OK){
    gps_write(buf, buf_filled);
    if(buf_filled < sizeof(buf)){break;}
  }
}

void gps_init(){
  
  // init wait
  wait_ms(100);
  
  // set U-blox configuration
  set_ubx_cfg_prt();              // baudrate change
  while(uart0_tx_active());
  uart0_bauding(UBX_CFG_PRT_BAUDRATE);
  
  // baudrate change wait
  wait_ms(100);
  
  // clear buffer
  {
    char c;
    while(uart0_read(&c, 1) > 0);
  }
  
  set_ubx_cfg_rate();
  set_ubx_cfg_tp();
  set_ubx_cfg_sbas();
  
  set_ubx_cfg_msg(0x01, 0x02, 1);  // NAV-POSLLH  // 28 + 8 = 36 bytes
  set_ubx_cfg_msg(0x01, 0x03, 5);  // NAV-STATUS  // 16 + 8 = 24 bytes
  set_ubx_cfg_msg(0x01, 0x04, 5);  // NAV-DOP     // 18 + 8 = 26 bytes
  set_ubx_cfg_msg(0x01, 0x06, 1);  // NAV-SOL     // 52 + 8 = 60 bytes
  set_ubx_cfg_msg(0x01, 0x12, 1);  // NAV-VELNED  // 36 + 8 = 44 bytes
  set_ubx_cfg_msg(0x01, 0x20, 20);  // NAV-TIMEGPS  // 16 + 8 = 24 bytes
  set_ubx_cfg_msg(0x01, 0x21, 20);  // NAV-TIMEUTC  // 20 + 8 = 28 bytes
  set_ubx_cfg_msg(0x01, 0x30, 10);  // NAV-SVINFO  // (8 + 12 * x) + 8 = 112 bytes (@8)
  set_ubx_cfg_msg(0x02, 0x10, 1);  // RXM-RAW     // (8 + 24 * x) + 8 = 208 bytes (@8)
  set_ubx_cfg_msg(0x02, 0x11, 1);  // RXM-SFRB    // 42 + 8 = 50 bytes
  
  data_hub_load_config("GPS.CFG", additional_config);

#if GPS_DIRECT
  data_hub_load_config("DIRECT.GPS", gps_direct_init);
#endif
}

static void poll_aid_eph(u8 svid){
  unsigned char packet[1 + 6] = {
    0xB5, // packet[0] 
    0x62, // packet[1] 
    0x0B, // packet[2] 
    0x31, // packet[3] 
    expand_16(sizeof(packet) - 6), // packet[4 + 0]  
    svid, // packet[6 + 0]
  };
  gps_packet_write(packet, sizeof(packet));
}

static void poll_aid_hui(){
  static const __code unsigned char packet[0 + 6] = {
    0xB5, // packet[0] 
    0x62, // packet[1] 
    0x0B, // packet[2] 
    0x02, // packet[3] 
    expand_16(sizeof(packet) - 6), // packet[4 + 0]  
  };
  gps_packet_write(packet, sizeof(packet));
}

volatile __bit gps_time_modified = FALSE;
__xdata gps_time_t gps_time = {0, 0};
__xdata u8 gps_num_of_sat = 0;

static __xdata s8 leap_seconds = 0;
#if USE_GPS_STD_TIME
time_t gps_std_time(time_t *timer) {
  time_t res = 0;
  if(gps_time.wn > 0){ // valid
   res = ((time_t)60 * 60 * 24 * 7) * gps_time.wn
       + (global_ms / 1000) 
       - leap_seconds
       + (time_t)315964800; // Jan 01, 1970, 00:00:00 UTC
  }
  if(timer){*timer = res;}
  return res;
}
#endif

__bit gps_utc_valid = FALSE;
__xdata struct tm gps_utc;

typedef enum {
  NAV_SOL, NAV_TIMEGPS, NAV_TIMEUTC, NAV_SVINFO,
  RXM_RAW, RXM_SFRB,
  AID_HUI, AID_EPH,
  UNKNOWN = 0
} packet_type_t;

static void push_telemetry(char c){
  static __xdata char buf[PAGE_SIZE] = {'G'};
  static __xdata unsigned char index = 0;
  buf[++index] = c;
  if(index >= (sizeof(buf) - 1)){
    data_hub_send_telemetry(buf);
    index = 0;
  }
}

#define UBX_GPS_MAX_ID 32
static void make_packet(packet_t *packet){
  payload_t *dst = packet->current;
  u8 size = packet->buf_end - dst;
  
  //if(size == 0){return;} // guranteed that size > 0
  
  *(dst++) = 'G';
    
  // packetへの登録
  size = uart0_read(dst, --size);
    
  // GPS時刻の取得, 可視衛星に対するエフェメリスの要求
  while(size-- > 0){
    
    static __xdata struct {
      u16 index, size;
      u8 ck_a, ck_b; // チェックサム用
      packet_type_t packet_type;
    } ubx_state = {0};
    
    static __xdata u32 ephemeris_received_gps = 0;
    static __xdata u8 svid;
    static __xdata gps_time_t time_buf;
    
    u8 c = *(dst++);
    
    static __bit make_telemetry = FALSE;
    if(make_telemetry){push_telemetry(c);}

    switch(++ubx_state.index){
      case 1:
        if(c == 0xB5){
          ubx_state.size = 8;
        }else{
          ubx_state.index = 0;
        }
        continue; // jump to while loop.
      case 2: 
        if(c != 0x62){ubx_state.index = 0;} 
        continue; // jump to while loop.
      case 3: 
        ubx_state.ck_a = ubx_state.ck_b = c;
        continue; // jump to while loop.
      case 4:
        switch(ubx_state.ck_a){
          case 0x01:
            switch(c){
              case 0x06: {
                static __xdata unsigned char count = 0;
                if(++count >= 5){ // approximately 1Hz
                  make_telemetry = TRUE;
                  count = 0;
                }
                ubx_state.packet_type = NAV_SOL;
                break;
              }
              case 0x20: ubx_state.packet_type = NAV_TIMEGPS; break;
              case 0x21: ubx_state.packet_type = NAV_TIMEUTC; break;
              case 0x30: ubx_state.packet_type = NAV_SVINFO; break;
            }
            break;
          case 0x02:
            switch(c){
              case 0x10: ubx_state.packet_type = RXM_RAW; break;
              case 0x11: ubx_state.packet_type = RXM_SFRB; break;
            }
            break;
          case 0x0B:
            switch(c){
              case 0x02: ubx_state.packet_type = AID_HUI; break;
              case 0x31: ubx_state.packet_type = AID_EPH; break;
            }
            break;
        }
        if(make_telemetry){
          push_telemetry(0xB5);
          push_telemetry(0x62);
          push_telemetry(ubx_state.ck_a);
          push_telemetry(c);
        }
        break;
      case 5:
        ubx_state.size += c;
        break;
      case 6:
        ubx_state.size += ((u16)c << 8);
        break;
      default: {
        if(ubx_state.index >= (ubx_state.size - 1)){
          if(ubx_state.index == (ubx_state.size - 1)){
            if(ubx_state.ck_a == c){ // partially correct checksum
              continue; // jump to while loop.
            }
          }else if(ubx_state.ck_b == c){ // correct checksum
            if(ubx_state.packet_type == (GPS_TIME_FROM_RAW_DATA ? RXM_RAW : NAV_SOL)){
              time_buf.itow_ms = ((time_buf.itow_ms / 1000) + 1) * 1000;
              if(time_buf.itow_ms >= (u32)60 * 60 * 24 * 7 * 1000){
                time_buf.itow_ms = 0;
                time_buf.wn++;
              }
              gps_time_modified = FALSE;
              memcpy(&gps_time, &time_buf, sizeof(gps_time_t));
              gps_time_modified = TRUE;
            }else if(ubx_state.packet_type == NAV_TIMEGPS){
              static __xdata u8 sv_eph_selector = 0;
              if((++sv_eph_selector) > UBX_GPS_MAX_ID){
                sv_eph_selector = 0;
                poll_aid_hui();
              }else{
                poll_aid_eph(sv_eph_selector);
              }
            }else if((ubx_state.packet_type == NAV_TIMEUTC) && (gps_utc.tm_mday > 0)){
              gps_utc.tm_year -= 1900;
              gps_utc_valid = TRUE;
            }else if((ubx_state.packet_type == AID_EPH) && (svid <= UBX_GPS_MAX_ID)){
              u32 mask = 1;
              mask <<= (svid - 1);
              if(ubx_state.size > (8 + 8)){
                ephemeris_received_gps |= mask;
              }else{
                ephemeris_received_gps &= ~mask;
              }
            }
          }else{ // incorrect checksum
            // do something
          }

          // reset to initial state
          ubx_state.index = 0;
          ubx_state.packet_type = UNKNOWN;
          make_telemetry = FALSE;
          continue; // jump to while loop.
        }

        switch(ubx_state.packet_type){
          case NAV_TIMEGPS:
            if(ubx_state.index == 17){
              leap_seconds = (s8)c;
            }
            break;
          case NAV_TIMEUTC:
            switch(ubx_state.index){
              case 19:
                gps_utc_valid = FALSE;
              case 20:
                *((u8 *)(((u8 *)&gps_utc.tm_year) + (ubx_state.index - 19))) = c;
                break;
              case 21: gps_utc.tm_mon = c - 1; break;
              case 22: gps_utc.tm_mday = c; break;
              case 23: gps_utc.tm_hour = c; break;
              case 24: gps_utc.tm_min = c; break;
              case 25: gps_utc.tm_sec = c; break;
              case 26: if(!(c & 0x04)){gps_utc.tm_mday = 0; /*invalid UTC;*/} break;
            }
            break;
#if GPS_TIME_FROM_RAW_DATA // switch for gps_time source, RXM_RAW or NAV_SOL
          case RXM_RAW:
            switch(ubx_state.index){
              case 7:
              case 8:
              case 9:
              case 10:
                *((u8 *)(((u8 *)&(time_buf.itow_ms)) + (ubx_state.index - 7))) = c;
                break;
              case 11:
              case 12:
                *((u8 *)(((u8 *)&(time_buf.wn)) + (ubx_state.index - 11))) = c;
                break;
              case 13:
                gps_num_of_sat = c;
                break;
            }
            break;
#else
          case NAV_SOL:
            switch(ubx_state.index){
              case 7:
              case 8:
              case 9:
              case 10:
                *((u8 *)(((u8 *)&(time_buf.itow_ms)) + (ubx_state.index - 7))) = c;
                break;
              case 15:
              case 16:
                *((u8 *)(((u8 *)&(time_buf.wn)) + (ubx_state.index - 15))) = c;
                break;
              case 54:
                gps_num_of_sat = c;
                break;
            }
            break;
#endif
          case NAV_SVINFO: {
            if(ubx_state.index < (6 + 8)){break;}
            switch(ubx_state.index % 12){
              case ((6 + 8) % 12) + 2:
                svid = c;
                break;
              case ((6 + 8) % 12) + 3: {
                u32 mask = 1;
                if(svid > UBX_GPS_MAX_ID){break;}
                mask <<= (svid - 1);
                if(c & 0x08){
                  if(!(ephemeris_received_gps & mask)){
                    poll_aid_eph(svid);
                  }
                }else{
                  ephemeris_received_gps &= ~mask;
                }
                break;
              }
            }
            break;
          }
          case AID_EPH: {
            if(ubx_state.index == (6 + 1)){svid = c;}
            break;
          }
        }
        break; 
      }
    }
    
    // check sum update
    ubx_state.ck_a += c;
    ubx_state.ck_b += ubx_state.ck_a;
  }
  
  packet->current = dst;
}

void gps_polling(){
  u8 buf_size = uart0_rx_size();
  for(; buf_size >= (PAGE_SIZE - 1); buf_size -= (PAGE_SIZE - 1)){
    if(!data_hub_assign_page(make_packet)){break;}
  }
}
