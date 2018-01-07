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
#include "config.h"
#include "f38x_uart0.h"
#include "data_hub.h"
#include "gps.h"
#include "util.h"
#include "telemeter.h"

void gps_write(char *buf, u8 size){
  u8 written = 0;
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

static void set_ubx_cfg_rate(){
#if (defined(SDCC) && (SDCC < 270)) // lower than sdcc-2.7.0 ?
  unsigned char packet[6 + 6] = {
#else
  unsigned char packet[6 + 6];
  static const __code unsigned char _packet[sizeof(packet)] = {
#endif
    0xB5, // packet[0]
    0x62, // packet[1]
    0x06, // packet[2]
    0x08, // packet[3]
    expand_16(sizeof(packet) - 6),   // packet[4 + 0]
#if (defined(SDCC) && (SDCC < 270)) // lower than sdcc-2.7.0 ?
    expand_16(config.gps.rate.measurement_ms),    // packet[6 + 0]
    expand_16(config.gps.rate.navigation_cycles), // packet[6 + 2]
#else
    expand_16(0), // packet[6 + 0]
    expand_16(0), // packet[6 + 2]
#endif
    expand_16(1), // packet[6 + 4], alignment to reference time (!0)
  };
#if !(defined(SDCC) && (SDCC < 270)) // Not lower than sdcc-2.7.0?
  memcpy(packet, _packet, sizeof(packet));
  *(u16 *)(&packet[6 + 0]) = le_u16(config.gps.rate.measurement_ms);
  *(u16 *)(&packet[6 + 2]) = le_u16(config.gps.rate.navigation_cycles);
#endif
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
    0x03, // packet[6 + 0]; mode, SBAS Enabled (bit0 = 1), and allow test mode (bit1=1)
    0, // packet[6 + 1]; usage (ignore range, correction, integrity)
    3, // packet[6 + 2]; maxsbas
    0, // packet[6 + 3]; scanmode2
    expand_32(0), // packet[6 + 4], scanmode1
  };
  gps_packet_write(packet, sizeof(packet));
}

static void set_ubx_cfg_prt(u32 baudrate){
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
#if (defined(SDCC) && (SDCC < 270)) // lower than sdcc-2.7.0 ?
    expand_32(baudrate), // packet[6 + 8], baudrate
#else
    expand_32(0), // packet[6 + 8], baudrate
#endif
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
  *(u32 *)(&packet[6 + 8]) = le_u32(baudrate);
#endif
  
  {
    u8 i;
    for(i = 0; i < 3; i++){
      packet[6 + 0]  = i;
      gps_packet_write(packet, sizeof(packet));
    }
  }
  
  // Reference: NMEA
  /*const char code *str1 = "$PUBX,41,0,0007,0003,115200,1*1A\r\n";
  const char code *str2 = "$PUBX,41,1,0007,0003,115200,1*1B\r\n";
  const char code *str3 = "$PUBX,41,2,0007,0003,115200,1*18\r\n";  
  gps_write(str1, strlen(str1));
  gps_write(str2, strlen(str2));  
  gps_write(str3, strlen(str3));*/
}

static void set_ubx_cfg_msg(ubx_cfg_t *message){
  unsigned char packet[6 + 3] = { // 3 byte payload CFG-MSG, which will change message rate for the current port.
    0xB5, // packet[0] 
    0x62, // packet[1] 
    0x06, // packet[2] 
    0x01, // packet[3] 
    expand_16(sizeof(packet) - 6), // packet[4 + 0]  
    message->msg_class, // packet[6 + 0]
    message->msg_id,    // packet[6 + 1]
    message->rate,      // packet[6 + 2]
  };
  gps_packet_write(packet, sizeof(packet));
}

void gps_init(){
  
  // init wait
  wait_ms(100);
  
  // set U-blox configuration
  set_ubx_cfg_prt(config.baudrate.gps);  // baudrate change
  while(uart0_tx_active());
  uart0_bauding(config.baudrate.gps);
  
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
  
  {
    u8 i;
#define PROP config.gps.message
    for(i = 0; i < sizeof(PROP) / sizeof(PROP[0]); ++i){
      if((PROP[i].msg_class == 0) || (PROP[i].msg_id == 0)){continue;}
      set_ubx_cfg_msg(&PROP[i]);
    }
#undef PROP
  }
  
  data_hub_send_config("GPS.CFG", uart0_write);
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
__xdata gps_time_t gps_time = {-1, 0};
__xdata u8 gps_num_of_sat = 0;

static __xdata s8 leap_seconds = 0;
#if USE_GPS_STD_TIME
time_t gps_std_time(time_t *timer) {
  time_t res = 0;
  if(gps_time.wn >= 0){ // valid
   res = ((time_t)60 * 60 * 24 * 7) * (u16)(gps_time.wn)
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
  UNKNOWN = 0,
  NAV_SOL, NAV_TIMEGPS, NAV_TIMEUTC, NAV_SVINFO,
  RXM_RAW, RXM_SFRB,
  AID_HUI, AID_EPH,
} packet_type_t;

static void push_telemetry(char c){
  static __xdata struct {
    char header;
    char content[SYLPHIDE_PAGESIZE - 1];
  } buf = {{'G'}};
  static __xdata unsigned char index = 0;
  buf.content[index++] = c;
  if(index >= sizeof(buf.content)){
    telemeter_send((char *)&buf);
    index = 0;
  }
}

#define UBX_GPS_MAX_ID 32
static void make_packet(packet_t *packet){
  payload_t *dst = packet->current;
  u8 size = packet->buf_end - dst;
  
  //if(size == 0){return;} // guaranteed that size > 0
  
  *(dst++) = 'G';

  // read data and store it into packet
  size = uart0_read(dst, --size);
    
  // decode data in order to extract GPS time, request ephemeris, and so on.
  while(size-- > 0){
    
    static __xdata struct {
      u16 index, size;
      u8 ck_a, ck_b; // チェックサム用
      packet_type_t packet_type;
    } ubx_state = {0};
    
    static __xdata u32 ephemeris_received_gps = 0;
    static __xdata u8 svid;
    static __xdata gps_time_t time_buf;
    static __xdata u8 num_of_sat_buf;
    
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
        {
          u8 i;
#define PROP config.telemetry_truncate.g_page.item
          static __xdata unsigned char count[sizeof(PROP) / sizeof(PROP[0])] = {0};
          for(i = 0; i < sizeof(PROP) / sizeof(PROP[0]); ++i){
            if(PROP[i].msg_class != ubx_state.ck_a){continue;}
            if(PROP[i].msg_id != c){continue;}
            if((++(count[i])) >= PROP[i].rate){
              make_telemetry = TRUE;
              count[i] = 0;
            }
            break;
          }
#undef PROP
        }
        switch(ubx_state.ck_a){
          case 0x01:
            switch(c){
              case 0x06: ubx_state.packet_type = NAV_SOL; break;
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
        { /* packet size check */
          u8 size_true = 0;
          switch(ubx_state.packet_type){
            case NAV_SOL:     size_true = (8 + 52); break;
            case NAV_TIMEGPS: size_true = (8 + 16); break;
            case NAV_TIMEUTC: size_true = (8 + 20); break;
          }
          if((size_true > 0) && (ubx_state.size != size_true)){
            // invalid packet, reset to initial state
            ubx_state.index = 0;
            ubx_state.packet_type = UNKNOWN;
            make_telemetry = FALSE;
          }
        }
        break;
      default: {
        if(ubx_state.index >= (ubx_state.size - 1)){
          if(ubx_state.index == (ubx_state.size - 1)){
            if(ubx_state.ck_a == c){ // partially correct checksum
              continue; // jump to while loop.
            }
          }else if(ubx_state.ck_b == c){ // correct checksum
            if(ubx_state.packet_type == (GPS_TIME_FROM_RAW_DATA ? RXM_RAW : NAV_SOL)){
              u16 ms = time_buf.itow_ms % 1000;
              if((ms >= 200) && (ms <= 800)){
                time_buf.itow_ms += (1000 - ms);
                if(time_buf.itow_ms >= (u32)60 * 60 * 24 * 7 * 1000){
                  time_buf.itow_ms = 0;
                  time_buf.wn++;
                }
                gps_time_modified = FALSE;
                memcpy(&gps_time, &time_buf, sizeof(gps_time_t));
                gps_time_modified = TRUE;
              }
              gps_num_of_sat = num_of_sat_buf;
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
                num_of_sat_buf = c;
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
                num_of_sat_buf = c;
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
  for(; buf_size >= (SYLPHIDE_PAGESIZE - 1); buf_size -= (SYLPHIDE_PAGESIZE - 1)){
    if(!data_hub_assign_page(make_packet)){break;}
  }
}
