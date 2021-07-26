#include "all_sensors_elvr.h"
#include "main.h"

#include <string.h>

#include "f38x_i2c.h"
#include "data_hub.h"
#include "util.h"
#include "c8051f380.h"

#define I2C_ADDRESS_RW (0x78 << 1)

static __xdata u8 as_elvr_data[SYLPHIDE_PAGESIZE - 8];

/*
 * X page design =>
 * 'X', 0, 0, tickcount & 0xFF, // + 4
 * global_ms(4 bytes), // + 8
 * [0..11] pressure value(2bytes, big endian?) // + 32
 */

volatile __bit as_elvr_capture;

//#define i2c_read_write i2c0_read_write
#define i2c_read_write i2c1_read_write

void as_elvr_init(){
  as_elvr_capture = FALSE;
}

static void make_packet(packet_t *packet){
  payload_t *dst = packet->current;
  
  // Check whether buffer size is sufficient
  if((packet->buf_end - dst) < SYLPHIDE_PAGESIZE){
    return;
  }
    
  *(dst++) = 'X';
  *(dst++) = 1;
  *(dst++) = 0;
  *(dst++) = u32_lsbyte(tickcount);
  
  // Record time, LSB first
  memcpy(dst, &global_ms, sizeof(global_ms));
  dst += sizeof(global_ms);
  
  memcpy(dst, as_elvr_data, sizeof(as_elvr_data));
  dst += sizeof(as_elvr_data);
  
  packet->current = dst;
}

void as_elvr_polling(){
  
  static __xdata u8 * __xdata next_buf = as_elvr_data;
  
  if(as_elvr_capture){
    as_elvr_capture = FALSE;
    
    i2c_read_write(I2C_READ(I2C_ADDRESS_RW), next_buf, 2);
    next_buf += 2;

    if(next_buf == (as_elvr_data + sizeof(as_elvr_data))){
      // Rotate
      data_hub_assign_page(make_packet);
      next_buf = as_elvr_data;
    }
  }
}
