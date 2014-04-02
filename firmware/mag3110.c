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

#include "mag3110.h"
#include "main.h"

#include <string.h>

#include "f38x_i2c0.h"
#include "data_hub.h"

#define I2C_ADDRESS_RW (0x0E << 1)

static __xdata u8 mag_data[PAGE_SIZE - 8];

/*
 * M page design =>
 * 'M', 0x80, 0, tickcount & 0xFF, // + 4
 * global_ms(4 bytes), // + 8
 * mag_XYZ[0-3](little endian, 2 * 3 bytes) // + 32
 */

volatile __bit mag3110_capture;

void mag3110_init(){
  
  { // AUTO_MRST_EN, RAW
    u8 scale_cmd[] = {0x11, 0xA0};
    i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), scale_cmd, sizeof(scale_cmd));
  }
  { // ACTIVE MODE
    u8 mode_cmd[] = {0x10, 0x01};
    i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), mode_cmd, sizeof(mode_cmd));
  }
  
  mag3110_capture = FALSE;
}

static void make_packet(packet_t *packet){
  payload_t *dst = packet->current;
  
  // Check whether buffer size is sufficient
  if((packet->buf_end - dst) < PAGE_SIZE){
    return;
  }
    
  *(dst++) = 'M';
  *(dst++) = 0x80; // Little endian mode
  *(dst++) = 0;
  *(dst++) = u32_lsbyte(tickcount);
  
  // Record time, LSB first
  memcpy(dst, &global_ms, sizeof(global_ms));
  dst += sizeof(global_ms);
  
  memcpy(dst, mag_data, sizeof(mag_data));
  dst += sizeof(mag_data);
  
  packet->current = dst; 
}

void mag3110_polling(){
  
  static __xdata u8 * __xdata next_buf = mag_data;
  
  if(mag3110_capture){
    mag3110_capture = FALSE;
    
    {
      u8 move_pointer_cmd[] = {0x01};
      i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), move_pointer_cmd, sizeof(move_pointer_cmd));
    }
    i2c0_read_write(I2C_READ(I2C_ADDRESS_RW), next_buf, 6);
    {
      // swap x and y
      u8 tmp;
      tmp = next_buf[0];
      next_buf[0] = next_buf[2];
      next_buf[2] = tmp;
      tmp = next_buf[1];
      next_buf[1] = next_buf[3];
      next_buf[3] = tmp;
    }
    next_buf += 6;
    
    // Rotate
    if(next_buf >= mag_data + sizeof(mag_data)){
      data_hub_assign_page(make_packet);
      next_buf = mag_data;
    }
  }
}
