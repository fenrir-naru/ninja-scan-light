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

#include "ms5611.h"
#include "main.h"

#include <string.h>

#include "f38x_i2c.h"
#include "data_hub.h"

#define I2C_ADDRESS_RW (0x77 << 1)

static __xdata u8 pressure_data[SYLPHIDE_PAGESIZE - 8];

/*
 * P page design =>
 * 'P', 0, 0, tickcount & 0xFF, // + 4
 * global_ms(4 bytes), // + 8
 * pressure0(big endian, 3 bytes), temperature0(big endain, 3 bytes) // + 14
 * pressure1(big endian, 3 bytes), temperature1(big endain, 3 bytes) // + 20
 * coefficient[1-6] (big endain, 2 * 6 bytes) // + 32
 */

volatile __bit ms5611_capture;
static u8 capture_cycle;

static void send_command(u8 cmd){
  i2c0_read_write(I2C_WRITE(I2C_ADDRESS_RW), &cmd, sizeof(cmd));
}

void ms5611_init(){
  { // Read Coefficient 1-6 from PROM
    u8 i, cmd;
    __xdata u8 *buf;
    for(i = 0, cmd = 0xA2, buf = &(pressure_data[12]); 
        i < 6; 
        ++i, cmd += 2, buf += 2){
      send_command(cmd);
      i2c0_read_write(I2C_READ(I2C_ADDRESS_RW), buf, 2);
    }
  }
  
  send_command(0x42); // Read ADC1 (pressure) with OSR=512
  capture_cycle = 0;
  ms5611_capture = FALSE;
}

static void make_packet(packet_t *packet){
  payload_t *dst = packet->current;
  
  // Check whether buffer size is sufficient
  if((packet->buf_end - dst) < SYLPHIDE_PAGESIZE){
    return;
  }
    
  *(dst++) = 'P';
  *(dst++) = 0;
  *(dst++) = 0;
  *(dst++) = u32_lsbyte(tickcount);
  
  // Record time, LSB first
  memcpy(dst, &global_ms, sizeof(global_ms));
  dst += sizeof(global_ms);
  
  memcpy(dst, pressure_data, sizeof(pressure_data));
  dst += sizeof(pressure_data);
  
  packet->current = dst;
}

void ms5611_polling(){
  
  static __xdata u8 * __xdata next_buf = pressure_data;
  
  if(ms5611_capture){
    ms5611_capture = FALSE;
    
    send_command(0x00);
    i2c0_read_write(I2C_READ(I2C_ADDRESS_RW), next_buf, 3);
    next_buf += 3;
    
    switch(capture_cycle++){
      case 0:
      case 2:
        send_command(0x52); // Read ADC2 (temperature) with OSR=512
        break;
      case 3: // Rotate
        capture_cycle = 0;
        data_hub_assign_page(make_packet);
        next_buf = pressure_data;
      case 1:
        send_command(0x42); // Read ADC1 (pressure) with OSR=512
        break; 
    }
  }
}
