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

#include "c8051f380.h"    // SFR declarations

#include "f38x_i2c0.h"
#include "type.h"

/**
 * Configure i2c0 with Timer 0
 * 
 */
void i2c0_init(){
  // timer 0 setup => mode 2, 8bit w autoreload
  TMOD &= ~0x0F;
  TMOD |=  0x02;
  
  CKCON |= 0x04; // => use SYSCLK
  TH0 = TL0 = (0x100 - 40); // => freq: 48M / 40 = 1.2M
  //TH0 = TL0 = 0x00; // => freq: 48M / 256 
  TR0 = 1;
  
  // inhibit slave, extend setup / hold time, use timer 0 overflow
  SMB0CF = 0x40; // => 1.2M / 3clk = 400KHz (Eq. 22.2)
  //SMB0CF = (0x40 | 0x10); // => 1.2M / 12+11clk = 50KHz
  
  SMB0CF |= 0x80; // enable
}

/**
 * Read or Write via i2c0
 * 
 * @return remain bytes
 */
u8 i2c0_read_write(u8 address_wr_flag, u8 *buf, u8 size){
  
  // start bit
  STA0 = 1;
  while(!(SMB0CN & 0x05));
  STA0 = 0;
  
  if((SMB0CN & 0xDC) == 0xC0){
    // address + R/W flag
    SMB0DAT = address_wr_flag;
    SI0 = 0;
    while(!(SMB0CN & 0x05));
    
    if((SMB0CN & 0xFE) == 0xC2){
      // Read / Write
      if(address_wr_flag & 0x01){ 
        while(1){
          SI0 = 0;
          while(!(SMB0CN & 0x05));
          if((SMB0CN & 0xFC) == 0x88){
            *(buf++) = SMB0DAT;
            if(--size){
              ACK0 = 1;
            }else{
              ACK0 = 0;
              break;
            }
          }else{break;}
        }
      }else{
        while(size--){
          SMB0DAT = *(buf++);
          SI0 = 0;
          while(!(SMB0CN & 0x05));
          if((SMB0CN & 0xFE) != 0xC2){break;}
        }
      }
    }
  }
  STO0 = 1;
  SI0 = 0;
  
  return size;
}
