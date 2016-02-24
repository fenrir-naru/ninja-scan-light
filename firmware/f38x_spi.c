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

// SFR declarations
#include "c8051f380.h"

#include "f38x_spi.h"

u8 spi_ckr(u8 new_value){
  u8 old_value = SPI0CKR;
  SPI0CKR = new_value;
  return old_value;
}

/**
 * Configure SPI0 for 8-bit, 400KHz SCK, Master mode, polled operation, data
 * sampled on 1st SCK rising edge.
 * 
 */
void spi_init(){
  /*
   * data sampled on rising edge, clk active low,
   * 8-bit data words, master mode;
   */
  SPI0CFG = 0x70;
  
  // 4-wire mode; SPI enabled; flags cleared
  SPI0CN = 0x0F;
  
  // SPI clock = 400K
  spi_clock(400);
}

void spi_send_8clock(){
  SPI0DAT = 0xFF;
  while(!SPIF);
  SPIF = 0;
}

/**
 * Function sends one byte to spi and reads ony byte from spi
 * it will be written with SYSCLK as clk
 * 
 * @param byte value to write
 * @return SPI byte
 */
unsigned char spi_write_read_byte(unsigned char byte){
  SPI0DAT = byte;
  while(!SPIF);                      
  SPIF = 0;
  return SPI0DAT;
}

void spi_read(unsigned char * pchar, unsigned int length){
  while(length--){
    SPI0DAT = 0xFF;
    while(!SPIF);
    SPIF = 0;
    *(pchar++) = SPI0DAT;
  }
}

void spi_write(unsigned char * pchar, unsigned int length){
  while(length--){
    SPI0DAT = *(pchar++);
    while(!SPIF);
    SPIF = 0;
  }
}
