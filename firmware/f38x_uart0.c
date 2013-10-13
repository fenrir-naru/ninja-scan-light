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

#include <stdio.h>

#include "main.h"
#include "f38x_uart0.h"
#include "fifo.h"

#define DEFAULT_BAUDRATE     9600UL  // Baud rate of UART in bps

#define CRITICAL_UART0(func) \
{\
  ES0 = 0;\
  func;\
  ES0 = 1;\
}

__xdata fifo_char_t fifo_tx0; ///< FIFO TX
__xdata fifo_char_t fifo_rx0; ///< FIFO RX
static __xdata __at (0x000) 
  char buffer_tx0[UART0_TX_BUFFER_SIZE];
static __xdata __at (0x000 + UART0_TX_BUFFER_SIZE) 
  char buffer_rx0[UART0_RX_BUFFER_SIZE];

/**
 * Change UART0 baudrate
 */
void uart0_bauding(unsigned long baudrate){
  unsigned long selector1 = SYSCLK/baudrate/2;
  unsigned long selector2 = selector1 / 256;
  
  TR1 = 0;
  
  if (selector2 < 1){
    TH1 = 0x0ff & (-selector1);
    CKCON |=  0x08;                  // T1M = 1; SCA1:0 = xx
  }else if (selector2 < 4){
    TH1 = 0x0ff & (-(selector1/4));
    CKCON &= ~0x0B;                  
    CKCON |=  0x01;                  // T1M = 0; SCA1:0 = 01
  }else if (selector2 < 12){
    TH1 = 0x0ff & (-(selector1/12));
    CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
  }else{
    TH1 = 0x0ff & (-(selector1/48));
    CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
    CKCON |=  0x02;
  }
  
  TL1 = TH1;        // init Timer1
  TMOD &= ~0xf0;    // TMOD: timer 1 in 8-bit autoreload
  TMOD |=  0x20;
  TR1 = 1;
}

/**
 * Initialize UART0
 */
void uart0_init() {
  SCON0 = 0x10;     // SCON0: 8-bit variable bit rate
                    //        level of STOP bit is ignored
                    //        RX enabled
                    //        ninth bits are zeros
                    //        clear RI0 and TI0 bits
  fifo_char_init(&fifo_tx0, buffer_tx0, UART0_TX_BUFFER_SIZE); 
  fifo_char_init(&fifo_rx0, buffer_rx0, UART0_RX_BUFFER_SIZE); 

  uart0_bauding(DEFAULT_BAUDRATE);

  TB80 = 0;         // TB80は書込み中フラグは0(書込みしていない)
  ES0 = 1;          // 割り込み有効
  //PS0 = 1;          // 優先度1
}

/**
 * Regist sending data via UART0
 * 
 * @param data pointer to data
 * @param size size of data
 * @return (FIFO_SIZE_T) the size of registed data to buffer
 */
FIFO_SIZE_T uart0_write(char *buf, FIFO_SIZE_T size){
  // TB80は書込み中フラグとして使う
  // 0(書込みしていない)だったら手動割り込みをかける
  if(size){
    size = fifo_char_write(&fifo_tx0, buf, size);
    CRITICAL_UART0(
      if(!(SCON0 & 0x0A)){ // !TB80 && !TI0
        TI0 = 1;
        //interrupt_uart0(); // 手動で割込みをかける
      }
    );
  }
  return size;
}

/**
 * Return the size of untransimitted data via UART0
 * 
 * @return (FIFO_SIZE_T) the size
 */
FIFO_SIZE_T uart0_tx_size(){
  return fifo_char_size(&fifo_tx0);
}

/**
 * Get the recieved data via UART0
 * 
 * @param buf buffer
 * @param size the size of buffer
 * @return (FIFO_SIZE_T) the real size of grabbed data
 */
FIFO_SIZE_T uart0_read(char *buf, FIFO_SIZE_T size){
  return fifo_char_read(&fifo_rx0, buf, size);
}

/**
 * Return the size of ungrabbed data via UART0
 * 
 * @return (FIFO_SIZE_T) the size
 */
FIFO_SIZE_T uart0_rx_size(){
  return fifo_char_size(&fifo_rx0);
}

///< Interrupt(TI0 / RI0)
void interrupt_uart0 () __interrupt (INTERRUPT_UART0) {
  unsigned char res;

  if(RI0){
    RI0 = 0;
    /* push to ring buffer */
    FIFO_DIRECT_PUT(fifo_rx0, SBUF0, res);
    if(res == 0){
      // TODO: buffer overflow
    }
  }

  if(TI0){
    TI0 = 0;
    /* has byte to write? */
    FIFO_DIRECT_GET(fifo_tx0, SBUF0, res);
    if(res > 0){
      TB80 = 1; // TB80 is used as UART0 writing flag. set 1 (writing)
    }else{
      TB80 = 0; // TB80 is used as UART0 writing flag. set 0 (not writing)
    }
  }
}


// stdio.h support

/**
 * putchar - No blocking
 */
void putchar (char c){
  while(uart0_write(&c, 1) == 0);
}

/**
 * getchar - blocking
 */
char getchar (void){
  char c;
  while(uart0_read(&c, sizeof(c)) == 0);
  return c;
}

