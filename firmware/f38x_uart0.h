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

#ifndef _UART0_H_
#define _UART0_H_

#include "type.h"
#include "main.h"

void uart0_bauding_config(u16 baudrate_register);
#define uart0_bauding(baudrate) \
  uart0_bauding_config((u16)(SYSCLK/2/baudrate))
void uart0_init();

#define UART0_TX_BUFFER_SIZE 32
#define UART0_RX_BUFFER_SIZE (0x100 - 32)

#include "c8051F380.h"
#include "fifo.h"

FIFO_SIZE_T uart0_write(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart0_read(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart0_tx_margin();
FIFO_SIZE_T uart0_rx_size();

void interrupt_uart0() __interrupt (INTERRUPT_UART0);

#define uart0_tx_active() (TB80 == 1)

// For stdio.h
char getchar();
void putchar(char c);

#endif
