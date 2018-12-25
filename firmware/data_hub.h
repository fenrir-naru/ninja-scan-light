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

#ifndef __DATA_HUB_H__
#define __DATA_HUB_H__

#include "ff.h"
#include "type.h"

typedef char __xdata payload_t;
typedef unsigned char payload_size_t;

typedef struct{
  payload_t *buf_begin;
  payload_t *current;
  payload_t *buf_end;
} __xdata packet_t;

#define SYLPHIDE_PAGESIZE 32 // Do not change

extern const u8 sylphide_protocol_header[2];

/* Maximum log size in bytes per a file
 * For example, (1UL << 30) means log increases up to 1GB
 */
#define MAXIMUM_LOG_DAT_FILE_SIZE (1UL << 30)

/* Incremental log file name policy
 * The "incremental" means log.NNN (N is digit).
 * '1' uses "log.inc" file to assign NNN with "log.inc" file size.
 * '0' ignores "log.inc".
 * ('1' without "log.inc" equals to '0'.)
 * In both settings, when current log file reaches maximum size,
 * a new incremental file name is automatically assigned to continue to record.
 * For example,
 * '1': (power up, and log.inc is 3 bytes) => log.003 (at the same time, log.inc increases to 4 bytes)
 *      =(Event A or B) => log.004 (at the same time, log.inc increases to 5 bytes)
 *      =(Event A or B) => log.005 ...
 * '0': (power up) => log.dat
 *      =(Event B)=> log.000
 *      =(Event B)=> log.001 ...
 * Event A) power down then up, or USB connect to PC then disconnect
 * Event B) exceed maximum file size
 *
 * Note: log.999 is maximum, whose next is log.000 (roll over).
 */
#define CHECK_INCREMENT_LOG_DAT 1

/* 1 is to use function of direct connection
 * to UART.0(GPS), UART1(Telemeter), ...
 */
#define USE_DIRECT_CONNECTION 1

void data_hub_init();
long data_hub_read_long(FIL *);
void data_hub_send_config(char *fname, unsigned char (*send_func)(char *buf, unsigned char size));
void data_hub_load_config(char *fname, void (*load_func)(FIL *file));
payload_size_t data_hub_assign_page(void (*call_back)(packet_t *));
void data_hub_polling();

#endif /* __DATA_HUB_H__ */
