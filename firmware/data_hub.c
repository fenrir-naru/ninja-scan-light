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
#include <string.h>

#include "main.h"
#include "data_hub.h"
#include "util.h"
#include "usb_cdc.h"
#include "f38x_usb.h"
#include "ff.h"
#include "diskio.h"
#include "f38x_uart1.h"

static packet_t packet;

static void packet_init(
    packet_t *p, 
    payload_t *buf,
    payload_size_t max_size){
  
  p->buf_begin = p->current = buf;
  p->buf_end = p->current + max_size;
}

#define BUFFER_SIZE (PAGE_SIZE * 16 * 2) // "*2" means double buffer
#define PAGES (BUFFER_SIZE / PAGE_SIZE)

static payload_t payload_buf[BUFFER_SIZE];

static payload_t * __xdata free_page;
static payload_t * __xdata locked_page;

payload_size_t data_hub_assign_page(void (* packet_maker)(packet_t *)){
  payload_t *next_free_page = free_page + PAGE_SIZE;
  if(next_free_page >= (payload_buf + sizeof(payload_buf))){
    next_free_page -= sizeof(payload_buf);
  }
  
  if(next_free_page == locked_page){return 0;}
  
  packet_init(&packet, free_page, PAGE_SIZE);
  packet_maker(&packet);
  if(packet.current == packet.buf_begin){
    return 0;
  }

  free_page = next_free_page;

  do{
#define whether_send_telemetry(page, frequency) \
if(*packet.buf_begin == page){ \
  static __xdata unsigned char count = 0; \
  if(++count < frequency){break;} \
  count = 0; \
}
    whether_send_telemetry('A', 20) // 'A' page : approximately 5 Hz
    else whether_send_telemetry('P', 2) // 'P' page : approximately 1 Hz
    else whether_send_telemetry('M', 2) // 'M' page : approximately 1 Hz
    else break;
    data_hub_send_telemetry(packet.buf_begin);
  }while(0);
  
  return PAGE_SIZE;
}

static void force_cdc(FIL *f){
  cdc_force = TRUE;
}

static FIL file;
FATFS __at (0x01D0) fs;

static __bit log_file_opened;

void data_hub_load_config(char *fname, void (* func)(FIL *)){
  if(f_mount(0, &fs) == FR_OK){
    if(f_open(&file, fname, (FA_OPEN_EXISTING | FA_READ)) == FR_OK){
      func(&file);
      f_close(&file);
    }
    f_mount(0, NULL);
  }
}

static __xdata u16 log_block_size; // Must be multiple number of PAGE_SIZE

void data_hub_init(){
  log_file_opened = FALSE;
  free_page = locked_page = payload_buf;
  log_block_size = PAGE_SIZE;

  disk_initialize(0);

  data_hub_load_config("FORCE.CDC", force_cdc);
}

static u16 log_to_file(){
  u16 accepted_bytes;
  
  f_write(&file,
    locked_page,
    log_block_size, &accepted_bytes);
  {
    static __xdata u8 loop = 0;
    if((++loop) == 64){
      loop = 0;
      f_sync(&file);
    }
  }
  
  return accepted_bytes;
}

static const u8 protocol_header[] = {0xF7, 0xE0};

static u16 log_to_host(){
  static __xdata u16 sequence_num = 0;
  u16 crc = crc16(locked_page, log_block_size, 
      crc16((u8 *)&(++sequence_num), sizeof(sequence_num), 0));
  if(!(cdc_tx(protocol_header, sizeof(protocol_header))
      && cdc_tx((u8 *)&sequence_num, sizeof(sequence_num))
      && (cdc_tx(locked_page, log_block_size) == log_block_size)
      && cdc_tx((u8 *)&crc, sizeof(crc)))){
    return 0;
  }
  return log_block_size;
}

void data_hub_send_telemetry(char buf[PAGE_SIZE]){
  static __xdata u16 sequence_num = 0;
  u16 crc = crc16(buf, PAGE_SIZE,
      crc16((u8 *)&(++sequence_num), sizeof(sequence_num), 0));
  if(uart1_tx_margin() < (
      sizeof(protocol_header) + sizeof(sequence_num) + PAGE_SIZE + sizeof(crc))){
    return;
  }
  uart1_write(protocol_header, sizeof(protocol_header));
  uart1_write((u8 *)&sequence_num, sizeof(sequence_num));
  uart1_write(buf, PAGE_SIZE);
  uart1_write((u8 *)&crc, sizeof(crc));
}

static u8 open_file(){
  char fname[] = "log.dat";
  if(f_mount(0, &fs) != FR_OK){return FALSE;}
#if CHECK_INCREMENT_LOG_DAT
  if(f_open(&file, "LOG.INC", (FA_OPEN_EXISTING | FA_WRITE)) == FR_OK){
    f_lseek(&file, file.fsize);
    {
      u16 num = (file.fsize % 1000);
      u8 i, j;
      for(i = 0, j = 6; i < 3; i++, j--){
        fname[j] = '0' + (num % 10);
        num /= 10;
      } // Generate file name such as log.000
      f_write(&file, "*", 1, &num); // Add 1 byte to log.inc
    }
    f_close(&file);
  }
#endif
  if(f_open(&file, fname, (FA_OPEN_ALWAYS | FA_WRITE)) != FR_OK){
    return FALSE;
  }
  f_lseek(&file, file.fsize);
  return TRUE;
}

void data_hub_polling() {
  
  __code u16 (* log_func)() = NULL;
  
  switch(usb_mode){
    case USB_INACTIVE:
    case USB_CABLE_CONNECTED:
      if(!log_file_opened){
        if(!open_file()){break;}
        log_file_opened = TRUE;
        log_block_size = BUFFER_SIZE / 2;
        free_page = locked_page = payload_buf;
        return;
      }
      log_func = log_to_file;
      break;
    case USB_CDC_ACTIVE:
      if(log_block_size != PAGE_SIZE){
        log_block_size = PAGE_SIZE;
        free_page = locked_page = payload_buf;
        return;
      }
      log_func = log_to_host;
      break;
    case USB_MSC_ACTIVE:
      if(log_file_opened){
        log_file_opened = FALSE;
        f_close(&file);
        f_mount(0, NULL);
      }
      break;
  }
    
  // Dump when the data size exceeds predefined boundary.
  while(TRUE){
    payload_t * next_locked_page = locked_page + log_block_size;
    if(next_locked_page >= (payload_buf + sizeof(payload_buf))){
      next_locked_page -= sizeof(payload_buf);
    }
    
    if((next_locked_page > locked_page) 
        ? ((free_page >= locked_page) && (free_page < next_locked_page))
        : !((free_page >= next_locked_page) && (free_page < locked_page))){
      break;
    }
    
    if(log_func){
      if(log_func()){
        __critical {
          sys_state |= SYS_LOG_ACTIVE;
        }
      }
    }
    
    locked_page = next_locked_page;
  }
}
