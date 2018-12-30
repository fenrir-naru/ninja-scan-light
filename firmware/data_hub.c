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
#include "config.h"
#include "data_hub.h"
#include "util.h"
#include "usb_cdc.h"
#include "f38x_usb.h"
#include "ff.h"
#include "diskio.h"
#include "f38x_uart0.h"
#include "f38x_uart1.h"
#include "telemeter.h"

static packet_t packet;

static void packet_init(
    packet_t *p, 
    payload_t *buf,
    payload_size_t max_size){
  
  p->buf_begin = p->current = buf;
  p->buf_end = p->current + max_size;
}

#define BUFFER_SIZE (SYLPHIDE_PAGESIZE * 16 * 2) // "*2" means double buffer
#define PAGES (BUFFER_SIZE / SYLPHIDE_PAGESIZE)

static payload_t payload_buf[BUFFER_SIZE];

static payload_t * __xdata free_page;
static payload_t * __xdata locked_page;

payload_size_t data_hub_assign_page(void (* packet_maker)(packet_t *)){
  payload_t *next_free_page = free_page + SYLPHIDE_PAGESIZE;
  if(next_free_page >= (payload_buf + sizeof(payload_buf))){
    next_free_page -= sizeof(payload_buf);
  }
  
  if(next_free_page == locked_page){return 0;}
  
  packet_init(&packet, free_page, SYLPHIDE_PAGESIZE);
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
    whether_send_telemetry('A', config.telemetry_truncate.a_page) // 'A' page : approximately 5 Hz
    else whether_send_telemetry('P', config.telemetry_truncate.p_page) // 'P' page : approximately 1 Hz
    else whether_send_telemetry('M', config.telemetry_truncate.m_page) // 'M' page : approximately 1 Hz
    else break;
    telemeter_send(packet.buf_begin);
  }while(0);
  
  return SYLPHIDE_PAGESIZE;
}

static void force_cdc(FIL *f){
  cdc_force = TRUE;
}

static const char renew_config_fname[] = "RENEW.CFG";
static void set_new_config(FIL *f){
  UINT buf_filled;
  if((f_size(f) == sizeof(config_t))
      && (f_read(f, payload_buf, sizeof(config_t), &buf_filled) == FR_OK) // temporary sharing of payload_buf
      && (sizeof(config_t) == buf_filled)){
#if _FS_MINIMIZE < 1 // rename to another name for overwrite prevention
    f_close(f); // intentionally procedure before f_close() in data_hub_load_config().
    f_rename(renew_config_fname, "RENEWED.CFG");
#else // set illegal size for overwrite prevention
#define str_and_len(str) str, (sizeof(str) - 1)
    f_write(f, str_and_len("!RENEWED!"), &buf_filled);
#undef str_and_len
#endif
    config_renew((config_t *)payload_buf);
  }
}

long data_hub_read_long(FIL *f){
  // extract integer number from file, multiple invocation is supported.
  char buf[16];
  u16 read_bytes;
  long res = 0;
  if(f_read(f, buf, sizeof(buf) - 1, &read_bytes) == FR_OK){
    char *endptr;
    buf[read_bytes] = '\0';
    res = str2num(buf, &endptr);
    f_lseek(f, f_tell(f) - read_bytes + (endptr - buf));
  }
  return res;
}

static FIL file;
FATFS __at (0x01D0) fs;

void data_hub_send_config(char *fname, unsigned char (*send_func)(char *buf, unsigned char size)){
  if((!send_func) || (f_mount(0, &fs) != FR_OK)){return;}
  if(f_open(&file, fname, (FA_OPEN_EXISTING | FA_READ)) == FR_OK){
    char buf[16], *_buf;
    UINT buf_filled;
    while(f_read(&file, buf, sizeof(buf), &buf_filled) == FR_OK){
      char *_buf = buf;
      if(buf_filled == 0){break;}
      do{
        unsigned char sent = send_func(_buf, buf_filled);
        _buf += sent;
        buf_filled -= sent;
      }while(buf_filled > 0);
    }
    f_close(&file);
  }
  f_mount(0, NULL);
}

void data_hub_load_config(char *fname, void (*load_func)(FIL *file)){
  if((!load_func) || (f_mount(0, &fs) != FR_OK)){return;}
  if(f_open(&file, fname, (FA_OPEN_EXISTING | FA_WRITE | FA_READ)) == FR_OK){
    load_func(&file);
    f_close(&file);
  }
  f_mount(0, NULL);
}

static __bit log_file_opened;
static __xdata u16 log_block_size; // Must be multiple number of SYLPHIDE_PAGESIZE


#if USE_DIRECT_CONNECTION

static __xdata u8 direct_uart_port_num;

static void direct_uart_change_spec(){
  if(direct_uart_port_num == 0){
    uart0_bauding(cdc_line_coding.baudrate.i);
  }else{
    uart1_bauding(cdc_line_coding.baudrate.i);
  }
}

static void direct_uart_loop(){
  char buf[32];
  FIFO_SIZE_T (*uart_write)(char *buf, FIFO_SIZE_T size);
  FIFO_SIZE_T (*uart_read)(char *buf, FIFO_SIZE_T size);
  if(direct_uart_port_num == 0){
    uart_write = uart0_write;
    uart_read = uart0_read;
  }else{
    uart_write = uart1_write;
    uart_read = uart1_read;
  }
  while(1){
    usb_polling();
    cdc_tx(buf, (u16)uart_read(buf, sizeof(buf)));
    uart_write(buf, (u8)cdc_rx(buf, sizeof(buf)));
  }
}

static void direct_uart_init(){
  cdc_force = TRUE;
  main_loop_prologue = direct_uart_loop;
  cdc_change_line_spec = direct_uart_change_spec;
}

static void direct_uart0_init(FIL *f){
  direct_uart_port_num = 0;
  cdc_line_coding.baudrate.i = le_u32(config.baudrate.gps);
  direct_uart_init();
}
static void direct_uart1_init(FIL *f){
  direct_uart_port_num = 1;
  cdc_line_coding.baudrate.i = le_u32(config.baudrate.telemeter);
  direct_uart_init();
}

#endif

void data_hub_init(){
  log_file_opened = FALSE;
  free_page = locked_page = payload_buf;
  log_block_size = SYLPHIDE_PAGESIZE;

  disk_initialize(0);

  data_hub_load_config(renew_config_fname, set_new_config);
  data_hub_load_config("FORCE.CDC", force_cdc);

#if USE_DIRECT_CONNECTION
  data_hub_load_config("DIRECT.GPS", direct_uart0_init);
  data_hub_load_config("DIRECT.TLM", direct_uart1_init);
#endif
}

static __xdata s16 log_file_suffix = -1;

static u8 open_file();

static u16 log_to_file(){
  u16 accepted_bytes;
  
  f_write(&file,
    locked_page,
    log_block_size, &accepted_bytes);
  {
    static __xdata u8 loop = 0;
    if((++loop) == 64){
      loop = 0;

      if(file.fsize < MAXIMUM_LOG_DAT_FILE_SIZE){
        f_sync(&file);
      }else{
        // close current log file when its size exceeds predefined bytes
        f_close(&file); // internally f_sync(&file) is invoked
        log_file_suffix++;
        if(!open_file()){ // try to open another file
          log_file_opened = FALSE;
        }
      }
    }
  }
  
  return accepted_bytes;
}

const u8 sylphide_protocol_header[2] = {0xF7, 0xE0};

static u16 log_to_host(){
  static __xdata u16 sequence_num = 0;
  u16 crc = crc16(locked_page, log_block_size, 
      crc16((u8 *)&(++sequence_num), sizeof(sequence_num), 0));
  if(!(cdc_tx(sylphide_protocol_header, sizeof(sylphide_protocol_header))
      && cdc_tx((u8 *)&sequence_num, sizeof(sequence_num))
      && (cdc_tx(locked_page, log_block_size) == log_block_size)
      && cdc_tx((u8 *)&crc, sizeof(crc)))){
    return 0;
  }
  return log_block_size;
}

static u8 open_file(){
  char fname[] = "log.dat";

  while(1){
    u16 num;

#if CHECK_INCREMENT_LOG_DAT
    if(f_open(&file, "LOG.INC", (FA_OPEN_EXISTING | FA_WRITE)) == FR_OK){
      log_file_suffix = (file.fsize % 1000);
      f_lseek(&file, file.fsize);
      f_write(&file, "*", 1, &num); // Add 1 byte to log.inc
      f_close(&file);
    }
#endif

    if(log_file_suffix >= 0){
      u8 i;
      num = log_file_suffix;
      for(i = 6; i >= 4; i--){
        fname[i] = '0' + (num % 10);
        num /= 10;
      } // Generate file name such as log.000
    }

    if(f_open(&file, fname, (FA_OPEN_ALWAYS | FA_WRITE)) != FR_OK){
      return FALSE;
    }

    // file size check; up to bytes defined by MAXIMUM_LOG_DAT_FILE_SIZE
    if(file.fsize < MAXIMUM_LOG_DAT_FILE_SIZE){break;}
    f_close(&file);
    log_file_suffix++;
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
        if(f_mount(0, &fs) != FR_OK){break;}
        if(!open_file()){break;}
        log_file_opened = TRUE;
        log_block_size = BUFFER_SIZE / 2;
        free_page = locked_page = payload_buf;
        return;
      }
      log_func = log_to_file;
      break;
    case USB_CDC_ACTIVE:
      if(log_block_size != SYLPHIDE_PAGESIZE){
        log_block_size = SYLPHIDE_PAGESIZE;
        free_page = locked_page = payload_buf;
        return;
      }
      log_func = log_to_host;
      { // TODO provisional; CDC RX will be used for debug purpose, and currently thrown away.
        u16 read_count;
        u8 buf[8];
        while(read_count = cdc_rx(buf, sizeof(buf)));
      }
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
