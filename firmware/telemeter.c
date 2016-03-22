/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
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

#include <ctype.h>
#include <stdlib.h>

#include "telemeter.h"
#include "config.h"
#include "f38x_uart1.h"
#include "util.h"
#include "ff.h"
#include "main.h"

static __bit telemeter_ready = 0;

void telemeter_send(char buf[SYLPHIDE_PAGESIZE]){
  static __xdata u16 sequence_num = 0;
  u16 crc;
  if((!telemeter_ready)
      || (uart1_tx_margin() < (
        sizeof(sylphide_protocol_header) + sizeof(sequence_num)
          + SYLPHIDE_PAGESIZE + sizeof(crc)))){
    return;
  }
  crc = crc16(buf, SYLPHIDE_PAGESIZE,
      crc16((u8 *)&(++sequence_num), sizeof(sequence_num), 0));
  uart1_write(sylphide_protocol_header, sizeof(sylphide_protocol_header));
  uart1_write((u8 *)&sequence_num, sizeof(sequence_num));
  uart1_write(buf, SYLPHIDE_PAGESIZE);
  uart1_write((u8 *)&crc, sizeof(crc));
}

static void expect(FIL *file){
  enum {
    INIT,
    BEFORE_SEND,
    SENDING,
    SENDING_ESCAPE,
    SENDING_ESCAPE_HEX1,
    SENDING_ESCAPE_HEX2,
    BEFORE_EXPECT,
    EXPECTING,
    EXPECT_FAILED,
    EXPECT_TIMEOUT,
    BEFORE_WAIT,
    WAITING,
    IGNORE_LINE,
  } state = INIT;

  char c;
  char buf[8];
  u8 buf_length, expect_index;
  u16 read_size;

  while(uart1_read(buf, sizeof(buf)) > 0);

  telemeter_ready = 0;

  while((f_read(file, &c, sizeof(c), &read_size) == FR_OK)
      && (read_size > 0)){
    unsigned char is_endline = ((c == '\r') || (c == '\n'));
    switch(state){
      case INIT:
        if(c == '$'){
          state = BEFORE_SEND;
        }else if(c == '>'){
          state = BEFORE_EXPECT;
          buf_length = expect_index = 0;
          timeout_10ms = 0;
          do{
            while(!uart1_read(&c, sizeof(c))){
              wait_us(10);
              if(timeout_10ms > 100){
                state = EXPECT_TIMEOUT;
                return;
              }
            }
            if(c == '\r' || c == '\n'){
              if(buf_length > 0){break;}
            }else if(buf_length < sizeof(buf)){
              buf[buf_length++] = c;
            }
          }while(1);
        }else if(c == '@'){
          state = BEFORE_WAIT;
        }else if(!isspace(c)){
          state = IGNORE_LINE;
        }
        break;
      case BEFORE_SEND:
        if(is_endline){
          state = INIT;
          break;
        }else if(isspace(c)){break;}
        state = SENDING;
      case SENDING:
        if(is_endline){
          state = INIT;
        }else if(c == '\\'){ // escape char
          state = SENDING_ESCAPE;
        }else{
          uart1_write(&c, sizeof(c));
        }
        break;
      case SENDING_ESCAPE:
        if(c == 'x'){
          state = SENDING_ESCAPE_HEX1;
          break;
        }
        switch(c){
          case 'r': c = '\r'; break;
          case 'n': c = '\n'; break;
          case '\\': c = '\\'; break;
        }
        uart1_write(&c, sizeof(c));
        state = SENDING;
        break;
      case SENDING_ESCAPE_HEX1:
        buf[0] = ((c & 0x0F) + (c >= 'A' ? 10 : 0)) << 4;
        state = SENDING_ESCAPE_HEX2;
        break;
      case SENDING_ESCAPE_HEX2:
        buf[0] += ((c & 0x0F) + (c >= 'A' ? 10 : 0));
        uart1_write(buf, 1);
        state = SENDING;
        break;
      case BEFORE_EXPECT:
        if(is_endline){ // match any
          state = INIT;
          break;
        }else if(isspace(c)){break;}
        state = EXPECTING;
      case EXPECTING:
        if(is_endline){
          state = INIT;
        }else if(expect_index < buf_length){
          if(c != buf[expect_index++]){
            state = EXPECT_FAILED;
            return;
          }
        }
        break;
      case BEFORE_WAIT:
        if(is_endline){
          state = INIT;
          break;
        }else if(isspace(c)){break;}
        buf_length = 0;
        state = WAITING;
      case WAITING:
        if(is_endline){
          buf[buf_length] = '\0';
          wait_ms((unsigned int)atol(buf));
          state = INIT;
        }else if(buf_length < (sizeof(buf) - 1)){
          buf[buf_length++] = c;
        }
        break;
      case IGNORE_LINE:
        if(is_endline){state = INIT;}
        break;
    }
  }

  telemeter_ready = 1;
}

void telemeter_init(){
  uart1_bauding(config.baudrate.telemeter);

  telemeter_ready = 1;
  data_hub_load_config("TLM.EXP", expect);
  data_hub_send_config("TLM.CFG", uart1_write);
}

static void make_packet(packet_t *packet){
  payload_t *dst = packet->current;
  *(dst++) = 'C';

  // read data and store it into packet
  uart1_read(dst, packet->buf_end - dst);
}

void telemeter_polling(){
  u8 buf_size = uart1_rx_size();
  for(; buf_size >= (SYLPHIDE_PAGESIZE - 1); buf_size -= (SYLPHIDE_PAGESIZE - 1)){
    if(!data_hub_assign_page(make_packet)){break;}
  }
}
