/*
 * Copyright (c) 2015, M.Naruoka (fenrir)
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

#include <string.h>

#include "f38x_flash.h"
#include "c8051f380.h"
#include "main.h"
#include "util.h"

typedef struct {
  u8 ea, flscl, vdm0cn, rstsrc;
} backup_t;

/*
 * @see http://community.silabs.com/t5/8-bit-MCU-Knowledge-Base/FLASH-Corruption/ta-p/110449
 * @see http://community.silabs.com/t5/8-bit-MCU-Knowledge-Base/All-about-the-VDD-Monitor/ta-p/110805
 */

#if 1
static void prologue(backup_t *res){
  backup_t orig;

  orig.ea = EA; // Preserve EA
  EA = 0; // Disable interrupts; Need more EA = 0; ? @see 16. Interrupts

  orig.flscl = FLSCL & 0x90;
  if(SYSCLK > 25000000UL){ // check clock speed
    FLSCL = (orig.flscl | 0x10);
  }

  orig.vdm0cn = VDM0CN;
  if(!(VDM0CN & 0x80)){
    VDM0CN = (orig.vdm0cn | 0x80); // Enable VDD monitor
    wait_us(100);
  }

  orig.rstsrc = RSTSRC & 0xA6;
  RSTSRC = (orig.rstsrc | 0x02); // Enable VDD monitor as a reset source; do not use a bit-wise OP

  memcpy(res, &orig, sizeof(backup_t));
}
#else
#define prologue(res){ \
\
  (res)->ea = EA; /* Preserve EA */ \
  EA = 0; /* Disable interrupts */ \
\
  (res)->flscl = FLSCL & 0x90; \
  if(SYSCLK > 25000000UL){ /* check clock speed */ \
    FLSCL = ((res)->flscl | 0x10); \
  } \
\
  (res)->vdm0cn = VDM0CN; \
  if(!(VDM0CN & 0x80)){ \
    VDM0CN = ((res)->vdm0cn | 0x80); /* Enable VDD monitor */ \
    wait_us(100); \
  } \
\
  (res)->rstsrc = RSTSRC & 0xA6; \
  RSTSRC = ((res)->rstsrc | 0x02); /* Enable VDD monitor as a reset source; do not use a bit-wise OP */ \
}
#endif

#if 0
static void epilogue(const backup_t *in){
  RSTSRC = in->rstsrc;
  VDM0CN = in->vdm0cn;
  FLSCL = in->flscl;
  EA = in->ea ? 1 : 0; /* Restore interrupts */
}
#else
#define epilogue(in) { \
  RSTSRC = (in)->rstsrc; \
  VDM0CN = (in)->vdm0cn; \
  FLSCL = (in)->flscl; \
  EA = (in)->ea ? 1 : 0; /* Restore interrupts */ \
}
#endif

#define set_keys() { \
  FLKEY = 0xA5; \
  FLKEY = 0xF1; \
}

#define write_byte(addr, byte) { \
  *((u8 __xdata *)addr) = byte; \
}

#define read_byte(addr) *((u8 __code *)(addr))

static u8 check_erased(flash_address_t addr, u16 size){
  u8 already_erased = TRUE;
  while(size--){
    if(read_byte(addr) != 0xFF){
      already_erased = FALSE;
      break;
    }
    addr++;
  }
  return already_erased;
}

void flash_erase_page(flash_address_t addr){
  backup_t orig;
#if 1
  addr &= ~((flash_address_t)FLASH_PAGESIZE - 1);
  if(check_erased(addr, FLASH_PAGESIZE)){return;}
#endif

  prologue(&orig);

  set_keys();
  PSCTL |= 0x03; // PSWE = 1, PSEE = 1
  write_byte(addr, 0);
  PSCTL &= ~0x03; // PSWE = 0, PSEE = 0

  epilogue(&orig);
}

static u16 write_zeros(flash_address_t dst, u8 *src, u16 size){
  u16 size_orig = size;
  u8 b[2];
  if(size > 0){
    backup_t orig;
    prologue(&orig);
#if 0 // smaller ROM required, however slower
    do{
      b[0] = *(src++);
      set_keys();
      PSCTL |= 0x01; // PSWE = 1
      write_byte(dst, b[0]);
      PSCTL &= ~0x01; // PSWE = 0
      dst++;
    }while(--size);
#else // faster, however require larger ROM
    if(dst & 0x1){ // odd
      b[0] = *(src++);
      PSCTL |= 0x01; // PSWE = 1
      set_keys();
      write_byte(dst, b[0]);
      PSCTL &= ~0x01; // PSWE = 0
      dst++;
      size--;
    }
    PFE0CN |= 0x01;
    while(size >= 2){
      u16 dst_odd = dst + 1;
      b[0] = *(src++);
      b[1] = *(src++);
      PSCTL |= 0x01; // PSWE = 1
      set_keys();
      write_byte(dst, b[0]);
      set_keys();
      write_byte(dst_odd, b[1]);
      PSCTL &= ~0x01; // PSWE = 0
      dst = dst_odd + 1;
      size -= 2;
    }
    PFE0CN &= ~0x01;
    if(size > 0){
      b[0] = *src;
      PSCTL |= 0x01; // PSWE = 1
      set_keys();
      write_byte(dst, b[0]);
      PSCTL &= ~0x01; // PSWE = 0
    }
#endif
    epilogue(&orig);
  }
  return size_orig;
}

u16 flash_renew_page(flash_address_t dst, u8 *src, u16 size){
  flash_address_t dst_align = dst & ~((flash_address_t)FLASH_PAGESIZE - 1);
  if(((flash_address_t)src) + size > (dst_align + FLASH_PAGESIZE)){ // maximum size is limited up to FLASH_PAGESIZE.
    size = (dst_align + FLASH_PAGESIZE) - ((flash_address_t)src);
  }
  flash_erase_page(dst_align);
  write_zeros(dst, src, size);

  return size;
}
