#ifndef __UTIL_H__
#define __UTIL_H__

#include "main.h"

void wait_10n4clk(unsigned char i);
void wait_us(unsigned int count);
void wait_ms(unsigned int count);

#ifdef ENDIAN_CORRECT_BY_FUNC
u32 swap_u32(u32 dw);
u16 swap_u16(u16 w);
#else
#define swap_u32(dw) \
( (u32)(((u32)dw & 0x000000FF) << 24) \
  | (((u32)dw & 0x0000FF00) << 8) \
  | (((u32)dw & 0x00FF0000) >> 8) \
  | (((u32)dw & 0xFF000000) >> 24) \
)
#define swap_u16(w) \
( (u16)(((u16)w & 0x00FF) << 8) \
  | (((u16)w & 0xFF00) >> 8) \
)
#endif


#ifdef __SDCC__
#define le_u32(dw) (dw)
#define le_u16(w) (w)
#define be_u32(dw) swap_u32(dw)
#define be_u16(w) swap_u16(w)
#else
#define le_u32(dw) swap_u32(dw)
#define le_u16(w) swap_u16(w)
#define be_u32(dw) (dw)
#define be_u16(w) (w)

#endif

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

u16 crc16(u8 *buf, u8 size, u16 crc);

#endif /* __UTIL_H__ */

