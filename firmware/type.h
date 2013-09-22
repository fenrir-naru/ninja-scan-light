#ifndef __TYPE_H__
#define __TYPE_H__

typedef unsigned char u8;
typedef signed char s8;
typedef unsigned short u16;
typedef signed short s16;
typedef unsigned long u32;
typedef signed long s32;

typedef unsigned char UCHAR;
typedef unsigned int UINT;
typedef unsigned long ULONG;

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef u8 BYTE;
typedef u16 WORD;
typedef u32 DWORD;

typedef union {u16 i; u8 c[2];} WORD_t;
typedef union {u32 i; u8 c[4];} DWORD_t;

#define LSB 0
#define MSB 1

#if !(defined(__SDCC) || defined(SDCC))
#define __bit
#define __data
#define __xdata
#define __code
#define __interrupt(x)
#define u32_lsbyte(x) ((x) & 0xFF)
#else
#define u32_lsbyte(x) (((DWORD_t *)&(x))->c[0]) // Little Endian
#endif

#endif /* __TYPE_H__ */
