#ifndef __MAIN_H__
#define __MAIN_H__

#include "type.h"

//#define _USB_LOW_SPEED_                      // Change this comment to make Full/Low speed

// SYSCLK frequency in Hz
#define SYSCLK    48000000UL

extern volatile __xdata u32 global_ms;
extern volatile __xdata u32 tickcount;

extern volatile __xdata u8 state;
#define STATE_PERIODIC_ACTIVE 0x01
#define STATE_POLLING_ACTIVE 0x02
#define STATE_LOG_ACTIVE 0x04

extern volatile u8 timeout_10ms;

// Define Endpoint Packet Sizes
#ifdef _USB_LOW_SPEED_
// This value can be 8,16,32,64 depending on device speed, see USB spec
#define PACKET_SIZE_EP0     0x40
#else
#define PACKET_SIZE_EP0     0x40
#endif /* _USB_LOW_SPEED_ */ 

// Can range 0 - 1024 depending on data and transfer type
#define PACKET_SIZE_EP1 0x0010
#define PACKET_SIZE_EP2 0x0040
#define PACKET_SIZE_EP3 0x0040

#define CRITICAL_GLOBAL(func) \
{ \
  EA = 0; \
  { \
    func; \
  } \
  EA = 1; \
}

#endif      /* __MAIN_H__ */
