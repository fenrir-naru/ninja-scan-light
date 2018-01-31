/*
 * Copyright (c) 2018, M.Naruoka (fenrir)
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
#include "util.h"

#include "c8051f380.h"
#include "f38x_usb.h"
#include "f38x_spi.h"

#include "diskio.h"
#include "usb_cdc.h"

volatile __xdata u32 global_ms = 0;
volatile __xdata u32 tickcount = 0;
volatile __xdata u8 sys_state = 0;
volatile u8 timeout_10ms = 0;

__xdata void (*main_loop_prologue)() = NULL;

static void sysclk_init();
static void port_init();
static void timer_init();

static __xdata int standby_sec = 0;

#ifdef USE_ASM_FOR_SFR_MANIP
#define p21_hiz()   {__asm orl _P2,SHARP  0x02 __endasm; }
#define p21_low()   {__asm anl _P2,SHARP ~0x02 __endasm; }
#define scl0_hiz()  {__asm orl _P1,SHARP  0x02 __endasm; }
#define scl0_low()  {__asm anl _P1,SHARP ~0x02 __endasm; }
#define led3_on()   {__asm orl _P2,SHARP  0x08 __endasm; }
#define led4_on()   {__asm orl _P2,SHARP  0x04 __endasm; }
#define led34_on()  {__asm orl _P2,SHARP  (0x04 | 0x08) __endasm; }
#define led34_off() {__asm anl _P2,SHARP ~(0x04 | 0x08) __endasm; }
#else
#define p21_hiz()   (P2 |=  0x02)
#define p21_low()   (P2 &= ~0x02)
#define scl0_hiz()  (P1 |=  0x02)
#define scl0_low()  (P1 &= ~0x02)
#define led3_on()   (P2 |=  0x08)
#define led4_on()   (P2 |=  0x04)
#define led34_on()  (P2 |=  (0x04 | 0x08))
#define led34_off() (P2 &= ~(0x04 | 0x08))
#endif

void main() {
  sysclk_init(); // Initialize oscillator
  wait_ms(1000);
  port_init(); // Initialize crossbar and GPIO

  spi_init();
  disk_initialize(0);
  cdc_force = TRUE;

  timer_init();
  
  EA = 1; // Global Interrupt enable
  
  usb0_init();

  // Time Pulse Interrupt config (-INT0)
  IT0 = 1;    // Edge sense
  //PX0 = 1;    // Priority High
  EX0 = 1;    // Enable

  while (1) {
    static BYTE __xdata cdc_buf[512];
    static DWORD sector_start = 0, sectors = 0;

    usb_polling();

    if(sectors > 0){ // return result
      u8 i = (sectors > 0x10) ? 0x10 : (u8)sectors;
      do{
        if(disk_read(0, cdc_buf, sector_start, 1) != RES_OK){
          sectors = 0;
          cdc_tx(cdc_buf, sprintf(cdc_buf, "READ_ERROR!(%lu)\n", sector_start));
          break;
        }
        sector_start++; sectors--;
        cdc_tx(cdc_buf, sizeof(cdc_buf));
      }while((--i) > 0);
    }

    if(sectors == 0){ // parse input
      u16 read_bytes = cdc_rx(cdc_buf, sizeof(cdc_buf) - 1);
      cdc_buf[read_bytes] = 0;
      while(read_bytes > 0){
        DWORD dw = 0;
        u8 hit = 0, count = 0;
        u16 i = 0;

        { // check command
          char *next;
          DWORD v = 0;
          if((next = strstr(cdc_buf, "count")) != 0){
            disk_ioctl(0, GET_SECTOR_COUNT, &v);
            cdc_tx(cdc_buf, sprintf(cdc_buf, "COUNT => %lu\n", v));
            break;
          }else if((next = strstr(cdc_buf, "size")) != 0){
            disk_ioctl(0, GET_SECTOR_SIZE, &v);
            cdc_tx(cdc_buf, sprintf(cdc_buf, "SIZE => %lu\n", v));
            break;
          }else if((next = strstr(cdc_buf, "read")) != 0){
            i += (next - &(cdc_buf[0]));
          }
        }

        // read setup
        sectors = 1;
        for(; i <= read_bytes; ++i){
          if((cdc_buf[i] >= '0') && (cdc_buf[i] <= '9')){
            dw *= 10;
            dw += (cdc_buf[i] - '0');
            hit++;
          }else if(hit > 0){
            count++;
            if(count == 1){
              sector_start = dw;
            }else if(count == 2){
              sectors = dw;
              break;
            }
            dw = 0;
            hit = 0;
          }
        }
        cdc_tx(cdc_buf, sprintf(cdc_buf, "READ(%lu,%lu) => \n", sector_start, sectors));
        break;
      }
    }

    sys_state |= SYS_POLLING_ACTIVE;
  }
}

// System clock selections (SFR CLKSEL)
#define SYS_INT_OSC              0x00        // Select to use internal oscillator
#define SYS_4X_MUL               0x03        // Select to use internal oscillator
#define SYS_EXT_OSC              0x01        // Select to use an external oscillator
#define SYS_4X_DIV_2             0x02

// USB clock selections (SFR CLKSEL)
#define USB_4X_CLOCK             0x00        // Select 4x clock multiplier, for USB Full Speed
#define USB_INT_OSC_DIV_2        0x10        // See Data Sheet section 13. Oscillators
#define USB_EXT_OSC              0x20
#define USB_EXT_OSC_DIV_2        0x30
#define USB_EXT_OSC_DIV_3        0x40
#define USB_EXT_OSC_DIV_4        0x50

static void sysclk_init(){
  REF0CN = 0x07;
  
  // Configure internal oscillator for its maximum frequency and enable missing clock detector
  OSCICN |= 0x03;

#ifdef _USB_LOW_SPEED_
  CLKSEL  = SYS_INT_OSC; // Select System clock
  CLKSEL |= USB_INT_OSC_DIV_2; // Select USB clock
#else
  // Select internal oscillator as input to clock multiplier
  CLKMUL  = 0x00;
  CLKMUL |= 0x80; // Enable clock multiplier
  wait_ms(1); // Delay for clock multiplier to begin
  CLKMUL |= 0xC0; // Initialize the clock multiplier
  wait_ms(1);
  while(!(CLKMUL & 0x20)); // Wait for multiplier to lock
  CLKSEL = SYS_4X_MUL;
  CLKSEL |= USB_4X_CLOCK; // Select USB clock
#endif  /* _USB_LOW_SPEED_ */
}


static void port_init() {
  
  // Default port state
  // Pn = 1 (High), PnMDIN = 1 (not analog), PnMDOUT = 0 (open-drain) => Hi-Z

  // check scl0(P1.1) and sda0(P1.0), if not Hi, perform clock out to reset I2C0.
  while((P1 & 0x03) != 0x03){
    scl0_low();
    while(P1 & 0x02);
    scl0_hiz();
    while(!(P1 & 0x02));
  }
  
  // P0
  // 0 => SPI_SCK, 1 => SPI_MISO, 2 => SPI_MOSI, 3 => SPI_-CS,
  // 4 => UART0_TX, 5 => UART0_RX, 6 => -INT0, 7 => VREF(analog)
  P0MDIN = 0x7F;
  P0MDOUT = 0x1D; // 1, 5, 6, 7 => open-drain
  P0 = 0x7F;
  P0SKIP = 0xC0;  // for -INT0 / VREF
  IT01CF = 0x76;  // -INT0 => 6 negative
  
  // P1
  // 0 => I2C0_SDA, 1 => I2C0_SCL, 2-3 => GPIO0-1,
  // 4 => IMU_SCK, 5 => IMU_MOSI, 6 => IMU_-CS, 7 => IMU_MISO
  P1MDIN = 0xFF;
  P1MDOUT = 0x7C; // 0, 1, 7 => open-drain
  P1 = 0xFF;
  P1SKIP = 0xFC;
  
  // P2
  // 0-1 => N.C., 2 => LED0, 3 => LED1
  // 4 => UART1_TX, 5 => UART1_RX, 6 => I2C1_SDA, 7 => I2C1_SCL
  P2MDIN = 0xFF; 
  P2MDOUT = 0x1C; // 0, 1, 5, 6, 7 => open-drain
  P2 = 0xF3;
  P2SKIP = 0x0F;
  
  // P3
  P3MDIN = 0xFF;
  P3MDOUT = 0xFF;
  P3 = 0x00;

  XBR0 = 0x07;  // UART0, SPI, I2C0 enabled
  XBR2 = 0x03;  // UART1, I2C1 enabled
  XBR1 = 0xC0;  // Enable crossbar & Disable weak pull-up
}

static void timer_init(){
  TMR3CN = 0x00;    // Stop Timer3; Clear TF3;
  CKCON &= ~0xC0;   // Timer3 clocked based on T3XCLK;
  TMR3RL = (0x10000 - (SYSCLK/12/100));  // Re-initialize reload value (100Hz, 10ms)
  TMR3 = 0xFFFF;    // Set to reload immediately
  EIE1 |= 0x80;     // Enable Timer3 interrupts(ET3)
  TMR3CN |= 0x04;   // Start Timer3(TR3)
}

/**
 * interrupt routine invoked when timer3 overflow
 * 
 */
void interrupt_timer3() __interrupt (INTERRUPT_TIMER3) {

  static u8 loop_50ms = 0;
  static u8 snapshot_gps = 0;
  static u8 snapshot_state = 0;
  static u8 loop_10s = 0;

  TMR3CN &= ~0x80; // Clear interrupt
  global_ms += 10;
  tickcount++;
  timeout_10ms++;
  switch(u32_lsbyte(tickcount) % 16){ // 6.25Hz
  }

  if(++loop_50ms < 5){return;}
  loop_50ms = 0;

  switch(++loop_10s % 5){
    case 0:
      if(loop_10s % 8 == 0){ // 50 * 5 * 8 = 2000 [ms]
        snapshot_state = (SYS_PERIODIC_ACTIVE | sys_state);
        sys_state = 0;
        if(loop_10s >= 200){ // 50 * 200 = 10000 [ms]
          loop_10s = 0;
          snapshot_gps = 0;
        }
      }
      if(snapshot_gps > 0){
        led4_on();
        snapshot_gps--;
      }
      if(snapshot_state & 0x01){
        led3_on();
      }
      break;
    case 1:
      led34_off(); // LED3 / 4 off
      snapshot_state >>= 1;
      break;
  }
}

DWORD get_fattime(){
  return 0;
}

unsigned char _sdcc_external_startup(){
  PCA0MD &= ~0x40; ///< Disable Watchdog timer
  return 0;
}
