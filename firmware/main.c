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

#include <stdio.h>
#include <time.h>

#include "main.h"
#include "util.h"
#include "config.h"

#include "c8051f380.h"
#include "f38x_usb.h"
#include "f38x_uart0.h"
#include "f38x_uart1.h"
#include "f38x_i2c0.h"
#include "f38x_spi.h"

#include "data_hub.h"

#include "gps.h"
#include "telemeter.h"
#if defined(NINJA_VER) && (NINJA_VER >= 200)
#include "mpu9250.h"
#else
#include "mpu6000.h"
#include "mag3110.h"
#endif
#include "ms5611.h"

volatile __xdata u32 global_ms = 0;
volatile __xdata u32 tickcount = 0;
volatile __xdata u8 sys_state = 0;
volatile u8 timeout_10ms = 0;

__xdata void (*main_loop_prologue)() = NULL;

static void sysclk_init();
static void port_init();
static void timer_init();

typedef struct {
  long delay_sec;
} software_reset_survive_t;

static __idata __at (0x100 - sizeof(software_reset_survive_t))
    software_reset_survive_t software_reset_survive;

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

#define software_reset() {RSTSRC = 0x10;} // RSTSRC.4(SWRSF) = 1 causes software reset

static void power_on_delay_check(FIL *f){
  // extract standby time[s] from file
  software_reset_survive.delay_sec = data_hub_read_long(f);
}

static void power_on_delay(){
  if((REG01CN & 0x40) || (RSTSRC & 0x02) || (!(RSTSRC & 0x10))
      || (software_reset_survive.delay_sec <= 0)){
    // Skip either when USB is connected, software reset is not invoked, or no delay
    software_reset_survive.delay_sec = 0;
    return;
  }

  /* How to standby with minimum power consumption
   * 1-1. Set all pins are configured as Hi-Z (open-drain and H) (except for P0.4, P2.2, P2.3).
   * 1-2. Sleep GPS
   * 1-3. Shutdown LTC3550 buck regulator by pulling P2.1 to L (Effective only for ver.2)
   * 1-4. Enable the low frequency internal oscillator
   * 2-1. Selecting the low frequency oscillator
   * 2-2. Disable unused peripherals including the high frequency oscillator
   * 2-3. Standby for the specified time
   * 3-1. Wake GPS up
   * 3-2. Perform software reset.
   */

  // step 1
  // P1MDOUT = P3MDOUT = 0; // default
  P0MDOUT = 0x10;
  P2MDOUT = (0x04 | 0x08);
  // P0 = P1 = P3 = 0xFF; // default
  P2 = ~(0x04 | 0x08);
  XBR0 = 0x01;  // Enable UART0
  XBR1 = 0xC0;  // Enable crossbar & Disable weak pull-up

  uart0_init_boot();
  gps_sleep();

  p21_low();

  OSCLCN = 0x82; // enable low frequency OSC with 40KHz output config
  while(!(OSCLCN & 0x40)); // wait for ready

  // step 2
  //TMR3CN = 0x00;    // Stop Timer3; Clear TF3; default
  CKCON |= 0xC0;   // Timer3 clocked based on SYSCLK;
  TMR3RL = (0x10000 - 40000);  // Re-initialize reload value (40KHz, 1s)
  TMR3 = 0xFFFF;    // Set to reload immediately
  CLKSEL = (CLKSEL & ~0x07) | 0x04; // Select L-F oscillator

  OSCICN &= ~0x80; // Disable H-F oscillator

  TMR3CN |= 0x04;   // Start Timer3(TR3)
  do{
    if(software_reset_survive.delay_sec & 0x0F){led34_off();}else{led34_on();}
    while(!(TMR3CN & 0x80));
    TMR3CN &=~0x80;
  }while((--software_reset_survive.delay_sec) > 0);

  // step 3
  gps_wakeup();
  software_reset();
}

#if defined(POSITION_MONITOR)
static __xdata u8 position_polarity = 0;
typedef struct {
  gps_pos_t upper, lower;
} position_range_t;
static volatile __code __at(CONFIG_ADDRESS + sizeof(config_t))
    position_range_t position_range = {
  {{ // position upper
    1800000000, 900000000, 100000000, // E180, N90, 100km
  }},
  {{ // position lower
    -1800000000, -900000000, -100000000, // W180, S90, -100km
  }},
};
static void position_monitor(__xdata gps_pos_t *pos){
  typedef enum {
    GPS_SAT_0 = 0,
    GPS_SAT_1 = 1,
    GPS_SAT_2 = 2,
    GPS_SAT_3 = 3,
    GPS_OUT_OF_RANGE,
    GPS_IN_RANGE,
  } gps_state_t;
  static __xdata gps_state_t periodic = GPS_SAT_0;
  static __xdata u16 periodic_count = 0;
  gps_state_t current = GPS_OUT_OF_RANGE;
  switch(gps_fix_type){
    case GPS_FIX_3D:
    case GPS_FIX_DEAD_RECKONING_COMBINED:
      if(gps_pos_accuracy >= GPS_POS_ACC_100M){
        u8 i, j;
        current = GPS_IN_RANGE;
        for(i = 0, j = 0x01; i < 3; ++i, j <<= 1){
          u8
              a = (position_range.upper.v[i] < pos->v[i]),
              b = (position_range.lower.v[i] > pos->v[i]);
          if((position_polarity & j) ? (a || b) : (a && b)){
            current = GPS_OUT_OF_RANGE;
            break;
          }
        }
      }
      break;
    default:
      if(gps_num_of_sat < 4){
        current = (gps_state_t)gps_num_of_sat;
      }
  }
  if(periodic < current){periodic = current;}
  if(usb_mode != USB_INACTIVE){
    periodic_count = 0;
    return;
  }
  ++periodic_count;
  if(periodic <= GPS_SAT_3){
    if(periodic_count >= 0x800){ // Timeout 409.6 [s] @ 5Hz
      software_reset_survive.delay_sec = 900; // Sleep 15 min.
      software_reset();
    }
  }else if(periodic <= GPS_OUT_OF_RANGE){
    if(periodic_count >= 0x100){ // Timeout 51.2 [s] @ 5Hz
      software_reset_survive.delay_sec = 120; // Sleep 2 min.
      software_reset();
    }
  }else{
    if(periodic_count >= 0x100){
      periodic = GPS_SAT_0;
      periodic_count = 0;
    }
  }
}
static void position_check(FIL *f){
  if(f_size(f)){ // check new configuration
    // file format must be "lng_upper lng_lower lat_upper lat_lower alt_upper alt_lower"
    DWORD f_pos = f_tell(f);
    __xdata config_t *buf = config_clone();
    __xdata position_range_t *buf_range = (__xdata position_range_t *)((u8 *)buf + sizeof(config_t));
    u8 i;
    for(i = 0; i < 3; ++i){
      buf_range->upper.v[i] = data_hub_read_long(f);
      buf_range->lower.v[i] = data_hub_read_long(f);
      if(f_tell(f) == f_pos){break;}
      f_pos = f_tell(f);
    }
    if((i == 3) && (memcmp(&position_range, buf_range, sizeof(position_range_t)) != 0)){
      config_renew(buf);
    }
  }
  {
    u8 i, j;
    for(i = 0, j = 0x01; i < 3; ++i, j <<= 1){
      if(position_range.upper.v[i] >= position_range.lower.v[i]){
        position_polarity |= j;
      }
    }
  }
  gps_position_monitor = position_monitor;
}
#endif

void main() {
  sysclk_init(); // Initialize oscillator
  wait_ms(1000);
  port_init(); // Initialize crossbar and GPIO

  spi_init();
  data_hub_init();

  if(!(REG01CN & 0x40)){
    // When USB is disconnected
    if(RSTSRC & 0x02){
      // When initial power on, which causes power on reset (RSTSRC.1(PORSF) = 1).
      data_hub_load_config("DELAY.CFG", power_on_delay_check);
      if(software_reset_survive.delay_sec > 0){software_reset();}
    }
#if defined(POSITION_MONITOR)
    data_hub_load_config("POSITION.CFG", position_check);
#endif
  }

  timer_init();
  uart0_init();
  uart1_init();
  i2c0_init();

#if defined(NINJA_VER) && (NINJA_VER >= 200)
  mpu9250_init();
#else
  mpu6000_init();
  mag3110_init();
#endif
  ms5611_init();
  
  EA = 1; // Global Interrupt enable
  
  gps_init();
  telemeter_init();

  usb0_init();

  // Time Pulse Interrupt config (-INT0)
  IT0 = 1;    // Edge sense
  //PX0 = 1;    // Priority High
  EX0 = 1;    // Enable

  while(main_loop_prologue){
    main_loop_prologue();
  }

  while (1) {
    gps_polling();
    telemeter_polling();
#if defined(NINJA_VER) && (NINJA_VER >= 200)
    mpu9250_polling();
#else
    mpu6000_polling();
    mag3110_polling();
#endif
    ms5611_polling();
    data_hub_polling();
    usb_polling();

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
#if defined(NINJA_VER) && (NINJA_VER >= 200)
  mpu9250_capture = TRUE;
#else
  mpu6000_capture = TRUE;
#endif
  global_ms += 10;
  tickcount++;
  timeout_10ms++;
  switch(u32_lsbyte(tickcount) % 16){ // 6.25Hz
    case 0:
      break;
    case 4:
#if !(defined(NINJA_VER) && (NINJA_VER >= 200))
      mag3110_capture = TRUE;
#endif
      break;
    case 8:
      ms5611_capture = TRUE; // Read pressure
      break;
    case 12:
      ms5611_capture = TRUE; // Read temperature
      break;
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
          snapshot_gps = gps_num_of_sat;
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

void interrupt_int0() __interrupt(INTERRUPT_INT0) {
  if(gps_time_modified){
    gps_time_modified = FALSE;
    global_ms = gps_time.itow_ms;
  }
}

DWORD get_fattime(){
  static __xdata DWORD res = 0;
#if USE_GPS_STD_TIME
  struct tm *t;
  time_t timer;
  gps_std_time(&timer);

  if(timer == 0){return res;}
  t = localtime(&timer);
#define GET_TM_ITEM(x) t->x
#else
  if(!gps_utc_valid){return res;}
#define GET_TM_ITEM(x) gps_utc.x
#endif

  // bit31:25 => year - 1980
  res = GET_TM_ITEM(tm_year) + 1900 - 1980;
  res <<= 4;
  // bit24:21 => month[1..12]
  res |= GET_TM_ITEM(tm_mon) + 1;
  res <<= 5;
  // bit20:16 => day[1..31]
  res |= GET_TM_ITEM(tm_mday);
  res <<= 5;
  // bit15:11 => hour[0..23]
  res |= GET_TM_ITEM(tm_hour);
  res <<= 6;
  // bit10:5 => minute[0..59]
  res |= GET_TM_ITEM(tm_min);
  res <<= 5;
  // bit4:0 => second / 2
  res |= GET_TM_ITEM(tm_sec) / 2;

  return res;
}

unsigned char _sdcc_external_startup(){
  PCA0MD &= ~0x40; ///< Disable Watchdog timer
  power_on_delay();
  return 0;
}
