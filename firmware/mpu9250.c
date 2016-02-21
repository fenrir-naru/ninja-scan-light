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
#include "c8051f380.h"
#include "main.h"

#include <string.h>

#include "mpu9250.h"

#include "util.h"
#include "type.h"
#include "data_hub.h"

#define cs_wait() wait_8n6clk(50)
#define clk_wait() wait_8n6clk(5)

typedef enum {
  SELF_TEST_X_GYRO = 0x00,
  SELF_TEST_Y_GYRO = 0x01,
  SELF_TEST_Z_GYRO = 0x02,
  SELF_TEST_X_ACCEL = 0x0D,
  SELF_TEST_Y_ACCEL = 0x0E,
  SELF_TEST_Z_ACCEL = 0x0F,
  XG_OFFSET_H = 0x13,
  XG_OFFSET_L = 0x14,
  YG_OFFSET_H = 0x15,
  YG_OFFSET_L = 0x16,
  ZG_OFFSET_H = 0x17,
  ZG_OFFSET_L = 0x18,
  SMPLRT_DIV = 0x19,
  CONFIG = 0x1A,
  GYRO_CONFIG = 0x1B,
  ACCEL_CONFIG = 0x1C,
  ACCEL_CONFIG2 = 0x1D,
  LP_ACCEL_ODR = 0x1E,
  WOM_THR = 0x1F,
  FIFO_EN = 0x23,
  I2C_MST_CTRL = 0x24,
  I2C_SLV0_ADDR = 0x25,
  I2C_SLV0_REG = 0x26,
  I2C_SLV0_CTRL = 0x27,
  I2C_SLV1_ADDR = 0x28,
  I2C_SLV1_REG = 0x29,
  I2C_SLV1_CTRL = 0x2A,
  I2C_SLV2_ADDR = 0x2B,
  I2C_SLV2_REG = 0x2C,
  I2C_SLV2_CTRL = 0x2D,
  I2C_SLV3_ADDR = 0x2E,
  I2C_SLV3_REG = 0x2F,
  I2C_SLV3_CTRL = 0x30,
  I2C_SLV4_ADDR = 0x31,
  I2C_SLV4_REG = 0x32,
  I2C_SLV4_DO = 0x33,
  I2C_SLV4_CTRL = 0x34,
  I2C_SLV4_DI = 0x35,
  I2C_MST_STATUS = 0x36,
  INT_PIN_CFG = 0x37,
  INT_ENABLE = 0x38,
  INT_STATUS = 0x3A,
  ACCEL_OUT_BASE = 0x3B,
  TEMP_OUT_BASE = 0x41,
  GYRO_OUT_BASE = 0x43,
  EXT_SENS_DATA_BASE = 0x49,
  I2C_SLV0_DO = 0x63,
  I2C_SLV1_DO = 0x64,
  I2C_SLV2_DO = 0x65,
  I2C_SLV3_DO = 0x66,
  I2C_MST_DELAY_CTRL = 0x67,
  SIGNAL_PATH_RESET = 0x68,
  MOT_DETECT_CTRL = 0x69,
  USER_CTRL = 0x6A,
  PWR_MGMT_1 = 0x6B,
  PWR_MGMT_2 = 0x6C,
  FIFO_COUNTH = 0x72,
  FIFO_COUNTL = 0x73,
  FIFO_R_W = 0x74,
  WHO_AM_I = 0x75,
  XA_OFFSET_H = 0x77,
  XA_OFFSET_L = 0x78,
  YA_OFFSET_H = 0x7A,
  YA_OFFSET_L = 0x7B,
  ZA_OFFSET_H = 0x7D,
  ZA_OFFSET_L = 0x7E,
} address_t;

typedef enum {
  AK8963_WIA    = 0x00, // Device ID
  AK8963_INFO   = 0x01, // Information
  AK8963_ST1    = 0x02, // Status 1
  AK8963_HXL    = 0x03, // Measurement data
  AK8963_HXH    = 0x04,
  AK8963_HYL    = 0x05,
  AK8963_HYH    = 0x06,
  AK8963_HZL    = 0x07,
  AK8963_HZH    = 0x08,
  AK8963_ST2    = 0x09, // Status 2
  AK8963_CNTL1  = 0x0A, // Control 1
  AK8963_CNTL2  = 0x0B, // Control 2
  AK8963_RSV    = 0x0B, // Reserved
  AK8963_ASTC   = 0x0C, // Self-test
  AK8963_TS1    = 0x0D, // Test 1
  AK8963_TS2    = 0x0E, // Test 2
  AK8963_I2CDIS = 0x0F, // I2C disable
  AK8963_ASAX   = 0x10, // X-axis sensitivity adjustment value
  AK8963_ASAY   = 0x11, // Y-axis sensitivity adjustment value
  AK8963_ASAZ   = 0x12, // Z-axis sensitivity adjustment value
} ak8963_address_t;

#define AK8963_I2C_ADDR 0x0C

/*
 * MPU-9250
 *
 * === Connection ===
 * C8051         MPU-9250
 *  P1.4(OUT) =>  -CS
 *  P1.5(OUT) =>  SCK
 *  P1.6(OUT) =>  MOSI
 *  P1.7(IN)  <=  MISO
 */

#ifdef USE_ASM_FOR_SFR_MANIP
#define clk_up()      {__asm orl _P1,SHARP  0x20 __endasm; }
#define clk_down()    {__asm anl _P1,SHARP ~0x20 __endasm; }
#define out_up()      {__asm orl _P1,SHARP  0x40 __endasm; }
#define out_down()    {__asm anl _P1,SHARP ~0x40 __endasm; }
#define cs_assert()   {__asm anl _P1,SHARP ~0x10 __endasm; }
#define cs_deassert() {__asm orl _P1,SHARP  0x10 __endasm; }
#else
#define clk_up()      (P1 |=  0x20)
#define clk_down()    (P1 &= ~0x20)
#define out_up()      (P1 |=  0x40)
#define out_down()    (P1 &= ~0x40)
#define cs_assert()   (P1 &= ~0x10)
#define cs_deassert() (P1 |=  0x10)
#endif
#define is_in_up()    (P1 & 0x80)

static void mpu9250_write(u8 *buf, u8 size){
  for(; size--; buf++){
    u8 mask = 0x80;
    do{
      clk_down();
      if((*buf) & mask){out_up();}else{out_down();}
      clk_wait();
      clk_up();
      clk_wait();
    }while(mask >>= 1);
  }
}

static void mpu9250_read(u8 *buf, u8 size){
  for(; size--; buf++){
    u8 temp = 0;
    u8 mask = 0x80;
    do{
      clk_down();
      clk_wait();
      clk_up();
      if(is_in_up()) temp |= mask;
      clk_wait();
    }while(mask >>= 1);
    *buf = temp;
  }
}

#define mpu9250_set(address, value) { \
  static const __code u8 addr_value[2] = {address, value}; \
  cs_assert(); \
  cs_wait(); \
  mpu9250_write(addr_value, sizeof(addr_value)); \
  cs_deassert(); \
  cs_wait(); \
}

#define mpu9250_get2(address, vp, size) { \
  static const __code u8 addr[1] = {0x80 | address}; \
  cs_assert(); \
  cs_wait(); \
  mpu9250_write(addr, sizeof(addr)); \
  mpu9250_read(vp, size); \
  cs_deassert(); \
  cs_wait(); \
}

#define mpu9250_get(address, value) mpu9250_get2(address, (u8 *)&(value), sizeof(value))

volatile __bit mpu9250_capture = FALSE;

static __bit mpu9250_available = FALSE;
static __bit ak8963_available = FALSE;

void mpu9250_init(){
  u8 v;

  cs_deassert();
  clk_up();
  mpu9250_set(PWR_MGMT_1, 0x80); // Chip reset
  wait_ms(100);
  mpu9250_set(PWR_MGMT_1, 0x01); // Wake up device and select the best available clock
  
  mpu9250_set(USER_CTRL, 0x34); // Enable Master I2C, disable primary I2C I/F, and reset FIFO.
  mpu9250_set(SMPLRT_DIV, 9); // SMPLRT_DIV = 9, 100Hz sampling;
  mpu9250_set(CONFIG, (1 << 6) | (1 << 0)); // FIFO_mode = 1 (accept overflow), Use LPF, Bandwidth_gyro = 184 Hz, Bandwidth_temperature = 188 Hz,
  mpu9250_set(GYRO_CONFIG, (3 << 3)); // FS_SEL = 3 (2000dps)
  mpu9250_set(ACCEL_CONFIG, (2 << 3)); // AFS_SEL = 2 (8G)

  mpu9250_set(I2C_MST_CTRL, (0xC8 | 13)); // Multi-master, Wait for external sensor, I2C stop then start cond., clk 400KHz

  { // AK8963 setup
    mpu9250_set(I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80); // Set the I2C slave 4 address of AK8963 and set for read.

    mpu9250_set(I2C_SLV4_REG, AK8963_WIA); //I2C slave 4 register address from where to begin data transfer
    mpu9250_set(I2C_SLV4_CTRL, 0x80); // Enable I2C slave 4
    wait_ms(20);
    mpu9250_get(I2C_SLV4_DI, v); // expecting 0x48
    if(v == 0x48){
      ak8963_available = TRUE;
    }

    mpu9250_set(I2C_SLV4_ADDR, AK8963_I2C_ADDR); // Set the I2C slave 4 address of AK8963 and set for write.

    mpu9250_set(I2C_SLV4_REG, AK8963_CNTL2); //I2C slave 4 register address from where to begin data transfer
    mpu9250_set(I2C_SLV4_DO, 0x01); // Reset AK8963
    mpu9250_set(I2C_SLV4_CTRL, 0x80); // Enable I2C slave 4
    wait_ms(20);

    mpu9250_set(I2C_SLV4_REG, AK8963_CNTL1); //I2C slave 4 register address from where to begin data transfer
    mpu9250_set(I2C_SLV4_DO, 0x12); // Register value to continuous measurement mode 1 (8Hz) in 16bit
    mpu9250_set(I2C_SLV4_CTRL, 0x80); // Enable I2C slave 4
    wait_ms(20);

    //mpu9250_set(I2C_SLV4_CTRL, 0x0F); // sampling rate is 100 / (1 + 15) Hz

    mpu9250_set(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80); // Set the I2C slave 0 address of AK8963 and set for read.

    mpu9250_set(I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    mpu9250_set(I2C_SLV0_CTRL, 0x87); // Enable I2C and set 7 byte,
    // which makes the AK8963A unlatch the data registers for the next measurement by reading ST2 register (0x09).
    //mpu9250_set(I2C_MST_DELAY_CTRL, 0x81); // Delayed sampling is applied to Slave 0.
  }

  mpu9250_set(FIFO_EN, 0xF9); // FIFO enabled for temperature(2), gyro(2 * 3), accelerometer(2 * 3), slave 0(7, delayed sample). Total 21 bytes.
  mpu9250_set(USER_CTRL, 0x70); // Enable FIFO with Master I2C enabled, and primary I2C I/F disabled.

  mpu9250_get(WHO_AM_I, v); // expecting 0x71
  if(v == 0x71){
    mpu9250_available = TRUE;
  }
}

static void make_packet_inertial(packet_t *packet){
  
  payload_t *dst = packet->current, *dst_end = packet->buf_end;
  
  // Check whether buffer has sufficient margin
  if((dst_end - dst) < SYLPHIDE_PAGESIZE){
    return;
  }
  
  *(dst++) = 'A';
  *(dst++) = u32_lsbyte(tickcount);
  
  // Record time, LSB first
  //*((u32 *)(packet->current)) = global_ms;
  memcpy(dst, &global_ms, sizeof(global_ms));
  dst += sizeof(global_ms);
  
  memset(dst, 0, dst_end - dst);
  
  // Get values
  {
    // from FIFO, accelerometer, temperature, and gyro values are extracted.
    u8 buf[14], i;
    __data u8 *_buf;
    mpu9250_get(FIFO_R_W, buf);
    
    /* 
     * In the following, take care of 2fs complement value.
     * raw => modified:
     * -128 = 0b10000000 (128) => 0b00000000 (0)
     * -36  = 0b11011100 (220) => 0b01011100 (92)
     *  36  = 0b00100100 (36)  => 0b10100100 (164)
     *  127 = 0b01111111 (127) => 0b11111111 (255)
     */
    
    _buf = buf;
    // accel, big endian, advances packet->current by 3 * 3 = 9 bytes
    for(i = 0; i < 3; ++i, _buf += 2){
      *_buf ^= 0x80;
      memcpy(++dst, _buf, 2);
      dst += 2;
    }
    _buf += 2; // skip temperature
    // gyro, big endian, advances packet->current by 3 * 3 = 9 bytes
    for(i = 0; i < 3; ++i, _buf += 2){
      *_buf ^= 0x80;
      memcpy(++dst, _buf, 2);
      dst += 2;
    }
    // ch.7, ch.8 unused, advances packet->current by 3 * 2 = 6 bytes
    dst += 6;
    // temperature, little endian
    *(dst++) = buf[7];
    *(dst++) = buf[6] ^ 0x80;
  }
  
  packet->current = dst;
}

/*
 * M page design =>
 * 'M', 0, 0, tickcount & 0xFF, // + 4
 * global_ms(4 bytes), // + 8
 * mag_XYZ[0-3](little endian, 2 * 3 bytes) // + 32
 */
static __xdata u8 mag_data[SYLPHIDE_PAGESIZE - 8];

static void make_packet_mag(packet_t *packet){
  payload_t *dst = packet->current;

  // Check whether buffer has sufficient margin
  if((packet->buf_end - dst) < SYLPHIDE_PAGESIZE){
    return;
  }

  *(dst++) = 'M';
  *(dst++) = 0; // Little endian mode
  *(dst++) = 0;
  *(dst++) = u32_lsbyte(tickcount);

  // Record time, LSB first
  memcpy(dst, &global_ms, sizeof(global_ms));
  dst += sizeof(global_ms);

  memcpy(dst, mag_data, sizeof(mag_data));
  dst += sizeof(mag_data);

  packet->current = dst;
}

void mpu9250_polling(){
  if(!mpu9250_available){return;}
  if(mpu9250_capture){

    // Check data availability
    WORD_t fifo_count;
    mpu9250_get(FIFO_COUNTH, fifo_count.c[1]);
    mpu9250_get(FIFO_COUNTL, fifo_count.c[0]);

    if(fifo_count.i < 21){return;}
    
    mpu9250_capture = FALSE;
    data_hub_assign_page(make_packet_inertial);
    
    do{ // check AK8963 data
      u8 buf[7];
      static __xdata u8 cycle = 0;
      static __xdata u8 * __xdata next_buf = mag_data;

      mpu9250_get(FIFO_R_W, buf);
      if((++cycle) % 16){break;}

      memcpy(next_buf, buf, 6);
      next_buf += 6;

      // Rotate
      if(next_buf >= mag_data + sizeof(mag_data)){
        data_hub_assign_page(make_packet_mag);
        next_buf = mag_data;
      }
    }while(0);

    // Reset FIFO if size is greater than expected
    if(fifo_count.i > 21){
      mpu9250_set(USER_CTRL, 0x34);
      mpu9250_set(USER_CTRL, 0x70);
    }
  }
}
