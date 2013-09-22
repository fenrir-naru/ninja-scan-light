#include "c8051f380.h"
#include "main.h"

#include <string.h>

#include "mpu6000.h"

#include "type.h"
#include "data_hub.h"

enum address_t {
  SELF_TEST_X = 0x0D,
  SELF_TEST_Y = 0x0E,
  SELF_TEST_Z = 0x0F,
  SELF_TEST_A = 0x10,
  SMPLRT_DIV = 0x19,
  CONFIG = 0x1A,
  GYRO_CONFIG = 0x1B,
  ACCEL_CONFIG = 0x1C,
  FIFO_EN = 23,
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
  USER_CTRL = 0x6A,
  PWR_MGMT_1 = 0x6B,
  PWR_MGMT_2 = 0x6C,
  FIFO_COUNTH = 0x72,
  FIFO_COUNTL = 0x73,
  FIFO_R_W = 0x74,
  WHO_AM_I = 0x75,
};

/*
 * MPU-6000
 *
 * === Connection ===
 * C8051           MPU-6000
 *  P1.4(OUT) => SCK
 *  P1.5(OUT) => MOSI
 *  P1.6(OUT) => -CS
 *  P1.7(IN)   <=  MISO
 */

#define clk_up()     (P1 |= 0x10)
#define clk_down()   (P1 &= ~0x10)
#define out_up()     (P1 |= 0x20)
#define out_down()   (P1 &= ~0x20)
#define cs_assert()   (P1 &= ~0x40)
#define cs_deassert()   (P1 |= 0x40)
#define is_in_up()   (P1 & 0x80)

#define _nop_() __asm__("nop")

static void mpu6000_write(u8 *buf, u8 size){
  for(; size--; buf++){
    u8 mask = 0x80;
    do{
      clk_down();
      ((*buf) & mask) ? out_up() : out_down();
      clk_up();
    }while(mask >>= 1);
  }
}

static void mpu6000_read(u8 *buf, u8 size){
  for(; size--; buf++){
    u8 temp = 0;
    u8 mask = 0x80;
    out_down();
    do{
      clk_down();
      _nop_();
      if(is_in_up()) temp |= mask;
      clk_up();
    }while(mask >>= 1);
    *buf = temp;
  }
}

#define mpu6000_set(address, value) { \
  static const u8 addr_value[] = {address, value}; \
  cs_assert(); \
  mpu6000_write(addr_value, sizeof(addr_value)); \
  cs_deassert(); \
  _nop_(); \
}

#define mpu6000_get(address, value) { \
  static const u8 addr[] = {0x80 | address}; \
  cs_assert(); \
  mpu6000_write(addr, sizeof(addr)); \
  mpu6000_read((u8 *)&(value), sizeof(value)); \
  cs_deassert(); \
  _nop_(); \
}

volatile __bit mpu6000_capture;

void mpu6000_init(){
  mpu6000_set(USER_CTRL, 0x34); // Enable Master I2C, disable primary I2C I/F, and reset FIFO.
  mpu6000_set(SMPLRT_DIV, 79); // SMPLRT_DIV = 79, 100Hz sampling;
  // CONFIG = 0; // Disable FSYNC, No DLPF
  mpu6000_set(GYRO_CONFIG, (3 << 3)); // FS_SEL = 3 (2000dps)
  mpu6000_set(ACCEL_CONFIG, (2 << 3)); // AFS_SEL = 2 (8G)
  mpu6000_set(FIFO_EN, 0xF8); // FIFO enabled for temperature(2), gyro(2 * 3), accelerometer(2 * 3). Total 14 bytes.
  mpu6000_set(I2C_MST_CTRL, (0xC8 | 13)); // Multi-master, Wait for external sensor, I2C stop then start cond., clk 400KHz
  mpu6000_set(USER_CTRL, 0x70); // Enable FIFO with Master I2C enabled, and primary I2C I/F disabled.
  mpu6000_capture = FALSE;
}

static void make_packet(packet_t *packet){
  
  payload_t *dst = packet->current, *dst_end = packet->buf_end;
  
  // packetを登録するのに十分なサイズがあるか確認
  if((dst_end - dst) < PAGE_SIZE){
    return;
  }
  
  *(dst++) = 'A';
  *(dst++) = u32_lsbyte(tickcount);
  
  // 時刻の登録、LSB first
  //*((u32 *)(packet->current)) = global_ms;
  memcpy(dst, &global_ms, sizeof(global_ms));
  dst += sizeof(global_ms);
  
  memset(dst, 0, dst_end - dst);
  
  // 値の取得
  {
    // FIFOから、accel, temp, gyroの順
    u8 buf[14], i;
    __data u8 *_buf;
    mpu6000_get(FIFO_R_W, buf);
    
    /* 
     * In the following, take care of 2’s complement value.
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

void mpu6000_polling(){
  if(mpu6000_capture){
    
    // データがあるか確認  
    WORD_t v;
    mpu6000_get(FIFO_COUNTH, v.c[1]);
    mpu6000_get(FIFO_COUNTL, v.c[0]);
    if(v.i < 14){return;}
    
    mpu6000_capture = FALSE;
    data_hub_assign_page(make_packet);
    
    // FIFOリセット
    if(v.i > 14){
      mpu6000_set(USER_CTRL, 0x34);
      mpu6000_set(USER_CTRL, 0x70);
    }
  }
}
