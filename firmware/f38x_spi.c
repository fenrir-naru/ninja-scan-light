// SFR declarations
#include "c8051F380.h"

#include "f38x_spi.h"

u8 spi_ckr(u8 new_value){
  u8 old_value = SPI0CKR;
  SPI0CKR = new_value;
  return old_value;
}

/**
 * Configure SPI0 for 8-bit, 400KHz SCK, Master mode, polled operation, data
 * sampled on 1st SCK rising edge.
 * 
 */
void spi_init(){
  /*
   * data sampled on rising edge, clk active low,
   * 8-bit data words, master mode;
   */
  SPI0CFG = 0x70;
  
  // 4-wire mode; SPI enabled; flags cleared
  SPI0CN = 0x0F;
  
  // SPI clock = 400K
  spi_clock(400);
}

void spi_send_8clock(){
  SPI0DAT = 0xFF;
  while(!SPIF);
  SPIF = 0;
}

/**
 * Function sends one byte to spi and reads ony byte from spi
 * it will be written with SYSCLK as clk
 * 
 * @param byte value to write
 * @return SPI byte
 */
unsigned char spi_write_read_byte(unsigned char byte){
  SPI0DAT = byte;
  while(!SPIF);                      
  SPIF = 0;
  return SPI0DAT;
}

void spi_read(unsigned char * pchar, unsigned int length){
  while(length--){
    SPI0DAT = 0xFF;
    while(!SPIF);
    SPIF = 0;
    *(pchar++) = SPI0DAT;
  }
}

void spi_write(unsigned char * pchar, unsigned int length){
  while(length--){
    SPI0DAT = *(pchar++);
    while(!SPIF);
    SPIF = 0;
  }
}
