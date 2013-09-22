#include "c8051f380.h"    // SFR declarations

#include "f38x_i2c0.h"
#include "type.h"

/**
 * Configure i2c0 with Timer 0
 * 
 */
void i2c0_init(){
  // timer 0 setup => mode 2, 8bit w autoreload
  TMOD &= ~0x0F;
  TMOD |=  0x02;
  
  CKCON |= 0x04; // => use SYSCLK
  TH0 = TL0 = (0x100 - 40); // => freq: 48M / 40 = 1.2M
  //TH0 = TL0 = 0x00; // => freq: 48M / 256 
  TR0 = 1;
  
  // inhibit slave, extend setup / hold time, use timer 0 overflow
  SMB0CF = 0x40; // => 1.2M / 3clk = 400KHz (Eq. 22.2)
  //SMB0CF = (0x40 | 0x10); // => 1.2M / 12+11clk = 50KHz
  
  SMB0CF |= 0x80; // enable
}

/**
 * Read or Write via i2c0
 * 
 * @return remain bytes
 */
u8 i2c0_read_write(u8 address_wr_flag, u8 *buf, u8 size){
  
  // start bit
  STA0 = 1;
  while(!(SMB0CN & 0x05));
  STA0 = 0;
  
  if((SMB0CN & 0xDC) == 0xC0){
    // address + R/W flag
    SMB0DAT = address_wr_flag;
    SI0 = 0;
    while(!(SMB0CN & 0x05));
    
    if((SMB0CN & 0xFE) == 0xC2){
      // Read / Write
      if(address_wr_flag & 0x01){ 
        while(1){
          SI0 = 0;
          while(!(SMB0CN & 0x05));
          if((SMB0CN & 0xFC) == 0x88){
            *(buf++) = SMB0DAT;
            if(--size){
              ACK0 = 1;
            }else{
              ACK0 = 0;
              break;
            }
          }else{break;}
        }
      }else{
        while(size--){
          SMB0DAT = *(buf++);
          SI0 = 0;
          while(!(SMB0CN & 0x05));
          if((SMB0CN & 0xFE) != 0xC2){break;}
        }
      }
    }
  }
  STO0 = 1;
  SI0 = 0;
  
  return size;
}
