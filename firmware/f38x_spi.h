#ifndef __F34X_SPI_H__
#define __F34X_SPI_H__

#include "main.h"
#include "type.h"

u8 spi_ckr(u8 new_value);
void spi_init();

#define spi_clock(khertz) spi_ckr((u8)((SYSCLK / 1000 / 2 / khertz) - 1)) 

unsigned char spi_write_read_byte(unsigned char byte);

void spi_send_8clock();
void spi_read(unsigned char * pchar, unsigned int length);
void spi_write(unsigned char * pchar, unsigned int length);

#endif
