#ifndef _UART0_H_
#define _UART0_H_

void uart0_bauding(unsigned long baudrate);
void uart0_init();

#define UART0_TX_BUFFER_SIZE 32
#define UART0_RX_BUFFER_SIZE (0x100 - 32)

#include "c8051F380.h"
#include "fifo.h"

FIFO_SIZE_T uart0_write(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart0_read(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart0_tx_size();
FIFO_SIZE_T uart0_rx_size();

void interrupt_uart0() __interrupt (INTERRUPT_UART0);

// For stdio.h
char getchar();
void putchar(char c);

#endif
