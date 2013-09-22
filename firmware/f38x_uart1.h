#ifndef _UART1_H_
#define _UART1_H_

void uart1_bauding(unsigned long baudrate);
void uart1_init();

#define UART1_TX_BUFFER_SIZE 128
#define UART1_RX_BUFFER_SIZE 64

#include "c8051F380.h"
#include "fifo.h"

FIFO_SIZE_T uart1_write(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart1_read(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart1_tx_size();
FIFO_SIZE_T uart1_rx_size();

void interrupt_uart1() __interrupt (INTERRUPT_UART1);

#endif
