#ifndef __DATA_HUB_H__
#define __DATA_HUB_H__

#include "ff.h"

typedef char payload_t;
typedef unsigned char payload_size_t;

typedef struct{
  payload_t *buf_begin;
  payload_t *current;
  payload_t *buf_end;
} packet_t;

#define PAGE_SIZE 32 // ŒÅ’è

void data_hub_init();
void data_hub_load_config(char *fname, void (*func)(FIL *));
payload_size_t data_hub_assign_page(void (*call_back)(packet_t *));
void data_hub_polling();

#endif /* __DATA_HUB_H__ */
