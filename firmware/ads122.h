#ifndef __ADS122_H__
#define __ADS122_H__

#include "type.h"

extern volatile __bit ads122_capture;

void ads122_init();
void ads122_polling();

#endif /* __ADS122_H__ */
