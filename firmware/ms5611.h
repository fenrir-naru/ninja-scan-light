#ifndef __MS5611_H__
#define __MS5611_H__

#include "type.h"

extern volatile __bit ms5611_capture;

void ms5611_init();
void ms5611_polling();

#endif /* __MS5611_H__ */
