#ifndef __MAG3110_H__
#define __MAG3110_H__

#include "type.h"

extern volatile __bit mag3110_capture;

void mag3110_init();
void mag3110_polling();

#endif /* __MAG3110_H__ */
