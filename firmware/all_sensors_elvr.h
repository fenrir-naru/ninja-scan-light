#ifndef __ALL_SENSORS_ELVR_H__
#define __ALL_SENSORS_ELVR_H__

#include "type.h"

extern volatile __bit as_elvr_capture;

void as_elvr_init();
void as_elvr_polling();

#endif /* __ALL_SENSORS_ELVR_H__ */
