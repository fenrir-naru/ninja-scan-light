#ifndef __GPS_H__
#define __GPS_H__

/*
 * u-block GPS => Little Endian!!
 * sdcc => Little Endian
 * 
 */

#include <time.h>
#include "type.h"

#define MAX_SAT 16
#define USE_GPS_STD_TIME 0

void gps_init();
void gps_polling();
void gps_write(char *buf, int size);
#if USE_GPS_STD_TIME
time_t gps_std_time(time_t *timeptr);
#endif

extern volatile __bit gps_time_modified;
extern __xdata u8 gps_num_of_sat;
extern __xdata u16 gps_wn;
extern __xdata s32 gps_ms;

extern __bit gps_utc_valid;
extern __xdata struct tm gps_utc;

#endif /* __GPS_H__ */
