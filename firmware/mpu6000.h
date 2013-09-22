#ifndef __MPU6000_H__
#define __MPU6000_H__

extern volatile __bit mpu6000_capture;

void mpu6000_init();
void mpu6000_polling();

#endif /* __MPU6000_H__ */
