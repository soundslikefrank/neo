#ifndef __IMU_H
#define __IMU_H

#define GRAVITY 9.81

extern float Axyz[3], Mxyz[3], Gxyz[3];

void beginIMU();
void readIMU();

#endif /* __IMU_H */
