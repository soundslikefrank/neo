#include "imu.h"

#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;
float Axyz[3], Mxyz[3], Gxyz[3];
// Calibration data (see support/calibration)
float A_B[3]{-182.12, -271.40, -119.74};
float A_Ainv[3][3]{{0.99866, -0.00285, 0.00023},
                   {-0.00285, 0.98698, 0.01243},
                   {0.00023, 0.01243, 0.99770}};
float M_B[3]{-25.45, 2867.99, 693.87};
float M_Ainv[3][3]{{1.47036, 0.02954, 0.02213},
                   {0.02954, 1.49626, -0.00035},
                   {0.02213, -0.00035, 1.52212}};

void beginIMU() {
    if (imu.begin(0x6B, 0x1E, Wire1) == false) {
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(10);
            digitalWrite(LED_BUILTIN, LOW);
            delay(20);
        }
    }
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
}

void readIMU() {
    // FIXME: It seems we need to calibrate the gyro:
    // https://github.com/aster94/SensorFusion/issues/7#issuecomment-877002733
    byte i;
    float temp[3];

    if (imu.accelAvailable() && imu.magAvailable() && imu.gyroAvailable()) {
        imu.readAccel();
        imu.readMag();
        imu.readGyro();

        Axyz[0] = imu.ax;
        Axyz[1] = imu.ay;
        Axyz[2] = imu.az;
        Mxyz[0] = imu.mx;
        Mxyz[1] = imu.my;
        Mxyz[2] = imu.mz;

        // apply offsets (bias) and scale factors from Magneto
        for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
        Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] +
                  A_Ainv[0][2] * temp[2];
        Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] +
                  A_Ainv[1][2] * temp[2];
        Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] +
                  A_Ainv[2][2] * temp[2];

        // apply offsets (bias) and scale factors from Magneto
        for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
        Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] +
                  M_Ainv[0][2] * temp[2];
        Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] +
                  M_Ainv[1][2] * temp[2];
        Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] +
                  M_Ainv[2][2] * temp[2];

        Gxyz[0] = imu.calcGyro(imu.gx) * DEG_TO_RAD;
        Gxyz[1] = imu.calcGyro(imu.gy) * DEG_TO_RAD;
        Gxyz[2] = imu.calcGyro(imu.gz) * DEG_TO_RAD;

        Axyz[0] = imu.calcAccel(Axyz[0]) * GRAVITY;
        Axyz[1] = imu.calcAccel(Axyz[1]) * GRAVITY;
        Axyz[2] = imu.calcAccel(Axyz[2]) * GRAVITY;

        Mxyz[0] = imu.calcMag(Mxyz[0]);
        Mxyz[1] = imu.calcMag(Mxyz[1]);
        Mxyz[2] = imu.calcMag(Mxyz[2]);
    }
}
