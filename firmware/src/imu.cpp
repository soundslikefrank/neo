#include "imu.h"

#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;
float Axyz[3], Mxyz[3], Gxyz[3];

// Calibration data (see support/calibration)
// Accelerometer
float A_B[3]{-209.32, -431.02, -81.96};
float A_Ainv[3][3]{{0.99720, -0.00122, 0.00208},
                   {-0.00122, 1.00531, -0.00307},
                   {0.00208, -0.00307, 0.99510}};

// Magnetometer
float M_B[3]{-368.72, 2484.30, 872.10};
float M_Ainv[3][3]{{1.34929, 0.06080, 0.01471},
                   {0.06080, 1.30756, 0.00921},
                   {0.01471, 0.00921, 1.34956}};

// Gyroscope
int G_Offset[3]{65, -13, 21};

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

        Gxyz[0] = imu.calcGyro(imu.gx - G_Offset[0]) * DEG_TO_RAD;
        Gxyz[1] = imu.calcGyro(imu.gy - G_Offset[1]) * DEG_TO_RAD;
        Gxyz[2] = imu.calcGyro(imu.gz - G_Offset[2]) * DEG_TO_RAD;

        Axyz[0] = imu.calcAccel(Axyz[0]) * GRAVITY;
        Axyz[1] = imu.calcAccel(Axyz[1]) * GRAVITY;
        Axyz[2] = imu.calcAccel(Axyz[2]) * GRAVITY;

        Mxyz[0] = imu.calcMag(Mxyz[0]);
        Mxyz[1] = imu.calcMag(Mxyz[1]);
        Mxyz[2] = imu.calcMag(Mxyz[2]);
    }
}
