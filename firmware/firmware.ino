#include "SensorFusion.h"
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>

#define GRAVITY 9.81
#define PRINT_INTERVAL 12
unsigned long lastPrint = 0; // Keep track of print time

LSM9DS1 imu;
SF filter;

float Axyz[3], Mxyz[3], Gxyz[3];
float pitch, roll, yaw;
float deltat;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
  }

  Wire1.begin();
  delay(100);
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

void loop() {
  if (imu.accelAvailable() && imu.magAvailable() && imu.gyroAvailable()) {
    imu.readAccel();
    imu.readMag();
    imu.readGyro();

    get_IMU(Axyz, Mxyz);
    Gxyz[0] = imu.calcGyro(imu.gx) * DEG_TO_RAD;
    Gxyz[1] = imu.calcGyro(imu.gy) * DEG_TO_RAD;
    Gxyz[2] = imu.calcGyro(imu.gz) * DEG_TO_RAD;

    Axyz[0] = imu.calcAccel(Axyz[0]) * GRAVITY;
    Axyz[1] = imu.calcAccel(Axyz[1]) * GRAVITY;
    Axyz[2] = imu.calcAccel(Axyz[2]) * GRAVITY;

    Mxyz[0] = imu.calcMag(Mxyz[0]);
    Mxyz[1] = imu.calcMag(Mxyz[1]);
    Mxyz[2] = imu.calcMag(Mxyz[2]);

    deltat = filter.deltatUpdate();
    filter.MadgwickUpdate(-Gxyz[0], Gxyz[1], Gxyz[2], // Flip Gyro Handedness
                          -Axyz[0], Axyz[1], Axyz[2], // Flip Accel Handedness
                          Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint >= PRINT_INTERVAL) {
      lastPrint = millis();
      /*
      Serial.print("A: ");
      Serial.print(-Axyz[0]);
      Serial.print(", ");
      Serial.print(Axyz[1]);
      Serial.print(", ");
      Serial.println(Axyz[2]);

      Serial.print("G: ");
      Serial.print(-Gxyz[0]);
      Serial.print(", ");
      Serial.print(Gxyz[1]);
      Serial.print(", ");
      Serial.println(Gxyz[2]);

      Serial.print("M: ");
      Serial.print(Mxyz[0]);
      Serial.print(", ");
      Serial.print(Mxyz[1]);
      Serial.print(", ");
      Serial.println(Mxyz[2]);
      */

      /* Serial.println("Orientation: "); */
      Serial.println(filter.getYaw());
      /* Serial.print(", ");
      Serial.print(filter.getPitch());
      Serial.print(", ");
      Serial.println(filter.getRoll()); */

      /* float *q;
      q = filter.getQuat();
      Serial.print("Quaternion: ");
      Serial.print(q[0], 4);
      Serial.print(", ");
      Serial.print(q[1], 4);
      Serial.print(", ");
      Serial.print(q[2], 4);
      Serial.print(", ");
      Serial.println(q[3], 4); */
    }
  }
}

// Accel scale 16457.0 to normalize
float A_B[3]{-654.19, 161.87, -597.01};

float A_Ainv[3][3]{{1.00475, -0.03135, -0.00073},
                   {-0.03135, 1.00864, 0.00819},
                   {-0.00073, 0.00819, 0.99144}};

// Mag scale 3746.0 to normalize
float M_B[3]{1326.39, 1526.05, 1503.85};

float M_Ainv[3][3]{{1.16966, 0.03568, -0.00257},
                   {0.03568, 1.19523, -0.02172},
                   {-0.00257, -0.02172, 1.15745}};

void get_IMU(float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
  Axyz[0] = imu.ax;
  Axyz[1] = imu.ay;
  Axyz[2] = imu.az;
  Mxyz[0] = imu.mx;
  Mxyz[1] = imu.my;
  Mxyz[2] = imu.mz;

  // apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++)
    temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] =
      A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] =
      A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] =
      A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];

  // apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++)
    temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] =
      M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] =
      M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] =
      M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
}
