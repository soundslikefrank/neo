#include <SPI.h>
#include <SensorFusion.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>
#include <math.h>

#define GRAVITY 9.81
#define PRINT_INTERVAL 100
unsigned long lastPrint = 0; // Keep track of print time
unsigned long motionStart = 0;
unsigned long elapsed = 0;
unsigned long wait = 0;

LSM9DS1 imu;
SF ahrs;

float Axyz[3], Mxyz[3], Gxyz[3];
float pitch, roll, yaw;
float deltat;

float aX, aY, aZ;
float velX, velY, velZ;
float dX, dY, dZ;

float a = 0;
/* float vel = 0;
float dist = 0; */

// High pass butterworth filter order=1 alpha1=0.01
class FilterBuHp1 {
public:
  FilterBuHp1() { v[0] = v[1] = 0.0; }

private:
  float v[2];

public:
  float step(float x) // class II
  {
    v[0] = v[1];
    v[1] = (9.695312529087461995e-1 * x) + (0.93906250581749239892 * v[0]);
    return (v[1] - v[0]);
  }
};

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

    deltat = ahrs.deltatUpdate();
    ahrs.MadgwickUpdate(-Gxyz[0], Gxyz[1], Gxyz[2], // Flip Gyro Handedness
                        -Axyz[0], Axyz[1], Axyz[2], // Flip Accel Handedness
                        Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    // pitch in radians
    pitch = ahrs.getPitch() * M_PI / 180;
    // roll in radians
    roll = ahrs.getRoll() * M_PI / 180;
    // compensated for gravity
    aX = -Axyz[0] + GRAVITY * sin(pitch);
    aY = Axyz[1] - GRAVITY * sin(roll) * cos(pitch);
    aZ = Axyz[2] - GRAVITY * cos(pitch) * cos(roll);

    // absolute value of acceleration
    a = sqrt(aX * aX + aY * aY + aZ * aZ);

    if (millis() - wait > 1000 && a > 4 && !motionStart) {
      motionStart = millis();
      wait = 0;
      velX = 0;
      velY = 0;
      velZ = 0;
      dX = 0;
      dY = 0;
      dZ = 0;
      Serial.println("START");
    }

    if (motionStart) {
      // integrate
      velX = velX + aX;
      velY = velY + aY;
      velZ = velZ + aZ;

      dX = dX + velX;
      dY = dY + velY;
      dZ = dZ + velZ;

      elapsed = millis() - motionStart;
      Serial.println(dX);
    }

    if (elapsed > 300 && a > 4) {
      motionStart = 0;
      elapsed = 0;
      wait = millis();
      Serial.println("STOP");
    }

    /* vel = sqrt(velX*velX + velY*velY + velZ*velZ); */

    if (millis() - lastPrint >= PRINT_INTERVAL) {
      lastPrint = millis();

      /* Serial.print(velX);
      Serial.print(" ");
      Serial.print(velY);
      Serial.print(" ");
      Serial.println(velZ); */
      /* vel = 0;
      dist = 0; */

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
      /* Serial.println(ahrs.getYaw()); */
      /* Serial.print(", ");
      Serial.print(ahrs.getPitch());
      Serial.print(", ");
      Serial.println(ahrs.getRoll()); */

      /* float *q;
      q = ahrs.getQuat();
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

float A_B[3]{-182.12, -271.40, -119.74};

float A_Ainv[3][3]{{0.99866, -0.00285, 0.00023},
                   {-0.00285, 0.98698, 0.01243},
                   {0.00023, 0.01243, 0.99770}};

float M_B[3]{-25.45, 2867.99, 693.87};

float M_Ainv[3][3]{{1.47036, 0.02954, 0.02213},
                   {0.02954, 1.49626, -0.00035},
                   {0.02213, -0.00035, 1.52212}};

void get_IMU(float Axyz[3], float Mxyz[3]) {

  // FIXME: It seems we need to calibrate the gyro: https://github.com/aster94/SensorFusion/issues/7#issuecomment-877002733
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
