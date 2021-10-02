#include <Arduino.h>
#include <SensorFusion.h>
#include <Wire.h>
#include <math.h>

#include "imu.h"

#define PRINT_INTERVAL 100
unsigned long lastPrint = 0;  // Keep track of print time

SF ahrs;

float pitch, roll, yaw;
float deltat;

float aX, aY, aZ;
float velX, velY, velZ;
float dX, dY, dZ;

float a = 0;

// High pass butterworth filter order=1 alpha1=0.01
class FilterBuHp1 {
   public:
    FilterBuHp1() { v[0] = v[1] = 0.0; }

   private:
    float v[2];

   public:
    float step(float x)  // class II
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
    beginIMU();
}

void loop() {
    readIMU();

    deltat = ahrs.deltatUpdate();
    ahrs.MadgwickUpdate(-Gxyz[0], Gxyz[1], Gxyz[2],  // Flip Gyro Handedness
                        -Axyz[0], Axyz[1],
                        Axyz[2],  // Flip Accel Handedness
                        Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    // pitch in radians
    pitch = ahrs.getPitch() * DEG_TO_RAD;
    // roll in radians
    roll = ahrs.getRoll() * DEG_TO_RAD;
    // compensated for gravity
    aX = -Axyz[0] + GRAVITY * sin(pitch);
    aY = Axyz[1] - GRAVITY * sin(roll) * cos(pitch);
    aZ = Axyz[2] - GRAVITY * cos(pitch) * cos(roll);

    if (millis() > lastPrint + PRINT_INTERVAL) {
        lastPrint = millis();

        Serial.print("aX: ");
        Serial.println(aX);
        Serial.print("aY: ");
        Serial.println(aY);
        Serial.print("aZ: ");
        Serial.println(aZ);
    }
}
