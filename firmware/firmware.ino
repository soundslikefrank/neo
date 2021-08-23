#include <Arduino_LSM9DS1.h>
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ArduinoBLE.h>

BLEMIDI_CREATE_INSTANCE("GLOVE", MIDI)

unsigned long t0 = millis();
unsigned long t1 = millis();
bool isConnected = false;
int fingers[5] = {0, 0, 0, 0, 0};
float x = 0;
float y = 0;
float z = 0;

// -----------------------------------------------------------------------------
// When BLE connected, LED will turn on (indication that connection was
// successful) When receiving a NoteOn, LED will go out, on NoteOff, light comes
// back on. This is an easy and conveniant way to show that the connection is
// alive and working.
// -----------------------------------------------------------------------------
void setup() {
  MIDI.begin();
  Serial.begin(115200);

  IMU.begin();

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  BLEMIDI.setHandleConnected([]() {
    isConnected = true;
    digitalWrite(LED_BUILTIN, HIGH);
  });

  BLEMIDI.setHandleDisconnected([]() {
    isConnected = false;
    digitalWrite(LED_BUILTIN, LOW);
  });

  MIDI.setHandleNoteOn([](byte channel, byte note, byte velocity) {
    digitalWrite(LED_BUILTIN, LOW);
  });
  MIDI.setHandleNoteOff([](byte channel, byte note, byte velocity) {
    digitalWrite(LED_BUILTIN, HIGH);
  });
}

void loop() {
  MIDI.read();

  if (isConnected && (millis() - t0) > 1000) {
    t0 = millis();

    MIDI.sendNoteOn(60, 100, 1); // note 60, velocity 100 on channel 1

    fingers[0] = analogRead(A0); // thumb
    fingers[1] = analogRead(A1); // index finger
    fingers[2] = analogRead(A2); // middle finger
    fingers[3] = analogRead(A3); // ring finger
    fingers[4] = analogRead(A4); // little finger

    /* fingers[0] = map(fingers[0], 2840, 4000, 0, 127);
    fingers[0] = constrain(fingers[0], 0, 127);

    fingers[1] = map(fingers[1], 2840, 4000, 0, 127);
    fingers[1] = constrain(fingers[1], 0, 127);

    fingers[2] = map(fingers[2], 2840, 4000, 0, 127);
    fingers[2] = constrain(fingers[2], 0, 127);

    fingers[3] = map(fingers[3], 2840, 4000, 0, 127);
    fingers[3] = constrain(fingers[3], 0, 127);

    fingers[4] = map(fingers[4], 2840, 4000, 0, 127);
    fingers[4] = constrain(fingers[4], 0, 127); */

    /* Serial.print("thumb: ");
    Serial.println(fingers[0]);

    Serial.print("index finger: ");
    Serial.println(fingers[1]);

    Serial.print("middle finger: ");
    Serial.println(fingers[2]);

    Serial.print("ring finger: ");
    Serial.println(fingers[3]);

    Serial.print("little finger: ");
    Serial.println(fingers[4]); */
  }

  if (IMU.accelerationAvailable() && millis() - t1 > 10) {
    t1 = millis();

    /* IMU.readAcceleration(x, y, z); */
    IMU.readGyroscope(x, y, z);
    Serial.println(z);
  }
}
