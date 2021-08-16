#include <BLEMIDI_Transport.h>

#include <hardware/BLEMIDI_ESP32_NimBLE.h>
/* #include <hardware/BLEMIDI_ESP32.h> */
// #include <hardware/BLEMIDI_nRF52.h>
// #include <hardware/BLEMIDI_ArduinoBLE.h>

BLEMIDI_CREATE_INSTANCE("GLOVE", MIDI)

unsigned long t0 = millis();
bool isConnected = false;

// -----------------------------------------------------------------------------
// When BLE connected, LED will turn on (indication that connection was
// successful) When receiving a NoteOn, LED will go out, on NoteOff, light comes
// back on. This is an easy and conveniant way to show that the connection is
// alive and working.
// -----------------------------------------------------------------------------
void setup() {
  MIDI.begin();
  Serial.begin(115200);

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
  delay(1000);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

int potValue = 0;
int value = 0;

void loop() {
  MIDI.read();

  if (millis() - t0 > 1001) {
    t0 = millis();
    potValue = analogRead(34);
    value = map(potValue, 2840, 4000, 0, 127);
    value = constrain(value, 0, 127);
    Serial.println(value);
    if (isConnected) {
      /* MIDI.sendControlChacge(1, value, 15); */
      MIDI.sendNoteOn(60, 100, 1); // note 60, velocity 100 on channel 1
    }
  }
}
