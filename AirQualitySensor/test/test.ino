
#include <Arduino.h>

const float max_volts = 5.0;
const float max_analog_steps = 1023.0;

#define PIN_CO  0
#define PIN_NO2 0
#define PIN_NH3 0


void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Setup initializing");

  pinMode(PIN_CO, INPUT_PULLUP);
  pinMode(PIN_NO2, INPUT_PULLUP);
  pinMode(PIN_NH3, INPUT_PULLUP);
}

void loop() {
  int a0_read = analogRead(PIN_CO);
  int a1_read = analogRead(PIN_NO2);
  int a2_read = analogRead(PIN_NH3);

  Serial.print("Latest reading in volts, CO (a0): ");
  Serial.print(a0_read * (max_volts / max_analog_steps));
  Serial.print(" NH3 (a1): ");
  Serial.print(a1_read * (max_volts / max_analog_steps));
  Serial.print(" NO2 (a2): ");
  Serial.print(a2_read * (max_volts / max_analog_steps));
  Serial.println("");

  delay(200);
}
