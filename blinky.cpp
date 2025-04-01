#include <Arduino.h>

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.print("on\n");
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);
  Serial.printf("off\n");    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

