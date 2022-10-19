#include <Arduino_MKRIoTCarrier.h>
MKRIoTCarrier carrier;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  carrier.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("lower_bound:0, ");
  //Serial.print(value);
  for (int i = 0; i < 5; i++) {
    Serial.print("button");
    Serial.print(i);
    Serial.print(":");
    Serial.print(analogRead(i));
    Serial.print(", ");
  }
  Serial.println("upper_bound:1200");
  // delay(10);
}