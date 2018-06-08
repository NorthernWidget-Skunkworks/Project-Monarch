//DysonDemoRead.ino
#include "Wire.h"

#define ADR 0x40

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Welcome to the Machine...");
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(ADR);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(ADR, 0x1A);
  delay(10);
  while(Wire.available() > 0) {
  	Serial.println(Wire.read());
  }
  Serial.print("\n\n");
  delay(1000);

}