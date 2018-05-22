#include <Sabertooth.h>

Sabertooth ST(128);
int potPin = 2;
int val = 0;
void setup() {
  // put your setup code here, to run once:
  SabertoothTXPinSerial.begin(9600);
  ST.autobaud();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(potPin);
  int power = val/8.06; //reads a value between 0 and 90
  
  Serial.println((int) power);
  ST.motor(1, power);
  ST.motor(2, power);
  delay(20);
}
