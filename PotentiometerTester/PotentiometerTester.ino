int potPin = 2;
int val = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(potPin);
  float angle = val;///11.366666; //reads a value between 0 and 90
  Serial.println((int) angle);
  delay (250);
}
