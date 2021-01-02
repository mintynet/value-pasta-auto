                                                  // Teensyduino 1.53
                                                  // Arduino 1.8.13
//**************************************************
void setup() {
  // initialize both serial ports:
  Serial.begin(250000);
  Serial5.begin(250000);
}
//**************************************************
void loop() {
  // read from port 1, send to port 0:
  if (Serial5.available()) {
    int inByte = Serial5.read();
    Serial.write(inByte);
  }

  // read from port 0, send to port 1:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial5.write(inByte);
  }
}
//**************************************************
