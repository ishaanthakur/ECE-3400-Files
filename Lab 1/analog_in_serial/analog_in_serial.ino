/* Test analog input functionality by printing a resistive divider/trimmer voltage at half second intervals */

const int ANALOG_INPUT_PIN = A1;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600); // set the baud rate to 9600
}

// the loop function runs over and over again forever
void loop() {
  unsigned int reading = analogRead(ANALOG_INPUT_PIN); // take a reading
  Serial.println(reading); // print to serial
  delay(500); // wait half a second
}
