/* Test PWM functionality by dimming an external LED according to the position of a trimmer */

const int ANALOG_INPUT_PIN = A0;
const int ANALOG_OUTPUT_PIN = 5;

// the setup function runs once when you press reset or power the board
void setup() {
}

// the loop function runs over and over again forever
void loop() {
  unsigned int reading = analogRead(ANALOG_INPUT_PIN); // take a reading
  analogWrite(ANALOG_OUTPUT_PIN, reading / 4); // set PWM duty cycle. divide by 4 to map 0-1023 -> 0-255
}
