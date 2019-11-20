/* Driver code */

#include <Arduino.h>
#include <Servo.h>

#include "drivers.hpp"

// allocate servo structures
static Servo servo_left;
static Servo servo_right;

static const int RED_LED_PIN = 9; // red indicator pin
static const int GREEN_LED_PIN = 8; // green indicator pin
static const int BLUE_LED_PIN = 13; // blue indicator pin
static const int LINE_LEFT_PIN = 4; // left line sensor input pin
static const int LINE_RIGHT_PIN = 7; // right line sensor input pin
static const int LINE_REAR_PIN = 2; // rear line sensor input pin (used for intersection detection)
static const int IR_LIGHTS_PIN = 3; // IR lights control pin
static const int IR_FRONT_PIN = A3; // front ir sensor input pin
static const int IR_SIDE_PIN = A5; // side ir sensors input pin
static const int IR_L_SEL_PIN = 1; // left ir sensor select pin
static const int IR_R_SEL_PIN = 0; // right ir sensor select pin
static const int DISTANCE_FRONT_PIN = A1; // front distance sensor input pin
static const int DISTANCE_LEFT_PIN = A4; // left distance sensor input pin
static const int DISTANCE_RIGHT_PIN = A2; // right distance sensor input pin
static const int SERVO_LEFT_PIN = 6; // left servo control pin
static const int SERVO_RIGHT_PIN = 5; // right servo control pin

static const uint32_t CLK_FREQ = 16000000; // arduino clock frequency
static const float ADC_CONV_TIME = 13.0f; // ADC clock cycles / conversion 

void driversTest() {
  Serial.println("Drivers Test");
  Serial.println();

  // test LEDs
  /*redLEDOn();
  greenLEDOn();
  blueLEDOn();*/

  // test line sensors
  /*while(1) {
    if(checkLineLeft()) { 
      redLEDOn();
    }
    else { 
      redLEDOff();
    }

    if(checkLineRight()) { 
      greenLEDOn();
    }
    else {
      greenLEDOff();
    }
    
    if(checkLineRear()) {
      blueLEDOn();
    }
    else { 
      blueLEDOff();
    }
  }*/
  
  // test distance sensors
  /*while(1) {
    Serial.print("Front Distance Sensor: ");
    Serial.println(getDistanceFront());
    Serial.print("Left Distance Sensor: ");
    Serial.println(getDistanceLeft());
    Serial.print("Right Distance Sensor: ");
    Serial.println(getDistanceRight());
    Serial.println();
    delay(1000);
  }*/

  // test IR sensors
  while(1) {
    uint16_t val;

    // multiple delays needed because IR detection uses serial pins
    Serial.print("Front IR Sensors: ");
    val = getIRFront();
    delay(100);
    Serial.println(val);
    
    Serial.print("Left IR Sensor: ");
    val = getIRLeft();
    delay(100);
    Serial.println(val);
    
    Serial.print("Right IR Sensor: ");
    val = getIRRight();
    delay(100);
    Serial.println(val);
    
    Serial.println();
    delay(700);
  }

  // test servos
  /*setSpeedLeft(100);
  setSpeedRight(100);*/
}

void ledInit() {
  pinMode(RED_LED_PIN, OUTPUT); // make red indicator pin an output
  pinMode(GREEN_LED_PIN, OUTPUT); // make green indicator pin an output
  pinMode(BLUE_LED_PIN, OUTPUT); // make blue indicator pin an output
}

// LEDs active high
void redLEDOn() {
  digitalWrite(RED_LED_PIN, HIGH);
}

void greenLEDOn() {
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void blueLEDOn() {
  digitalWrite(BLUE_LED_PIN, HIGH);
}

void redLEDOff() {
  digitalWrite(RED_LED_PIN, LOW);
}

void greenLEDOff() {
  digitalWrite(GREEN_LED_PIN, LOW);
}

void blueLEDOff() {
  digitalWrite(BLUE_LED_PIN, LOW);
}

void redLEDToggle() {
  digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
}

void greenLEDToggle() {
  digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
}

void blueLEDToggle() {
  digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));
}

// white line detection
bool checkLineLeft() {
  return !digitalRead(LINE_LEFT_PIN);
}

bool checkLineRight() {
  return !digitalRead(LINE_RIGHT_PIN);
}

bool checkLineRear() {
  return !digitalRead(LINE_REAR_PIN);
}

// distance sensor transfer functions obtained from datasheet
float getDistanceFront() {
  return 2457.6f / analogRead(DISTANCE_FRONT_PIN) - 0.42f;
}

float getDistanceLeft() {
  return 2457.6f / analogRead(DISTANCE_LEFT_PIN) - 0.42f;
}

float getDistanceRight() {
  return 2457.6f / analogRead(DISTANCE_RIGHT_PIN) - 0.42f;
}

void irInit() {
  pinMode(IR_LIGHTS_PIN, OUTPUT); // make IR lights control pin an output 
}

// IR lights active high
void irOn() {
  digitalWrite(IR_LIGHTS_PIN, HIGH);
}

void irOff() {
  digitalWrite(IR_LIGHTS_PIN, LOW);
}

uint16_t getIRFront() {
  irOff(); // prevent own IR lights from being detected
  delayMicroseconds(200); 
  uint16_t ret = analogRead(IR_FRONT_PIN);
  irOn(); // turn IR lights back on
  
  return ret;
}

uint16_t getIRLeft() {
  Serial.end(); // need to use serial pins
  
  // select the left IR sensor by connecting the phototransistor to GND
  pinMode(IR_L_SEL_PIN, OUTPUT);
  digitalWrite(IR_L_SEL_PIN, LOW);
  
  irOff(); // turn off own lights
  delayMicroseconds(200);
  uint16_t ret = analogRead(IR_SIDE_PIN); // read the shared input pin
  irOn(); // turn lights back on
  
  pinMode(IR_L_SEL_PIN, INPUT); // deactivate left IR sensor by leaving its phototransistor floating at one end
  Serial.begin(9600); // turn serial back on
  
  return ret;
}

uint16_t getIRRight() {
  Serial.end(); // need to use serial pins

  // select the right IR sensor by connecting the phototransistor to GND
  pinMode(IR_R_SEL_PIN, OUTPUT);
  digitalWrite(IR_R_SEL_PIN, LOW);
  
  irOff(); // turn off own lights
  delayMicroseconds(200);
  uint16_t ret = analogRead(IR_SIDE_PIN); // read the shared input pin
  irOn();
  
  pinMode(IR_R_SEL_PIN, INPUT); // deactivate right IR sensor by leaving its phototransistor floating at one end
  Serial.begin(9600); // turn serial back on
  
  return ret;
}

void servoInit() {
  // associate servo structures with corresponding control pins
  servo_left.attach(SERVO_LEFT_PIN);
  servo_right.attach(SERVO_RIGHT_PIN);

  // halt the motors on startup
  setSpeedLeft(0);
  setSpeedRight(0);
}

void setSpeedLeft(int8_t spd) {
  // map [-100, 100] -> [1300, 1700]
  servo_left.writeMicroseconds(1500 + (int16_t)spd * 2);
}

void setSpeedRight(int8_t spd) {
  // map [-100, 100] -> [1300, 1700]
  servo_right.writeMicroseconds(1500 - (int16_t)spd * 2);
}

float adcBeepInit() {
  ADMUX = 1 << REFS0; // use Vcc as analog reference
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | (1 << ADPS2) | (1 << ADPS1); // enable, start, set the adc to free running mode, and clear flag. set prescaler to 64

  return ((float)CLK_FREQ / 64.0f / ADC_CONV_TIME); // return the sampling frequency
}

void adcAnalogReadInit() {
  ADCSRA = (1 << ADEN) | (1 << ADIF) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable and clear flag. set the prescaler to 128
}

bool adcSampleAvailable() {
  if(ADCSRA & (1 << ADIF)) { // check whether the conversion complete flag has been set
    ADCSRA |= 1 << ADIF; // clear the flag
    return 1;
  }
  else {
    return 0; 
  }
}

uint16_t getADCSample() {
  uint16_t sample = ADCL; // ADCL must be read first
  sample |= ADCH << 8;

  return sample;
}
