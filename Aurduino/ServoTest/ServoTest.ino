// Identical to DAQBoard code, except meant for flowmeter calibration

/*
This code runs on the DAQ ESP32 and has a couple of main functions.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Recieve servo commands from COM ESP32
4. Send PWM signals to servos
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"

//define pins to use for the various sensors and connections. define takes up less space on the chip
#define ONBOARD_LED  13

#define CLK 19
#define CLK2 25
#define FM 4

#define S1S 26
#define S2S 25
#define flowmeter 34

//define servo min and max values
#define SERVO_MIN_USEC (900)
#define SERVO_MAX_USEC (2100)

//bitton stuff
const int buttonpin1 = 27;
bool pressed = false;
bool prevPressed = false;
bool valveOpened = false;
int incomingS1 = 0;
int angle = 180;
float pressTime = 0;
int servo1_curr = 0;
int servo2_curr = 0;
int S1=0;

int value = 0;
int lastLED = 0;



//FM counter
float fmcount;
float flowRate;
boolean currentState;
boolean lastState = false;
float LEDtime;


//Initialize the servo objects
Servo servo1;
Servo servo2;


//define servo necessary values
int ADC_Max = 4096;


int servoangle = 90;


int count=3;

float endTime;
float timeDiff;

float refTime;
float currTime;




void setup() {
  //attach servo pins
  servo1.attach(S1S,SERVO_MIN_USEC,SERVO_MAX_USEC);
  servo2.attach(S2S,SERVO_MIN_USEC,SERVO_MAX_USEC);
  
  // attach onboard LED
  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(flowmeter, OUTPUT);


 Serial.begin(115200);
 refTime = millis();
 servo1.write(servoangle);
 servo2.write(servoangle);
 servoangle = 0;
 LEDtime = millis();

  // Set device as a Wi-Fi Station

}

void loop() {
  
  currTime = millis();
  if ((currTime - refTime) >= 5000) {
     servo1.write(servoangle);
     servo2.write(servoangle);
     refTime = millis();
  }

  analogWrite(flowmeter, value);
  if (value >= 255) {
    value = 0;
  } else {
    value += 1;
    delay(5);
  }

  if (currTime - LEDtime >= 500) {
    LEDtime = millis();
    if (lastLED) {
      digitalWrite(ONBOARD_LED, LOW);
    } else {
      digitalWrite(ONBOARD_LED, HIGH);
    }
    lastLED = 1 - lastLED;
  }
  
  
  //servo1.write(90);
  //servo2.write(90);
  //digitalWrite(ONBOARD_LED, HIGH); 
  
  //delay(5000);
  
  //servo1.write(0);
  //servo2.write(0);
  //digitalWrite(ONBOARD_LED, LOW); 
  //delay(5000);

}
