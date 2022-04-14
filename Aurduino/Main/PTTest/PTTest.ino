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
#define PTDOUT 33
#define CLKPT 27

float pt1=-1;


//define servo min and max values
#define SERVO_MIN_USEC (900)
#define SERVO_MAX_USEC (2100)

//Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;


//Initialize the servo objects
Servo servo1;
Servo servo2;

//define servo necessary values
int ADC_Max = 4096;


void setup() {

//set gains for pt pins
  scale1.begin(PTDOUT, CLKPT);
  scale1.set_gain(64);
     //Sets the pin as an input

  Serial.begin(115200);

}

void loop() {

  getReadings();

  delay(5);

}

void getReadings(){


 pt1 = scale1.read();
       Serial.print("pt1: ");

 Serial.println(pt1);


}
