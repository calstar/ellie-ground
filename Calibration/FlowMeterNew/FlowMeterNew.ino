
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
#define SERVO_MIN_USEC (800)
#define SERVO_MAX_USEC (2100)
#define FMPIN 19
#define SERVOPIN1 13
#define servo1ClosedPosition 100
#define servo1OpenPosition 10
float currentPosition1 = float('inf');
Servo servo1;
int IsOpened = 0;
double flowRate;
volatile float count;
unsigned long openTimeControl = 0;
unsigned long openTime = 3000;
bool pulse = 0;


double totalVL = 0;
double totalVGal = 0;
double q = 0;

unsigned long timeDelta = 0;
unsigned long firstTime;

unsigned long secondTime;


void setup() {
  pinMode(FMPIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  if (digitalRead(FMPIN)) {
    Flow();
  } else {
    Serial.println("NO PULSE");
  }
}

void Flow()
{
  firstTime = millis();
  count = 0;
  Serial.println("1");
  //calculate pulse rate


  
 
  //secondTime = millis();
  // how many counts per second
//  timeDelta = secondTime - firstTime;
//  count = 1000 / timeDelta;
//  firstTime = secondTime;
//  q = 2 + exp(count-1847);
//  // gal
//  totalVGal = totalVGal + q*timeDelta/1000;
//  totalVL =  totalVGal*3.78541;
//  Serial.println(count, 4);

}
