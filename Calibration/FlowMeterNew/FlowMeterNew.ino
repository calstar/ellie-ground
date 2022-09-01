
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#define FMPIN 19

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
unsigned long duration;


void setup() {
  pinMode(FMPIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  duration = pulseIn(FMPIN, HIGH);
  Serial.println(duration);
}

void Flow()
{
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
