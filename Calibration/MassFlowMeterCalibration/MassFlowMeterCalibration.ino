
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
#define SERVO_MIN_USEC (800)
#define SERVO_MAX_USEC (2100)
#define FMPIN 18
#define SERVOPIN1 13
#define servo1ClosedPosition 100
#define servo1OpenPosition 10
float currentPosition1 = float('inf');
Servo servo1;
int IsOpened = 0;
double flowRate;
volatile int count;
unsigned long openTimeControl = 0;
unsigned long openTime = 3000;
bool pulse = 0;


double totalVL = 0;
double totalVGal = 0;
double q = 0;

unsigned long timeD = 0;
unsigned long firstTime = millis();
unsigned long secondTime = millis();
void setup() {
  // put your setup code here, to run once:
pinMode(FMPIN, INPUT);
//attach servo pins
servo1.attach(SERVOPIN1,SERVO_MIN_USEC,SERVO_MAX_USEC);

Serial.begin(115200);
// attachInterrupt(digitalPinToInterrupt(18), Flow, RISING);

// if you want to do an accurate valve open/close for calibration
// delay(5000);
// openTimeControl = millis();

}

void loop() {
  pulse = digitalRead(FMPIN);
  println(pulse);
  // count = 0;
  // Serial.print(millis()-openTimeControl);
//   if (millis()-openTimeControl <= openTime-1000) {
//     IsOpened = 1;
//   servo1.write(servo1OpenPosition);
//   interrupts();
//   delay(1000);
//   noInterrupts();
//   // count = 800;
//   // put your main code here, to run repeatedly:
// } else {
//   servo1.write(servo1ClosedPosition);
//   IsOpened = 0;
// }
// Serial.print(" ");
//   interrupts();
// Serial.print("flowRate (Gal/M): ");
// Serial.print(q);
// Serial.print(" Total V (Gal): ");
// Serial.print(totalVGal);
// Serial.print(" Total V (L): ");
// Serial.print(totalVL);
// Serial.print(" count: ");
// Serial.println(count);
// q = 0;
// count = 0;
delay(50);
}

void Flow()
{
  Serial.print("AHHAHAHHHA THERE IS FLUID IN THE SYSTEM HAHAHAHAHAHH");
  secondTime = millis();
  // how many counts per second
  timeD = secondTime-firstTime;
  count = 1000/timeD;
  firstTime = secondTime;
  q = 2+exp(count-1847);
  // gal
  totalVGal = totalVGal + q*timeD/1000;
  totalVL =  totalVGal*3.78541;

}
