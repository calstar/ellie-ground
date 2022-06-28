
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
void setup() {
  // put your setup code here, to run once:
pinMode(FMPIN, INPUT_PULLUP);
//attach servo pins
servo1.attach(SERVOPIN1,SERVO_MIN_USEC,SERVO_MAX_USEC);

Serial.begin(115200);
attachInterrupt(0, Flow, RISING);

delay(5000);
openTimeControl = millis();

}

void loop() {
  count = 0;
  // Serial.print(millis()-openTimeControl);
  if (millis()-openTimeControl <= openTime-1000) {
    IsOpened = 1;
  servo1.write(servo1OpenPosition);
  interrupts();
  delay(1000);
  noInterrupts();
  // count = 800;
  // put your main code here, to run repeatedly:
} else {
  servo1.write(servo1ClosedPosition);
  IsOpened = 0;
}
// Serial.print(" ");
Serial.print(count);
Serial.print(" ");
Serial.println(IsOpened);
}

void Flow()
{
  count++;
}
