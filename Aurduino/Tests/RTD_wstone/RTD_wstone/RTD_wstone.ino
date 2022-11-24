#include <Arduino.h>
#define V_READ_PIN 15
#define R_MIN 18 //resistance (Ohms) where bridge voltage = 0
#define R_REF 2200 //resistance of other two resistors in bridge
#define V_IN 3300 //input voltage in mV
#define ALPHA 0.00385
#define R_0 100


float wstone_factor; //made this a thing to make calculations easier
float v_out; //in mV
float read_value; //output of analog read
float r_rtd;
float temp;

void setup() {
  Serial.begin(115200);
}

void loop() {
  read_value = analogRead(V_READ_PIN);
  v_out = (read_value * 4096) / 3300;
  wstone_factor = (v_out / V_IN) + (R_0 / (R_MIN + R_REF));

  r_rtd = (wstone_factor / (1 - wstone_factor)) * R_REF;

  temp = ((r_rtd / R_0) - 1) / ALPHA;
  
  Serial.print("analog read: ");
  Serial.print(read_value);
  Serial.print("  voltage: ");
  Serial.print(v_out);
  Serial.print("  resistance: ");
  Serial.print(r_rtd);
  Serial.print("  temp: ");
  Serial.println(temp);
}
