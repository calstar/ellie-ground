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
// #define ONBOARD_LED  13
//#define PT1DOUT 33
//#define PT2DOUT 16 //update
//#define CLKPT1 27
//#define CLKPT2 25 //update
#define FM 4 //update
#define S1S 13
#define S2S 12
#define igniterPin 14
#define igniterPin2 27


//RESOLDER GROUND ON PROTOBOARD

int servo1ClosedPosition = 100;
int servo1OpenPosition = 0;
int servo2ClosedPosition = 155;
int servo2OpenPosition = 20;

float currentPosition1 = float('inf');
float currentPosition2 = float('inf');
//EH VENT SEFRVO 1 =180

//For breadboard
#define ONBOARD_LED  13
#define PTDOUT1 32
#define CLKPT1 5
#define PTDOUT2 15
#define CLKPT2 2
#define PTDOUT3 22
#define CLKPT3 23
#define PTDOUT4 19
#define CLKPT4 21
#define PTDOUT5 35
#define CLKPT5 25
#define PTDOUT6 34
#define CLKPT6 26
#define PTDOUT7 39
#define CLKPT7 33

#define FM 34
#define VOLTAGEIN 12
//#define S1S 21

//define servo min and max values
#define SERVO_MIN_USEC (900)
#define SERVO_MAX_USEC (2100)

//Initialize flow meter variables for how it computes the flow amount
float currentMillis = 0;
float goalTime = 50;
float currReading1;
float currReading2;
float loopTime=10;

float servo1curr =0;
float servo2curr=0;

//FM counter
float fmcount;
float flowRate;
boolean currentState;
boolean lastState = false;

// Serial Message setup
String serialMessage = "";

//Measuring output from voltage divider
int readVoltage;
float convertedVoltage;

//Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;
HX711 scale5;
HX711 scale6;
HX711 scale7;

//Initialize the servo objects
Servo servo1;
Servo servo2;

//define servo necessary values
int ADC_Max = 4096;

///////////////
//IMPORTANT
//////////////
// REPLACE WITH THE MAC Address of your receiver

//OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
//COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
//HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
//NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8};


int count=3;

// Define variables to store readings to be sent
float pt1=1;
float pt2=1;
float pt3=1;
float pt4=1;
float lc5=1;
float lc6=1;
float lc7=1;
float fm=2;
//the following are only used in the oposite direction, they are included because it may be necessary for the structure to be the same in both directions
int S1; int S2; int S1S2; int I;

// Define variables to store incoming commands, servos and igniter
int incomingS1;
int incomingS2;
int incomingS1S2;
bool incomingI;


float startTime;
float endTime;
float timeDiff;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float pt1;
    float pt2;
    float pt3;
    float pt4;
    float lc1;
    float lc2;
    float lc3;
    float fm;

    int S1;
    int S2;
    int S1S2;
    bool I;
} struct_message;

// Create a struct_message called Readings to hold sensor readings
struct_message Readings;

// Create a struct_message to hold incoming commands
struct_message Commands;


esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // if (status ==0){
  //   success = "Delivery Success :)";
  // }
  // else{
  //   success = "Delivery Fail :(";
  // }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Commands, incomingData, sizeof(Commands));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
      digitalWrite(ONBOARD_LED,HIGH);
  S1 =Commands.S1;
  S2 = Commands.S2;
  S1S2 = Commands.S1S2;
  I = Commands.I;
  //if (I) {
    //fireSequence();
  //}
  if (S1S2 == 99) {
    digitalWrite(igniterPin, LOW);
    digitalWrite(igniterPin2, LOW);
    delay(750);
    digitalWrite(igniterPin, HIGH);
    digitalWrite(igniterPin2, HIGH);

  }
}

void fireSequence() {
    //Serial.println( "In Fire Sequence");
  servo1curr=servo1OpenPosition;
  servo2curr=servo2OpenPosition;

  servo1.write(servo1curr);
  servo2.write(servo2curr);
  float beginTime = millis();
  float currentTime = millis();
  while ((currentTime - beginTime) <= 3000) {
    currentTime = millis();
    servo1.write(servo1curr);
    servo2.write(servo2curr);
    S1=servo1curr;
    S2=servo2curr;


    delay(5);

  }
  beginTime = millis();
  currentTime = millis();

  servo1curr=servo1ClosedPosition;
   servo2curr=servo2OpenPosition;


  while ((currentTime - beginTime) <= 10) {
    currentTime = millis();
    S1=servo1curr;
    S2=servo2curr;

  servo1.write(servo1curr);
  servo2.write(servo2curr);


     delay(5);

  }
  servo1curr=servo1ClosedPosition;
  servo2curr=servo2ClosedPosition;

  servo1.write(servo1curr);

  servo2.write(servo2curr);

}

void setup() {
  //attach servo pins
  servo1.attach(S1S,SERVO_MIN_USEC,SERVO_MAX_USEC );
  servo2.attach(S2S,SERVO_MIN_USEC,SERVO_MAX_USEC );

  // attach onboard LED
  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(igniterPin, OUTPUT);
  pinMode(igniterPin2, OUTPUT);

  digitalWrite(igniterPin, HIGH);
  digitalWrite(igniterPin2, HIGH);


//attach flowmeter pin
  //pinMode(FM, INPUT_PULLUP);

//set gains for pt pins
//set gains for pt pins
  scale1.begin(PTDOUT1, CLKPT1);
  scale1.set_gain(64);
     //Sets the pin as an input

//set gains for pt pins
  scale2.begin(PTDOUT2, CLKPT2);
  scale2.set_gain(64);

  //set gains for pt pins
  scale3.begin(PTDOUT3, CLKPT3);
  scale3.set_gain(64);

  //set gains for pt pins
  scale4.begin(PTDOUT4, CLKPT4);
  scale4.set_gain(64);

  //set gains for pt pins
  scale5.begin(PTDOUT5, CLKPT5);
  scale5.set_gain(64);

  //set gains for pt pins
  scale6.begin(PTDOUT6, CLKPT6);
  scale6.set_gain(64);

//set gains for pt pins
  scale7.begin(PTDOUT7, CLKPT7);
  scale7.set_gain(64);

//Flowmeter untreupt
  pinMode(FM, INPUT);           //Sets the pin as an input

  Serial.begin(115200);


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  //reading voltage
  // readVoltage = analogRead(VOLTAGEIN);
  // convertedVoltage = readVoltage * (3.3/4032) * ((22 + 68) / 22);
  // Serial.print("Voltage is: ");
  // Serial.print(convertedVoltage);
  //
  // startTime=millis();
  // Serial.println("In Main Loop");
  // //Set LED back to low
  // digitalWrite(ONBOARD_LED,LOW);

 //ADD PRINT STATEMENTS FOR DEBUGGING HERE IF NCESSARY
 // printSerial();

//UPDATE SERVO POSITIONS
  //Check new data for servo status updates
  //switch (S1) {
    //case 0:
        //servo1.write(0);
        //break;
     //case 45:
        //servo1.write(45);
        //break;
    //case 90:
        //servo1.write(90);
        //break;
    //case 130:
        //servo1.write(135);
        //break;
  //}
    servo1curr=S1;
    servo2curr=S2;
    if (abs((currentPosition1 - servo1curr)) >= 2) {
      currentPosition1 = servo1curr;
      servo1.write(servo1curr);
    }
    if (abs((currentPosition2 - servo2curr)) >= 2) {
      currentPosition2 = servo2curr;
      servo2.write(servo2curr);
    }
    //servo1.write(servo1curr);
    //servo2.write(servo2curr);

  //Serial.print("loop");

  getReadings();
  //Serial.print("loop2");

  // Set values to send
  Readings.pt1 = pt1;
  Readings.pt2 = pt2;
  Readings.pt3 = pt3;
  Readings.pt4 = pt4;
  Readings.lc1 = lc5;
  Readings.lc2 = lc6;
  Readings.lc3 = lc7;
  Readings.fm  = fm;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));

  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }

  endTime=millis();
  timeDiff=endTime-startTime;
//  if (timeDiff<loopTime) {
//    delay(timeDiff);
//  }

  delay(29);

// Serial.println(servo1curr);
// Serial.println(servo2curr);

}

void getReadings(){
  currentMillis = millis();
  fmcount = 0;

 while (millis() - currentMillis < goalTime) {
    servo1.write(servo1curr);
    servo2.write(servo2curr);



    currentState = digitalRead(FM);
    if (!(currentState == lastState)) {

     lastState = currentState;
     fmcount += 1;
   }
 }
  flowRate = fmcount;
  fm =int(flowRate+1);  // Print the integer part of the variable

 pt1 = scale1.read();
      //Serial.print("pt1");
      //Serial.print(" ");

 pt2 = scale2.read();
      //Serial.print("pt2");
      //Serial.print(" ");

 pt3 = scale3.read();
 //Serial.print("pt3");
 //Serial.print(" ");

 pt4 = scale4.read();

 lc5 = scale5.read();
 lc6 = scale6.read();
 lc7 = scale7.read();

 serialMessage = "";
 //
 serialMessage.concat(millis());
 serialMessage.concat(" ");
 serialMessage.concat(pt1);
 serialMessage.concat(" ");
 serialMessage.concat(pt2);
 serialMessage.concat(" ");
 serialMessage.concat(pt3);
 serialMessage.concat(" ");
 serialMessage.concat(pt4);
 serialMessage.concat(" ");
 serialMessage.concat(lc5);
 serialMessage.concat(" ");
 serialMessage.concat(lc6);
 serialMessage.concat(" ");
 serialMessage.concat(lc7);
 Serial.println(serialMessage);



}
