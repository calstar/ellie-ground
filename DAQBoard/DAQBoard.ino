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

//define pins to use for the various sensors and connetcions. define takes up less space on the chip 
#define ONBOARD_LED  12
#define PT1DOUT 26
#define PT2DOUT 44

#define CLK 25
#define FM 4

#define S1S 21

//define servo min and max values
#define SERVO_MIN_USEC (900)
#define SERVO_MAX_USEC (2100)

//Initialize flow meter variables for how it computes the flow ammount
float currentMillis = 0;
float goalTime = 100;        // [ms]
float currReading1;
float currReading2;


//FM counter
float fmcount;
float flowRate;
boolean currentState;
boolean lastState = false;


//Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;

//Initialize the servo objects
Servo servo1;
Servo servo2;

//define servo necessary values
int ADC_Max = 4096;   


///////////////
//IMPORTANT
//////////////
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C};


// Define variables to store readings to be sent
float pt1;
float pt2;
float pt3;
float pt4;
float pt5;
float lc1;
float lc2;
float lc3;
float fm;
//the following are only used in the oposite diretcion, they are included because it may be necessary for the structure to be the same in both directions
int S1; int S2; int S1S2; int I;




// Define variables to store incoming commands, servos and igniter
int incomingS1;
int incomingS2;
int incomingS1S2;
bool incomingI;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float pt1;
    float pt2;
    float pt3;
    float pt4;
    float pt5;
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
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Commands, incomingData, sizeof(Commands));
  Serial.print("Bytes received: ");
  Serial.println(len);
  S1 =Commands.S1;
  S2 = Commands.S2;
  S1S2 = Commands.S1S2;
  I = Commands.I;
  //Serial.print("IT IS WORKING");
}

 
void setup() {
  //attach servo pins
  servo1.attach(S1S,SERVO_MIN_USEC,SERVO_MAX_USEC );
  
  // attach onboard LED
  pinMode(ONBOARD_LED,OUTPUT);


//attach flowmeter pin
  pinMode(FM, INPUT_PULLUP);

//set gains for pt pins
  scale1.begin(PT1DOUT, CLK);
  scale1.set_gain(64);
  scale2.begin(PT2DOUT, CLK);
  scale2.set_gain(64);

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
  //Set LED back to low 
    digitalWrite(ONBOARD_LED,LOW);
 


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
    servo1.write(S1);
  //}

  getReadings();

 
  // Set values to send
  Readings.pt1 = pt1;
  Readings.pt2 = pt2;
  Readings.pt3 = pt3;
  Readings.pt4 = pt4;
  Readings.pt5 = pt5;
  Readings.lc1 = lc1;
  Readings.lc2 = lc2;
  Readings.lc3 = lc3;
  Readings.fm  = fm;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    digitalWrite(ONBOARD_LED,HIGH);
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(50);
}



void getReadings(){
  currentMillis = millis();
  fmcount = 0;
  while (millis() - currentMillis < goalTime) {
    currentState = digitalRead(FM);
    if (!(currentState == lastState)) {
      lastState = currentState;
      fmcount += 1;
    }
  }
  flowRate = fmcount * 1000 / goalTime;
  //if (goalTime < currentMillis) {
    
    //goalTime = currentMillis + 50;
    //flowRate=fmcount;
    //fmcount=0;
  //}

    //currentState = digitalRead(FM);
    //if (!(currentState == lastState)) {
      //lastState = currentState;
      //fmcount+=1;
    //}
  
  fm =int(flowRate*10000+1);  // Print the integer part of the variable

   pt1 = scale1.read();
   
   pt2 = scale2.read();

}
// hi! How r u?
