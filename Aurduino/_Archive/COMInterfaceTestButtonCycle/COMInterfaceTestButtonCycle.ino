// We assigned a name LED pin to pin number 22
const int LEDPIN = 32;
const int LEDPIN2 = 14;
const int LEDPIN3 = 22;
const int LEDPIN4 = 23;
const int LEDPIN5 = 25;
const int LEDPIN6 = 5;
const int LEDPIN7 = 4;
const int LEDPIN8 = 26;

// this will assign the name PushButton to pin numer 15
const int PushButton =16;
const int PushButton2 =19;
const int PushButton3 =17;
const int PushButton4 =21;

// This Setup function is used to initialize everything 
void setup()
{
// This statement will declare pin 22 as digital output 
pinMode(LEDPIN, OUTPUT);
pinMode(LEDPIN2, OUTPUT);
pinMode(LEDPIN3, OUTPUT);
pinMode(LEDPIN4, OUTPUT);
pinMode(LEDPIN5, OUTPUT);
pinMode(LEDPIN6, OUTPUT);
pinMode(LEDPIN7, OUTPUT);
pinMode(LEDPIN8, OUTPUT);
// This statement will declare pin 15 as digital input 
pinMode(PushButton, INPUT);
Serial.begin(115200);
}

void loop()

{
// digitalRead function stores the Push button state 
// in variable push_button_state
int Push_button_state = digitalRead(PushButton);
int Push_button_state2 = digitalRead(PushButton2);
int Push_button_state3 = digitalRead(PushButton3);
int Push_button_state4 = digitalRead(PushButton4);

// if condition checks if push button is pressed
// if pressed LED will turn on otherwise remain off 




digitalWrite(LEDPIN, HIGH);
delay(5000); 
digitalWrite(LEDPIN2, HIGH); 
//delay(50);
delay(5000);

digitalWrite(LEDPIN, LOW); 
digitalWrite(LEDPIN2, LOW); 

//delay(50);
//delay(5000);

digitalWrite(LEDPIN3, HIGH);
delay(5000); 
digitalWrite(LEDPIN4, HIGH); 
//delay(50);
delay(5000);

digitalWrite(LEDPIN3, LOW); 
digitalWrite(LEDPIN4, LOW); 
//delay(50);
//delay(5000);


digitalWrite(LEDPIN5, HIGH);
delay(5000); 
digitalWrite(LEDPIN6, HIGH); 
delay(5000);

digitalWrite(LEDPIN5, LOW); 
digitalWrite(LEDPIN6, LOW); 

//delay(50);
delay(5000);

digitalWrite(LEDPIN7, HIGH); 
digitalWrite(LEDPIN8, HIGH); 

//delay(50);
delay(5000);
digitalWrite(LEDPIN7, LOW); 
digitalWrite(LEDPIN8, LOW); 
//delay(50);
delay(5000);

}
