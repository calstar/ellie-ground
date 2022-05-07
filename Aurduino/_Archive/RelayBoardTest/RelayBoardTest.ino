// We assigned a name LED pin to pin number 22
const int RELAYPIN = 21; 
// this will assign the name PushButton to pin numer 15
const int PushButton =26;
// This Setup function is used to initialize everything 
void setup()
{
 pinMode(RELAYPIN, OUTPUT);

// This statement will declare pin 22 as digital output 
digitalWrite(RELAYPIN, HIGH); 
// This statement will declare pin 15 as digital input 
}

void loop()   {
digitalWrite(RELAYPIN, HIGH); 
delay(2000);
digitalWrite(RELAYPIN, LOW); 

delay(2000); 
}
