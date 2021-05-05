
#include <Arduino.h>
#include "MkrSineChopperTcc.h"

extern int _handler_count;

// the setup function runs once when you press reset or power the board
void setup() 
{
  SerialUSB.begin(115200);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  int hz = 1000;
  int chops = 6;  
  MkrSineChopperTcc.start(hz, chops);
}

// the loop function runs over and over again forever
void loop() 
{
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second

  static int n = 0;  
  Serial.print(++n);
  Serial.print(": handlers=");
  Serial.print(_handler_count);
  MkrSineChopperTcc.printValues();
  Serial.println("");
}



