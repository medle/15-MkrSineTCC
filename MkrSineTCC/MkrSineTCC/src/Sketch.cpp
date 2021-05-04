
#include <Arduino.h>
#include "MkrSineChopperTcc0.h"

static int _handler_count = 0;

#define PHASE_PIN 1

static void userCallback()
{
  _handler_count += 1;
  static bool b = 0;
  digitalWrite(PHASE_PIN, b = !b);
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  SerialUSB.begin(115200);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PHASE_PIN, OUTPUT);

  int hz = 1000;
  int chops = 10;  
  MkrSineChopperTcc0.start(hz, chops, userCallback);
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
  MkrSineChopperTcc0.printValues();
  Serial.println("");
}



