
#include <Arduino.h>
#include "MkrSineChopperTcc.h"
#include "MkrUtil.h"

extern "C" void _system_events_init();

extern int _handler_count;

// the setup function runs once when you press reset or power the board
void setup() 
{
  SerialUSB.begin(115200);
  delay(1000); // let USB setup finish racing interrupts

  // initialize ASF events driver
  _system_events_init();

  int hz = 7000;
  int chops = 10;  
  MkrSineChopperTcc.start(hz, chops);
}

// the loop function runs over and over again forever
void loop() 
{
  blink(2, 100);
  delay(800);
  
  static int n = 0;  
  Serial.print(++n);
  Serial.print(": handlers=");
  Serial.print(_handler_count);
  //MkrSineChopperTcc.printValues();
  Serial.println("");
}



