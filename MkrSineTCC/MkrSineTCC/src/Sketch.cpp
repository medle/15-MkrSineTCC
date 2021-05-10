
#include <Arduino.h>
#include "MkrSineChopperTcc.h"
#include "MkrUtil.h"

extern "C" void _system_events_init();

static void restartChopper()
{
  MkrSineChopperTcc.stop();
  int hz = 7000;
  int chops = 10;
  panicIf(MkrSineChopperTcc.start(hz, chops));
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  SerialUSB.begin(115200);
  delay(1000); // let USB setup finish racing interrupts

  // initialize ASF events driver
  _system_events_init();
  
  restartChopper();
}

// the loop function runs over and over again forever
void loop() 
{
  blink(2, 100);
  delay(800);
  
  static int n = 0;  
  Serial.print(++n);
  Serial.print(": ");
  MkrSineChopperTcc.printValues();
  Serial.println("");
  
  static int reset_counter = 0;
  if(++reset_counter == 5) {
    restartChopper();
    reset_counter = 0;
  }
}



