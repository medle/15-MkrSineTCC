
#include <Arduino.h>
#include "MkrSineChopperTcc.h"
#include "MkrUtil.h"

extern "C" void _system_events_init();

static void restartChopper()
{
  MkrSineChopperTcc.stop();
  int hz = 5000;
  int chopsPerCycle = 2;
  int percentage = 10;
  panicIf(MkrSineChopperTcc.start(hz, chopsPerCycle, percentage));
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
  static int reset_counter = 1;
  blink(reset_counter, 200);
  delay(1000);
  
  /*
  static int uh = 0;
  uh = (uh == 0 ? 1 : 0);
  pinMode(8, OUTPUT);
  digitalWrite(8, uh);
  static int uh2 = 0;
  uh2 += 1;
  pinMode(2, OUTPUT);
  digitalWrite(2, (uh2 >> 1) & 1);
  pinMode(3, OUTPUT);
  digitalWrite(3, (uh2 >> 1) & 1);
  */
  
  static int n = 0;  
  Serial.print(++n);
  Serial.print(": ");
  MkrSineChopperTcc.printValues();
  Serial.println("");
  
  if(++reset_counter > 3) {
    restartChopper();
    reset_counter = 1;
  }
}



