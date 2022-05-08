
#include <Arduino.h>
#include <system.h> // ASF core
#include "MkrSineChopperTcc.h"
#include "MkrUtil.h"

static int _numCycleEnds = 0;
static void atCycleEndCallback()
{
  _numCycleEnds += 1;
}

static void restartChopper()
{
  MkrSineChopperTcc.stop();
  int hz = 5000;
  int chopsPerHalfCycle = 0; // zero chops for pulsing mode
  int dutyCycle1024 = 1023 * 2 / 100;
  panicIf(MkrSineChopperTcc.start(
    hz, chopsPerHalfCycle, dutyCycle1024, atCycleEndCallback));
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  SerialUSB.begin(115200);
  delay(1000); // let USB setup finish racing interrupts

  // initialize ASF core including event system
  system_init();
  
  restartChopper();
}

// the loop function runs over and over again forever
void loop() 
{
  static int reset_counter = 1;
  blink(reset_counter, 200);
  delay(1000);
  
  static int n = 0;  
  Serial.print(++n);
  Serial.print(": cycleEnds=");
  Serial.print(_numCycleEnds);
  Serial.print(" ");
  MkrSineChopperTcc.printValues();
  Serial.println("");
  
  if(++reset_counter > 3) {
    restartChopper();
    reset_counter = 1;
  }
}



