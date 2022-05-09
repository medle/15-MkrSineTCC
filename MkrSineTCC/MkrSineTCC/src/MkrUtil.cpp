/*
 * MkrUtil.cpp
 *
 * Created: 07.05.2021 14:25:38
 * Author: SL
 */ 

#include <Arduino.h>
#include "MkrUtil.h"

// clock speed defined in command line options
#ifndef F_CPU
#define F_CPU 48000000
#endif

#define MICROS_PER_SECOND 1000000

int convertHertzToCycleMicroseconds(int hertz)
{
  if(hertz <= 0) return 0;
  return (MICROS_PER_SECOND / hertz);
}

int convertCycleMicrosecondsToClocksPerCycle(int cycleMicroseconds)
{
  return (F_CPU / MICROS_PER_SECOND) * cycleMicroseconds;
}

void blink(int numBlinks, int msDelayEach)
{
  pinMode(LED_BUILTIN, OUTPUT);
  
  if(msDelayEach <= 0) msDelayEach = 100;
  
  bool high = false;
  for(int i = 0; i < numBlinks * 2; i++) {
    digitalWrite(LED_BUILTIN, high = !high);
    delay(msDelayEach / 2);
  }  
}

void panicAt(int code, const char *file, int line)
{
  for(;;) {
    blink(10, 80); // 10 blinks 80ms each
        
    delay(200);
    Serial.print("Panic Code=");
    Serial.print(code);
    Serial.print(" Line=");
    Serial.print(line);
    Serial.print(" [");
    Serial.print(file);
    Serial.println("]");
  }  
}
