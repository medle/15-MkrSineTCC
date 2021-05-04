/*
 * MkeSineChopper5.h
 * Arduino MKR Zero 16-bit Sine-wave chopper based on Timer/Counter5.
 * Created: 03.05.2021
 * Author: SL
 */ 
#ifndef MKRSINECHOPPER5_H
#define MKRSINECHOPPER5_H

#include <Arduino.h>

// Arduino MKR Zero Timer5 outputs PWM to D5~ pin.
#define MKRTIMER5_PIN D5

class __MkrSineChopper5 {
    public:
      int convertHertzToMicroseconds(int hertz);
      void start(int cyclesPerSecond, int chopsPerPhase, void (*halfPhaseCallback)() = 0);
      void stop();
      int getPrescaler();
      int getPeriod();
      void printValues();
};

extern __MkrSineChopper5 MkrSineChopper5;

#endif /* MKRSINECHOPPER5_H */
