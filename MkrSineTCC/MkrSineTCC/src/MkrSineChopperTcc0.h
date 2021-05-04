/*
 * MkrSineChopperTcc0.h
 *
 * Created: 04.05.2021 19:02:38
 * Author: SL
 */ 

#ifndef MKRSINECHOPPERTCC0_H_
#define MKRSINECHOPPERTCC0_H_

#include <Arduino.h>

// Arduino MKR Zero TCC0 WO0 outputs to D11 pin.

class __MkrSineChopperTcc0 {
  public:
    void start(int cyclesPerSecond, int chopsPerPhase, void (*halfPhaseCallback)() = 0);
    void stop();
    void printValues();
};

extern __MkrSineChopperTcc0 MkrSineChopperTcc0;

#endif /* MKRSINECHOPPERTCC0_H_ */