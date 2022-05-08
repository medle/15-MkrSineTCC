/*
 * MkrSineChopperTcc0.h
 *
 * Created: 04.05.2021 19:02:38
 * Author: SL
 */ 

#ifndef MKRSINECHOPPERTCC0_H_
#define MKRSINECHOPPERTCC0_H_

#include <Arduino.h>

// Sine-wave invertor output pins on ARDUINO MKR ZERO:
// D2: left high-side signal
// D3: right high-side signal
// D11: low-side signal (both left and right)
class __MkrSineChopperTcc {
  public:
    int start(int cyclesPerSecond, int chopsPerHalfCycle, 
      int dutyCycle1024 = 512, void (*cycleEndCallback)() = 0);
    void stop();
    void printValues();
};

extern __MkrSineChopperTcc MkrSineChopperTcc;

#endif /* MKRSINECHOPPERTCC0_H_ */