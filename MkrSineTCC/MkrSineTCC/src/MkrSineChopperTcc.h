/*
 * MkrSineChopperTcc0.h
 *
 * Created: 04.05.2021 19:02:38
 * Author: SL
 */ 

#ifndef MKRSINECHOPPERTCC0_H_
#define MKRSINECHOPPERTCC0_H_

#include <Arduino.h>

class __MkrSineChopperTcc {
  public:
    void start(int cyclesPerSecond, int chopsPerPhase);
    void stop();
    void printValues();
};

extern __MkrSineChopperTcc MkrSineChopperTcc;

#endif /* MKRSINECHOPPERTCC0_H_ */