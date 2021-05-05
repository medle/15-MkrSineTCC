/*
 * MkrSineChopperTcc0.cpp
 *
 * Created: 04.05.2021 19:06:43
 * Author: SL
 */ 

#include <Arduino.h>
#include <math.h>
#include "tcc\tcc.h"
#include "tcc\tcc_callback.h"
#include "MkrSineChopperTcc.h"

// clock speed defined in command line options
#ifndef F_CPU
#define F_CPU 48000000
#endif

// global single instance
__MkrSineChopperTcc MkrSineChopperTcc;

// local single TCC module 
static struct tcc_module _tcc0;
static struct tcc_module _tcc2;
static bool _isEnabled = false;

// TOP value used for double slope counting
uint32_t _chopTopValue;

// table of precomputed match values used as variable duty-cycle sequence
#define MAX_CHOPS_PER_HALF_CYCLE 10
static uint32_t _chopMatchValues[MAX_CHOPS_PER_HALF_CYCLE];
static volatile int _currentChopIndex;
static int _numChopsPerHalfCycle;

static void changeDutyCycleCallback(struct tcc_module *const tcc);
static void computeChopMatchValues(int chopsPerCycle);

static void configure_tcc0()
{
  _currentChopIndex = 0;
  uint32_t firstMatchValue = _chopMatchValues[_currentChopIndex];

  // double slope operation to make pulse at the center of the chop:
  // count up from zero to top, then down to zero,
  // output active when counter is above "match value",
  // trigger callback at bottom
  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, TCC0);
  //config_tcc.double_buffering_enabled = false;
  config_tcc.counter.period = _chopTopValue;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTTOM;
  config_tcc.compare.match[0] = firstMatchValue;
  config_tcc.pins.enable_wave_out_pin[0] = true;
  config_tcc.pins.wave_out_pin[0]        = PIN_PA08E_TCC0_WO0; // D11 on MKR Zero 
  config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA08E_TCC0_WO0;
  tcc_init(&_tcc0, TCC0, &config_tcc);

  tcc_register_callback(&_tcc0, changeDutyCycleCallback, (enum tcc_callback)0);
  tcc_enable_callback(&_tcc0, (enum tcc_callback)0);
}

static void configure_tcc2()
{
  // single slope PWM output is active when counter is above "match value"
  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, TCC2);
  config_tcc.counter.period = _chopTopValue * 2 * _numChopsPerHalfCycle * 2 - 1;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
  config_tcc.compare.match[0] = _chopTopValue * 2 * _numChopsPerHalfCycle;
  config_tcc.pins.enable_wave_out_pin[0] = true;
  config_tcc.pins.wave_out_pin[0]        = PIN_PA16E_TCC2_WO0; // D8 on MKR Zero
  config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA16E_TCC2_WO0;
  tcc_init(&_tcc2, TCC2, &config_tcc);
}

void __MkrSineChopperTcc::start(int cyclesPerSecond, int chopsPerCycle)
{
  if(cyclesPerSecond < 1 || chopsPerCycle < 2) return;
  if(_isEnabled) stop();

  // compute the period for TCC as clocks per chop / 2 for double slope counting
  uint32_t clocksPerCycle = F_CPU / cyclesPerSecond;
  _chopTopValue = clocksPerCycle / (chopsPerCycle * 2);
  computeChopMatchValues(chopsPerCycle);
  
  configure_tcc0();
  configure_tcc2();
  
  tcc_enable(&_tcc0);
  tcc_enable(&_tcc2);
  
  _isEnabled = true;
}

void __MkrSineChopperTcc::stop()
{
  if(_isEnabled) {
    _isEnabled = false;
    
    tcc_disable(&_tcc0);
    tcc_disable(&_tcc2);
   
    tcc_callback cb_type = (tcc_callback)0;
    tcc_disable_callback(&_tcc0, cb_type);
    tcc_unregister_callback(&_tcc0, cb_type);
  }
}

int _handler_count = 0;

static void changeDutyCycleCallback(struct tcc_module *const tcc)
{
  _handler_count += 1;
  
  // compute next chop index
  if(++_currentChopIndex == _numChopsPerHalfCycle) {
    _currentChopIndex = 0;
  }

  // set match value for the next chop
  tcc_set_compare_value(
    &_tcc0,
    (enum tcc_match_capture_channel)0,
    _chopMatchValues[_currentChopIndex]);
}

static void computeChopMatchValues(int chopsPerCycle)
{
  _numChopsPerHalfCycle = chopsPerCycle / 2;
  float chopCx = PI / _numChopsPerHalfCycle; // angle of one chop in radians

  for(int i = 0; i < _numChopsPerHalfCycle; i++) {

    float x1 = i * chopCx; // angle where chop begins
    float x2 = x1 + chopCx; // angle where chop ends
    float chopS = cos(x1) - cos(x2); // square of chop sine graph between x1 and x2
    float fillFactor = 1 - chopS / chopCx; // divide sine wave chop square by 100% chop square

    int matchValue = (int)(_chopTopValue * fillFactor);
    _chopMatchValues[i] = matchValue;
  }
}

void __MkrSineChopperTcc::printValues()
{
  for(int i=0; i<_numChopsPerHalfCycle; i++) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print("=");
    Serial.print((float)(100 - _chopMatchValues[i] * 100 / _chopTopValue));
    Serial.print("%");
  }

  Serial.print(" TOP=");
  Serial.print(_chopTopValue);
}

