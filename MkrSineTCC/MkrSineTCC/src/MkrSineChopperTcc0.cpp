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
#include "MkrSineChopperTcc0.h"

// clock speed defined in command line options
#ifndef F_CPU
#define F_CPU 48000000
#endif

// global single instance
__MkrSineChopperTcc0 MkrSineChopperTcc0;

// TCC module configuration 
#define CONF_PWM_MODULE  TCC0
#define CONF_PWM_CHANNEL 0
#define CONF_PWM_OUTPUT  0
#define CONF_PWM_OUT_PIN PIN_PA08E_TCC0_WO0  // on Arduino MkrZero it's pin D11 
#define CONF_PWM_OUT_MUX MUX_PA08E_TCC0_WO0  // (see samd21g18a.h for other possible values)

// local single TCC module 
static struct tcc_module _tcc;
static bool _isEnabled = false;

// TOP value used for double slope counting
uint32_t _topValue;

// table of precomputed match values used as variable duty-cycle sequence
#define MAX_CHOPS_PER_HALF_CYCLE 10
static uint32_t _chopMatchValues[MAX_CHOPS_PER_HALF_CYCLE];
static volatile int _currentChopIndex;
static int _numChopsPerHalfCycle;

static void changeDutyCycleCallback(struct tcc_module *const tcc);
static void computeChopMatchValues(int chopsPerCycle);

// attached user callback routine
static void (*_halfCycleUserCallback)();

void __MkrSineChopperTcc0::start(int cyclesPerSecond, int chopsPerCycle, void (*halfCycleCallback)())
{
  if(cyclesPerSecond < 1 || chopsPerCycle < 2) return;
  
  if(_isEnabled) stop();

  _halfCycleUserCallback = halfCycleCallback;

  // compute the period for TCC as clocks per chop / 2 for double slope counting
  uint32_t clocksPerCycle = F_CPU / cyclesPerSecond;
  _topValue = clocksPerCycle / (chopsPerCycle * 2);

  computeChopMatchValues(chopsPerCycle);
  uint32_t firstMatchValue = _chopMatchValues[0];

  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, CONF_PWM_MODULE);

  config_tcc.counter.period = _topValue;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTTOM;
  config_tcc.compare.match[CONF_PWM_CHANNEL] = firstMatchValue;

  config_tcc.pins.enable_wave_out_pin[CONF_PWM_OUTPUT] = true;
  config_tcc.pins.wave_out_pin[CONF_PWM_OUTPUT]        = CONF_PWM_OUT_PIN;
  config_tcc.pins.wave_out_pin_mux[CONF_PWM_OUTPUT]    = CONF_PWM_OUT_MUX;

  _currentChopIndex = (_numChopsPerHalfCycle - 1); 

  tcc_init(&_tcc, CONF_PWM_MODULE, &config_tcc);

  tcc_register_callback(&_tcc, changeDutyCycleCallback, (enum tcc_callback)(CONF_PWM_CHANNEL));
  tcc_enable_callback(&_tcc, (enum tcc_callback)(CONF_PWM_CHANNEL));
  
  tcc_enable(&_tcc);
  _isEnabled = true;  
}

void __MkrSineChopperTcc0::stop()
{
  if(_isEnabled) {
    _isEnabled = false;
    
    tcc_disable(&_tcc);
   
    tcc_callback cb_type = (tcc_callback)CONF_PWM_CHANNEL;
    tcc_disable_callback(&_tcc, cb_type);
    tcc_unregister_callback(&_tcc, cb_type);
    
    _halfCycleUserCallback = NULL;
  }
}

static void changeDutyCycleCallback(struct tcc_module *const tcc)
{
  // compute next chop index
  if(++_currentChopIndex == _numChopsPerHalfCycle) {
    _currentChopIndex = 0;
  }

  // set match value for the next chop
  tcc_set_compare_value(
    &_tcc,
    (enum tcc_match_capture_channel)CONF_PWM_CHANNEL,
    _chopMatchValues[_currentChopIndex]);

  // call user callback if new half cycle begins
  if(_currentChopIndex == 0 && _halfCycleUserCallback != NULL) {
    _halfCycleUserCallback();
  }
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

    int matchValue = (int)(_topValue * fillFactor);
    _chopMatchValues[i] = matchValue;
  }
}

void __MkrSineChopperTcc0::printValues()
{
  for(int i=0; i<_numChopsPerHalfCycle; i++) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print("=");
    Serial.print((float)(100 - _chopMatchValues[i] * 100 / _topValue));
    Serial.print("%");
  }

  Serial.print(" TOP=");
  Serial.print(_topValue);
}

