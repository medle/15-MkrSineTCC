/*
 * MkrSineChopper5.cpp
 * Arduino MKR Zero 16-bit Sine-wave chopper based on TimerCounter5.
 * Created: 03.05.2021 
 * Author: SL
 */ 
#include <Arduino.h>
#include <math.h>
#include "MkrSineChopper5.h"

#include "tc\tc.h" // ASF Timer driver
#include "tc\tc_interrupt.h"

// clock speed defined in command line options
#ifndef F_CPU
#define F_CPU 48000000
#endif

// global single instance
__MkrSineChopper5 MkrSineChopper5;

// TimerCounter5 16-bit resolution.
#define MAX_16BIT_PERIOD_VALUE 0xFFFF

#define MICROS_PER_SECOND 1000000

#define MAX_CHOPS_PER_HALF_CYCLE 10
static int _chopMatchValues[MAX_CHOPS_PER_HALF_CYCLE];
static volatile int _currentChopIndex;
static int _numChopsPerHalfCycle;

// ASF/TC driver assets
static bool _is_enabled = false;
static struct tc_config _config_tc;
static struct tc_module _tc_instance;

static tc_clock_prescaler _prescaler;
static uint32_t _periodValue;

static void computePrescalerAndPeriodValue(int periodMicros);
static void computeChopMatchValues(int chopsPerCycle);

// attached user callback routine
static void (*_halfCycleUserCallback)();
static void _tc5_isr(tc_module *const module)
{
  // compute next chop index
  if(++_currentChopIndex == _numChopsPerHalfCycle) {
    _currentChopIndex = 0;
  }

  // set match value for the next chop
  tc_set_compare_value(&_tc_instance, TC_COMPARE_CAPTURE_CHANNEL_1, _chopMatchValues[_currentChopIndex]);

  // call user callback if new half cycle begins
  if(_currentChopIndex == 0 && _halfCycleUserCallback != NULL) {
    _halfCycleUserCallback();
  }
}

void __MkrSineChopper5::start(int cyclesPerSecond, int chopsPerCycle, void (*halfCycleCallback)())
{
  stop();

  if(cyclesPerSecond < 1 || chopsPerCycle < 2) return;

  _halfCycleUserCallback = halfCycleCallback;

  int chopsPerSecond = cyclesPerSecond * chopsPerCycle;
  int microsPerChop = convertHertzToMicroseconds(chopsPerSecond);  
  int periodMicros = microsPerChop;

  computePrescalerAndPeriodValue(periodMicros);
  computeChopMatchValues(chopsPerCycle);  

  // load default configuration values
  tc_get_config_defaults(&_config_tc);
  
  _config_tc.clock_prescaler = _prescaler;
  _config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
  _config_tc.wave_generation = TC_WAVE_GENERATION_MATCH_FREQ;
  _config_tc.count_direction = TC_COUNT_DIRECTION_UP;

  // configure PWM pin, use channel 1 hard-wired with PB11 pin 
  uint16_t firstMatchValue = _chopMatchValues[0];
  int pwm_channum = TC_COMPARE_CAPTURE_CHANNEL_1;
  _config_tc.pwm_channel[pwm_channum].enabled = true;
  _config_tc.pwm_channel[pwm_channum].pin_out = PIN_PB11E_TC5_WO1; // PB11 => D5~ (on MKR Zero)
  _config_tc.pwm_channel[pwm_channum].pin_mux = MUX_PB11E_TC5_WO1; 

  // configure PWM period value and duty cycle value
  _config_tc.counter_16_bit.compare_capture_channel[TC_COMPARE_CAPTURE_CHANNEL_0] = _periodValue;
  _config_tc.counter_16_bit.compare_capture_channel[TC_COMPARE_CAPTURE_CHANNEL_1] = firstMatchValue;

  // initialize TimerCounter5
  tc_init(&_tc_instance, TC5, &_config_tc);

  // maybe attach a user callback routine
  tc_callback cb_type = TC_CALLBACK_CC_CHANNEL0; // call ISR for each end of period
  tc_register_callback(&_tc_instance, _tc5_isr, cb_type);
  tc_enable_callback(&_tc_instance, cb_type);
    
  // start timer 
  _is_enabled = true;
  tc_enable(&_tc_instance);
}

void __MkrSineChopper5::stop()
{
  if(_is_enabled) {
    _is_enabled = false;
    tc_disable(&_tc_instance);  

    _halfCycleUserCallback = NULL;
    tc_callback cb_type = TC_CALLBACK_CC_CHANNEL0;
    tc_disable_callback(&_tc_instance, cb_type);
    tc_unregister_callback(&_tc_instance, cb_type);
  }    
}

static void computePrescalerAndPeriodValue(int periodMicros)
{
  // choose the prescaler to slow down the timer when period value is greater than max period value
  _periodValue = (F_CPU / MICROS_PER_SECOND) * periodMicros;
  if(_periodValue <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV1;
  else if((_periodValue >>= 1) <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV2;
  else if((_periodValue >>= 1) <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV4;
  else if((_periodValue >>= 1) <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV8;
  else if((_periodValue >>= 1) <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV16;
  else if((_periodValue >>= 2) <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV64;
  else if((_periodValue >>= 2) <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV256;
  else if((_periodValue >>= 2) <= MAX_16BIT_PERIOD_VALUE) _prescaler = TC_CLOCK_PRESCALER_DIV1024;
}

static void computeChopMatchValues(int chopsPerCycle)
{
  _numChopsPerHalfCycle = chopsPerCycle / 2;
  float chopCx = PI / _numChopsPerHalfCycle; // angle of one chop

  for(int i = 0; i < _numChopsPerHalfCycle; i++) {

    float x1 = i * chopCx; // angle where chop begins
    float x2 = x1 + chopCx; // angle where chop ends
    float chopS = cos(x1) - cos(x2); // square of chop sine graph between x1 and x2
    float fillFactor = chopS / chopCx; // divide sine wave chop square by 100% chop square

    int matchValue = (int)(_periodValue * fillFactor);
    _chopMatchValues[i] = matchValue;
  }
}

int __MkrSineChopper5::convertHertzToMicroseconds(int hertz)
{
  if(hertz <= 0) return 0;
  return (MICROS_PER_SECOND / hertz);
}

int __MkrSineChopper5::getPrescaler() 
{ 
  return _prescaler; 
}

int __MkrSineChopper5::getPeriod() 
{ 
  return _periodValue; 
}

void __MkrSineChopper5::printValues()
{
  for(int i=0; i<_numChopsPerHalfCycle; i++) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print("=");
    Serial.print((float)_chopMatchValues[i] * 100 / _periodValue);
    Serial.print("%");
  }

  Serial.print(" chopClocks=");
  Serial.print(_periodValue);
  //Serial.print(" prescaler=");
  //Serial.print(_prescaler);
}

/*
 * End of MkrSineChopper5.cpp
 */
