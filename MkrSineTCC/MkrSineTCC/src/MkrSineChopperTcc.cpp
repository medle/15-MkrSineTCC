/*
 * MkrSineChopperTcc.cpp
 *
 * Created: 04.05.2021 19:06:43
 * Author: SL
 */ 

#include <Arduino.h>
#include <math.h>

#include "tcc\tcc.h"
#include "tcc\tcc_callback.h"
#include "events\events.h"

#include "MkrSineChopperTcc.h"
#include "MkrUtil.h"

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

// table of precomputed match values used as a sequence of varying duty-cycle values
#define MAX_CHOPS_PER_HALF_CYCLE 20
static uint32_t _chopMatchValues[MAX_CHOPS_PER_HALF_CYCLE];
static volatile int _currentChopIndex;
static int _numChopsPerHalfCycle;

#define TCC0_CHOP_CALLBACK_TYPE (enum tcc_callback)0

// local functions
static void changeChopDutyCycleCallback(struct tcc_module *const tcc);
static void precomputeChopMatchValues(int chopsPerCycle);
static void configureTCC0();
static void configureTCC2();
static void startTimersSimultaneously();

static int myError = 0;
static int last_status = 0;

void __MkrSineChopperTcc::start(int cyclesPerSecond, int chopsPerCycle)
{
  if(cyclesPerSecond < 1 || chopsPerCycle < 2) return;
  if(chopsPerCycle > MAX_CHOPS_PER_HALF_CYCLE * 2) return;
  
  if(_isEnabled) stop();

  // compute the period for TCC as clocks per chop / 2 for double slope counting
  uint32_t clocksPerCycle = F_CPU / cyclesPerSecond;
  _chopTopValue = clocksPerCycle / (chopsPerCycle * 2);
  precomputeChopMatchValues(chopsPerCycle);
 
  configureTCC0();
  configureTCC2();
  startTimersSimultaneously();

  // using this simple way timers will start not at the same time
  //tcc_enable(&_tcc0);
  //tcc_enable(&_tcc2);
  
  _isEnabled = true;
}

static void configureTCC0()
{
  _currentChopIndex = 0;
  uint32_t firstMatchValue = _chopMatchValues[_currentChopIndex];

  // double slope operation to make pulse at the center of the chop:
  // count up from zero to top, then down to bottom zero,
  // output active when counter is above "match value",
  // trigger callback at bottom
  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, TCC0);
  config_tcc.counter.period = _chopTopValue;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTTOM;
  config_tcc.compare.match[0] = firstMatchValue;
  config_tcc.pins.enable_wave_out_pin[0] = true;
  config_tcc.pins.wave_out_pin[0]        = PIN_PA08E_TCC0_WO0; // D11 on MKR Zero
  config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA08E_TCC0_WO0;
  panicIf(tcc_init(&_tcc0, TCC0, &config_tcc));

  tcc_register_callback(&_tcc0, changeChopDutyCycleCallback, TCC0_CHOP_CALLBACK_TYPE);
  tcc_enable_callback(&_tcc0, TCC0_CHOP_CALLBACK_TYPE);
}

static void configureTCC2()
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
  panicIf(tcc_init(&_tcc2, TCC2, &config_tcc));
}

static void startTimersSimultaneously()
{
  struct events_resource starter_event;
  struct events_config starter_config;
  events_get_config_defaults(&starter_config);
  panicIf(events_allocate(&starter_event, &starter_config));
  panicIf(events_attach_user(&starter_event, EVSYS_ID_USER_TCC0_EV_0));
  panicIf(events_attach_user(&starter_event, EVSYS_ID_USER_TCC2_EV_0));
  
  struct tcc_events events;
  memset(&events, 0, sizeof(events));
  events.on_input_event_perform_action[0] = true;
  events.input_config[0].modify_action = true;
  events.input_config[0].action = (tcc_event_action)TCC_EVENT0_ACTION_START;
  
  panicIf(tcc_enable_events(&_tcc0, &events));
  panicIf(tcc_enable_events(&_tcc2, &events));

  tcc_enable(&_tcc0);
  tcc_stop_counter(&_tcc0);
  tcc_set_count_value(&_tcc0, 0);

  tcc_enable(&_tcc2);
  tcc_stop_counter(&_tcc2);
  tcc_set_count_value(&_tcc2, 0);
  
  _currentChopIndex = 0;

  // trigger START event by software  
  while(events_is_busy(&starter_event)); 
  panicIf(events_trigger(&starter_event)); 

  // cleanup   
  while(events_is_busy(&starter_event)); 
  //tcc_disable_events(&_tcc0, &events); // may be done only when timer is not enabled
  //tcc_disable_events(&_tcc2, &events);
  panicIf(events_detach_user(&starter_event, EVSYS_ID_USER_TCC0_EV_0));
  panicIf(events_detach_user(&starter_event, EVSYS_ID_USER_TCC2_EV_0));
  panicIf(events_release(&starter_event));
}

void __MkrSineChopperTcc::stop()
{
  if(_isEnabled) {
    _isEnabled = false;
    
    tcc_disable(&_tcc2);
    tcc_disable(&_tcc0);
    
    tcc_disable_callback(&_tcc0, TCC0_CHOP_CALLBACK_TYPE);
    tcc_unregister_callback(&_tcc0, TCC0_CHOP_CALLBACK_TYPE);
  }
}

int _handler_count = 0;

// This callback is called by TCC0 module at the end of each chop period, after
// counter went up from zero to "top" and returned back down to "bottom" zero.
// NOTE: this handler is very time-sensitive so at the start of the MCU when USB
// setup interrupts are active this callback may miss a call or two. Thus it is 
// found useful to perform a blank "delay(1000)" at the start of the MCU to
// let USB stabilize and not compete for cycles with this handler.
static void changeChopDutyCycleCallback(struct tcc_module *const tcc)
{
  _handler_count += 1;
  
  // the current chop index advances
  if(++_currentChopIndex == _numChopsPerHalfCycle) {
    _currentChopIndex = 0;
  }

  // Because writing of "compare value" is done in a double-buffered mode
  // the value we will write now will get into compare register not 
  // immediately but only at the end of the current chop. So we choose
  // the match value not for the "current" chop but for the "next" chop. 
  // This double-buffering allows to have enough time for setting new compare 
  // value in "relatively slow" callback routine avoiding wave-distortion 
  // effects when writing occurs in "race condition" with the TCC counter.
  int nextIndex = _currentChopIndex + 1;
  if(nextIndex == _numChopsPerHalfCycle) nextIndex = 0;
  uint32_t nextMatchValue = _chopMatchValues[nextIndex];  
  tcc_set_compare_value(&_tcc0, (tcc_match_capture_channel)0, nextMatchValue);
}

// Writes an array of the "match" values for individual chops in a sequence of sine wave generation.
// The idea is as follows: for each chop we want the time when current is on be just such as to
// pass power equal in amount as a true sine wave generator.
static void precomputeChopMatchValues(int chopsPerCycle)
{
  _numChopsPerHalfCycle = chopsPerCycle / 2;
  float chopCx = PI / _numChopsPerHalfCycle; // angle of one chop in radians

  // special case when there is only one chop per half-cycle:
  // we make it a 100% square wave and not 64% as in sine-wave
  if(_numChopsPerHalfCycle == 1) {
    _chopMatchValues[0] = 0;
    return;
  }

  // As both half-cycles of wave are the same we recalculate only the first half-cycle,
  // the sign of the wave is handled by TCC2 "direction" signal.
  for(int i = 0; i < _numChopsPerHalfCycle; i++) {

    float x1 = i * chopCx; // angle where chop begins
    float x2 = x1 + chopCx; // angle where chop ends
    float chopS = cos(x1) - cos(x2); // square of chop sine graph between x1 and x2
    float fillFactor = chopS / chopCx; // divide sine wave chop square by 100% chop square

    // Output will be active when counter is above match, so fill factor should be inverted,
    // for example when fill factor is 60% match value should be 40% thus there will be 60% 
    // time counter will be above match value.
    int matchValue = (int)(_chopTopValue * (1 - fillFactor));
    _chopMatchValues[i] = matchValue;
  }
}

// Debug print method.
void __MkrSineChopperTcc::printValues()
{
  for(int i=0; i<_numChopsPerHalfCycle; i++) {
    Serial.print(" ");
    Serial.print(i);
    Serial.print("=");
    Serial.print((float)(100 - (_chopMatchValues[i] * 100 / _chopTopValue)));
    Serial.print("%");
  }

  Serial.print(" chopTOP=");
  Serial.print(_chopTopValue);
}

