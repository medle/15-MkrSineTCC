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

// local TCC modules used 
static struct tcc_module _tcc1;
static struct tcc_module _tcc2;
static bool _isEnabled = false;

// TOP value used for double slope counting
uint32_t _chopTopValue;

// table of precomputed match values used as a sequence of varying duty-cycle values
#define MAX_CHOPS_PER_HALF_CYCLE 20
static uint32_t _chopMatchValues[MAX_CHOPS_PER_HALF_CYCLE];
static volatile int _currentChopIndex;
static int _numChopsPerHalfCycle;
static int _chopperCallbackCounter = 0;

#define CHOP_CALLBACK_TYPE (enum tcc_callback)0

inline bool isChopCallbackUsed() { return (_numChopsPerHalfCycle > 1); }

// local functions
static void changeChopDutyCycleCallback(struct tcc_module *const tcc);
static void precomputeChopMatchValues(int cyclesPerSecond, int chopsPerCycle, int percentage);
static void configureTCC1();
static void configureTCC2forChopping();
static void configureTCC2forPulsing(int percentage);
static void startTimersSimultaneously();

int __MkrSineChopperTcc::start(int cyclesPerSecond, int chopsPerCycle, int percentage)
{
  if(cyclesPerSecond < 1 || chopsPerCycle < 2) return 1;
  if(chopsPerCycle > MAX_CHOPS_PER_HALF_CYCLE * 2) return 1;
  if(percentage < 0 || percentage > 100) return 1;
  
  if(_isEnabled) stop();

  precomputeChopMatchValues(cyclesPerSecond, chopsPerCycle, percentage);
 
  configureTCC1();
  if(chopsPerCycle == 2) configureTCC2forPulsing(percentage);
  else configureTCC2forChopping();
  startTimersSimultaneously();

  // using this simple way timers will start not at the same time
  //tcc_enable(&_tcc1);
  //tcc_enable(&_tcc2);
  
  _isEnabled = true;
  return 0;
}

static int getDeadTimeCpuCycles()
{
  return 10;  
}

// Configure 24-bit TCC1 as "high-side" LEFT and RIGHT signals.
static void configureTCC1()
{
  // above that match value there will be a signal in double slope operation
  uint32_t matchValue = getDeadTimeCpuCycles() / 2;
  uint32_t periodValue = _chopTopValue * _numChopsPerHalfCycle;
  
  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, TCC1);
  config_tcc.counter.period = periodValue;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTTOM;
  
  config_tcc.compare.match[0] = matchValue;
  config_tcc.pins.enable_wave_out_pin[0] = true;
  config_tcc.pins.wave_out_pin[0]        = PIN_PA10E_TCC1_WO0; // D2 on MKR ZERO
  config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA10E_TCC1_WO0;

  config_tcc.compare.match[1] = matchValue;
  config_tcc.pins.enable_wave_out_pin[1] = true;
  config_tcc.pins.wave_out_pin[1]        = PIN_PA11E_TCC1_WO1; // D3 on MKR ZERO
  config_tcc.pins.wave_out_pin_mux[1]    = MUX_PA11E_TCC1_WO1;

  // RAMP2 operation: in cycle A, odd channel output (_WO1) is disabled, and in cycle B, 
  // even channel output (_WO0) is disabled. The ramp cycle changes after each update.
  config_tcc.compare.wave_ramp = TCC_RAMP_RAMP2;
  
  panicIf(tcc_init(&_tcc1, TCC1, &config_tcc));
}

// Configure 16-bit TCC2 as "low-side" signal for single pulse with given duty-cycle.
static void configureTCC2forPulsing(int percentage)
{
  // single-slope frequency = F_CPU / (TOP + 1) so we need to subtract 
  // one cycle from TOP to get exact match of frequency with double-slope
  // operation of the second timer
  uint32_t period = (_chopTopValue * 2) - 1;
  uint32_t match = period * percentage / 100;

  // single-slope PWM mode with up counting: output active at start and cleared 
  // on compare match, so the pulse appears at the start of each cycle
  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, TCC2);
  config_tcc.counter.period = period;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

  config_tcc.compare.match[0] = match;
  config_tcc.pins.enable_wave_out_pin[0] = true;
  config_tcc.pins.wave_out_pin[0]        = PIN_PA16E_TCC2_WO0; // D8 on MKR ZERO
  config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA16E_TCC2_WO0;
  
  panicIf(tcc_init(&_tcc2, TCC2, &config_tcc));
}

// Configure 16-bit TCC2 as "low-side" signal for chopping with variable duty-cycle.
static void configureTCC2forChopping()
{
  _currentChopIndex = 0;
  uint32_t firstMatchValue = _chopMatchValues[_currentChopIndex];

  // double slope operation to make pulse at the center of the chop:
  // count up from zero to top, then down to bottom zero,
  // output active when counter is above "match value",
  // trigger callback at bottom
  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, TCC2);
  config_tcc.counter.period = _chopTopValue;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTTOM;
  
  config_tcc.compare.match[0] = firstMatchValue;
  config_tcc.pins.enable_wave_out_pin[0] = true;
  config_tcc.pins.wave_out_pin[0]        = PIN_PA16E_TCC2_WO0; // D8 on MKR ZERO
  config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA16E_TCC2_WO0;
  
  panicIf(tcc_init(&_tcc2, TCC2, &config_tcc));

  if(isChopCallbackUsed()) {
    panicIf(tcc_register_callback(&_tcc2, changeChopDutyCycleCallback, CHOP_CALLBACK_TYPE));
    tcc_enable_callback(&_tcc2, CHOP_CALLBACK_TYPE);
  }  
}

// Start the two timers from the same clock using MCU event system.
static void startTimersSimultaneously()
{
  struct events_resource eventResource;
  struct events_config eventResourceConfig;
  events_get_config_defaults(&eventResourceConfig);
  panicIf(events_allocate(&eventResource, &eventResourceConfig));
  panicIf(events_attach_user(&eventResource, EVSYS_ID_USER_TCC1_EV_0));
  panicIf(events_attach_user(&eventResource, EVSYS_ID_USER_TCC2_EV_0));
  
  struct tcc_events eventActionConfig;
  memset(&eventActionConfig, 0, sizeof(eventActionConfig));
  eventActionConfig.on_input_event_perform_action[0] = true;
  eventActionConfig.input_config[0].modify_action = true;
  eventActionConfig.input_config[0].action = (tcc_event_action)TCC_EVENT0_ACTION_START;
  
  panicIf(tcc_enable_events(&_tcc1, &eventActionConfig));
  panicIf(tcc_enable_events(&_tcc2, &eventActionConfig));

  tcc_enable(&_tcc1);
  tcc_stop_counter(&_tcc1);
  tcc_set_count_value(&_tcc1, 0);

  tcc_enable(&_tcc2);
  tcc_stop_counter(&_tcc2);
  tcc_set_count_value(&_tcc2, 0);
  
  _currentChopIndex = 0;

  // trigger the ACTION_START event by software  
  while(events_is_busy(&eventResource)); 
  panicIf(events_trigger(&eventResource)); 

  // cleanup   
  while(events_is_busy(&eventResource)); 
  //tcc_disable_events(&_tcc1, &events); // may be done only when timer is not enabled,
  //tcc_disable_events(&_tcc2, &events); // tcc_reset() at stop() will do it anyway
  panicIf(events_detach_user(&eventResource, EVSYS_ID_USER_TCC1_EV_0));
  panicIf(events_detach_user(&eventResource, EVSYS_ID_USER_TCC2_EV_0));
  panicIf(events_release(&eventResource));
}

void __MkrSineChopperTcc::stop()
{
  if(_isEnabled) {
    _isEnabled = false;
    
    tcc_reset(&_tcc1);
    tcc_reset(&_tcc2);
    
    if(isChopCallbackUsed()) {
      tcc_disable_callback(&_tcc2, CHOP_CALLBACK_TYPE);
      tcc_unregister_callback(&_tcc2, CHOP_CALLBACK_TYPE);
    }    
  }
}

// This callback is called by TCC2 module at the end of each chop period, after
// counter went up from zero to "top" and returned back down to "bottom" zero.
// NOTE: this handler is very time-sensitive so at the start of the MCU when USB
// setup interrupts are active this callback may miss a call or two. Thus it is 
// found useful to perform a blank "delay(1000)" at the start of the MCU to
// let USB stabilize and not compete for cycles with this handler.
static void changeChopDutyCycleCallback(struct tcc_module *const tcc)
{
  // for debugging only
  _chopperCallbackCounter += 1;
  
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
  tcc_set_compare_value(&_tcc2, (tcc_match_capture_channel)0, nextMatchValue);
}

// Writes an array of the "match" values for individual chops in a sequence of sine wave generation.
// The idea is as follows: for each chop we want the time when current is on be just such as to
// pass power equal in amount as a true sine wave generator.
static void precomputeChopMatchValues(int cyclesPerSecond, int chopsPerCycle, int percentage)
{
  // compute the period for TCC as clocks per chop / 2 for double slope counting
  uint32_t clocksPerCycle = (F_CPU / cyclesPerSecond);
  _chopTopValue = clocksPerCycle / (chopsPerCycle * 2);
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
    if(percentage != 100) fillFactor = fillFactor * percentage / 100;

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
  Serial.print("chop=");
  Serial.print(_chopperCallbackCounter);
  Serial.print(" ");
  
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

// Configure 24-bit TCC0 as a simple square wave "direction" signal for the full-bridge.
// There appears to be a difficulty with TCC0 peripheral: I can't make it output
// anything into any channel other than _WO0 - signal just doesn't go to _WO1-_WO7.
/*
static void configureTCC0()
{
  uint32_t matchValue = _chopTopValue * 2 * _numChopsPerHalfCycle;
  uint32_t periodValue = (matchValue * 2) - 1; // minus one works by reason unclear
  
  // single slope PWM output is active when counter is above "match value"
  struct tcc_config config_tcc;
  tcc_get_config_defaults(&config_tcc, TCC0);
  config_tcc.counter.period = periodValue;
  config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
  
  config_tcc.compare.match[0] = matchValue;
  config_tcc.pins.enable_wave_out_pin[0] = true;
  config_tcc.pins.wave_out_pin[0]        = PIN_PA08E_TCC0_WO0; // D11 on MKR-ZERO
  config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA08E_TCC0_WO0;
  
  panicIf(tcc_init(&_tcc0, TCC0, &config_tcc));
}
*/
