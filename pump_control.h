#ifndef PUMP_CONTROL_H_INCLUDED
  #define PUMP_CONTROL_H_INCLUDED
  
  #include <Arduino.h> 
  #include "io_util.h"
  
// --------------------
// CONFIGURABLE VALUES
//
// Voltages are always in [mV].
// Pressures are always in [mBar].
// Durations are always in [ms].
// --------------------

const millivolt_t PUMP_MAX_VOLTAGE           = 6000;   // [mV]
const millivolt_t PUMP_LOW_THRESHOLD_VOLTAGE =  500;   // [mV] // below this voltage, the pump will not rotate
const millibar_t  PUMP_MAX_PRESSURE          = 4900;   // [mBar]

// Pump soft start and stop:
const millivolt_t PUMP_START_DURATION = 4000;  // [ms] duration from full stop to full throttle
const millivolt_t PUMP_STOP_DURATION  = 1000;   // [ms] duration from full throttle to full stop

// Control cycle: output values are set only once per cycle
const duration32_ms_t CONTROL_CYCLE_DURATION = 200; // [ms]
const duration32_ms_t MIN_PUMP_STATE_PERSISTENCE = 200; // milliseconds


// --------------------
// DO NOT TOUCH THE VALUES OF THE FOLLOWING CONSTANTS
// --------------------

const duty_value_t PUMP_OUT_LOW_THRESHOLD = (uint32_t) ANALOG_OUT_MAX * PUMP_LOW_THRESHOLD_VOLTAGE /  PUMP_MAX_VOLTAGE;

typedef enum {MODE_OFF, MODE_ON} PumpMode;

PumpMode pumpMode();

typedef enum {
  PUMP_OFF,          // mode == MODE_OFF
  PUMP_STARTING,     // mode == MODE_ON
  PUMP_ON,           // mode == MODE_ON
  PUMP_SPEEDING_UP,  // mode == MODE_ON
  PUMP_SLOWING_DOWN, // mode == MODE_ON
  PUMP_STOPPING      // mode == MODE_ON
} PumpState;

PumpState pumpState();

#ifdef VERBOSE
  char* pumpStateName();
#endif

/*
 * If the function returns true, then the new target values are stored in pumpTargetDutyValue, however, pumpActualDutyValue remains unchanged! 
 */
bool upddateTargetWaterFlow(analog_read_t targetFlow); 

void toggleMode(uint32_t now);

/*
 * A new targetDutyValue has been set in state PUMP_ON
 */
void adaptActualWaterFlow();

void handlePumpStateTransition();

void setStatusLED(boolean value);
void invertStatusLED();

// #### FOR TESTING PURPOSES ONLY ###
void setPumpDutyValue(duty_value_t value);

#endif
