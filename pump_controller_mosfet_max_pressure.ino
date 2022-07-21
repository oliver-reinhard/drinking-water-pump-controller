
#include <avr/io.h>
#include <util/delay.h>
#include "io_util.h"
#include "pins.h"
#include "pump_control.h"

#define DEBUG_STATES
#define INPUT_OUTPUT_TEST_MODE

// --------------------
// CONFIGURABLE VALUES
//
// Voltages are always in [mV].
// Pressures are always in [mBar].
// Durations are always in [ms].
// --------------------


const millibar_t PRESSURE_SENSOR_MIN_PRESSURE = MILLIBAR_PER_PSI *  0 /*psi*/;
const millibar_t PRESSURE_SENSOR_MAX_PRESSURE = MILLIBAR_PER_PSI * 80 /*psi*/;

const millibar_t PRESSURE_CHANGE_MIN_DELTA = 30; // Only turn on the pump if there is a significant pressure drop

const millivolt_t PRESSURE_SENSOR_MIN_VOLTAGE = 457;    // corresponds to PRESSURE_SENSOR_MIN_PRESSURE
const millivolt_t PRESSURE_SENSOR_MAX_VOLTAGE = 4500;   // corresponds to PRESSURE_SENSOR_MAX_PRESSURE


//
// CONTROLLER STATES
//

typedef enum {MODE_CONTINUOUS, MODE_PRESSURE_DRIVEN, MODE_ERROR} ControllerMode;

ControllerMode controllerMode;

typedef enum {
    STATE_TRANSITIONING,  // mode == MODE_CONTINUOUS or MODE_PRESSURE_DRIVEN
    STATE_CONTINUOUS,     // mode == MODE_CONTINUOUS
    STATE_PRESSURE_LOW,   // mode == MODE_PRESSURE_DRIVEN
    STATE_PRESSURE_UP,    // mode == MODE_PRESSURE_DRIVEN
    STATE_ERROR           // mode == MODE_ERROR
} ControllerState;

ControllerState controllerState;

#ifdef VERBOSE
  char* controllerStateName() {
    switch (controllerState) {
      case STATE_TRANSITIONING: return "TRANSITIONING";
      case STATE_CONTINUOUS: return "CONTINUOUS";
      case STATE_PRESSURE_LOW: return "PRESSURE_LOW";
      case STATE_PRESSURE_UP: return "PRESSURE_UP";
      case STATE_ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }
#endif 

//
// DIGITAL IN
//

// ** Controller Mode **
ControllerMode readMode() {
  return digitalRead(OPERATION_MODE_IN_PIN) ? MODE_PRESSURE_DRIVEN : MODE_CONTINUOUS;
}

void updateControllerModeAndState() {
  controllerMode = readMode();
  controllerState = controllerMode == MODE_CONTINUOUS ? STATE_CONTINUOUS : STATE_PRESSURE_LOW;
  
  #ifdef VERBOSE
    Serial.print("Mode = ");
    Serial.println(controllerMode == MODE_CONTINUOUS ? "CONTINUOUS" : "PRESSURE-DRIVEN");
    Serial.print("State = ");
    Serial.println(controllerStateName());
  #endif
  
  if (controllerMode == MODE_CONTINUOUS) {
    upddateTargetWaterFlow(ANALOG_IN_MAX);
  } else {
    upddateTargetWaterFlow(readTargetWaterFlowRaw());
  }
}

//
// ANALOG IN
//

// ** Target Pressure **
const millibar_t TARGET_PRESSURE_MIN_CHANGE = 20; // manual potentiometer changes are considered only if delta to current value >= this value

millibar_t targetPressure = 0;

analog_read_t readTargetPressureRaw() {
  return analogRead(TARGET_PRESSURE_POTI_IN_PIN); // --> 0..1023
}

millibar_t convertTargetPressure(analog_read_t pressureRaw) {
  return (uint32_t) pressureRaw * PUMP_MAX_PRESSURE / ANALOG_IN_MAX;
}

bool readTargetPressure() {
  millibar_t previousTargetPressure = targetPressure;
  analog_read_t raw = readTargetPressureRaw();
  millibar_t p = convertTargetPressure(raw);
  if (abs(p - previousTargetPressure) >= TARGET_PRESSURE_MIN_CHANGE) {
    targetPressure = p;
    #ifdef VERBOSE
      Serial.print("New Target Pressure = ");
      Serial.print(targetPressure);
      Serial.println(" mBar");
    #endif
    return true;
  }
  return false;
}


// ** Actual Pressure **
const analog_read_t PRESSURE_SENSOR_MIN_RAW =  (uint32_t) ANALOG_IN_MAX * PRESSURE_SENSOR_MIN_VOLTAGE / ANALOG_IN_MAX_VOLTAGE;
const analog_read_t PRESSURE_SENSOR_MAX_RAW =  (uint32_t) ANALOG_IN_MAX * PRESSURE_SENSOR_MAX_VOLTAGE / ANALOG_IN_MAX_VOLTAGE;
const analog_read_t PRESSURE_SENSOR_RANGE_RAW = PRESSURE_SENSOR_MAX_RAW - PRESSURE_SENSOR_MIN_RAW;

const millibar_t PRESSURE_SENSOR_MIN_CHANGE = 50; // pressure changes are considered only if delta to current value >= this value

analog_read_t calibratedMinPressureRaw = PRESSURE_SENSOR_MIN_RAW;
millibar_t actualPressure = PRESSURE_SENSOR_MIN_PRESSURE;

analog_read_t readActualPressureRaw() {
  return analogRead(PRESSURE_SENSOR_IN_PIN);  // --> 0..1023
}

millibar_t convertActualPressure(analog_read_t pressureRaw) {
  if (pressureRaw < PRESSURE_SENSOR_MIN_RAW) { 
    // the sensor output can actually be below the specified minimum …
    calibratedMinPressureRaw = pressureRaw;
  }
  return PRESSURE_SENSOR_MIN_PRESSURE + (int32_t) (pressureRaw - calibratedMinPressureRaw) * PRESSURE_SENSOR_MAX_PRESSURE / PRESSURE_SENSOR_RANGE_RAW;
}

bool readActualPressure() {
  millibar_t previousActualPressure = actualPressure; 
  analog_read_t raw = readActualPressureRaw();
  millibar_t p = convertActualPressure(raw);
  if (abs(p - previousActualPressure) >= PRESSURE_SENSOR_MIN_CHANGE) {
    actualPressure = p;
    #ifdef VERBOSE
      Serial.print("New Actual Pressure = ");
      Serial.print(actualPressure);
      Serial.println(" mBar");
    #endif
    return true;
  }
  return false;
}

// ** Water Flow **
const duty_value_t WATER_FLOW_MIN_CHANGE = 5; // manual potentiometer changes are considered only if delta to current value >= this value

analog_read_t readTargetWaterFlowRaw() {
  return analogRead(WATER_FLOW_POTI_IN_PIN); // --> 0..1023
}

//
// ANALOG OUT
//
/* See pump_control.h and pump_control.cpp */


//
// SETUP
//
void setup() {  
  configInput(PRESSURE_SENSOR_IN_PIN);
  configInput(TARGET_PRESSURE_POTI_IN_PIN);
  configInput(WATER_FLOW_POTI_IN_PIN);
  configInputWithPullup(OPERATION_MODE_IN_PIN);
  
  configOutput(PUMP_PWM_OUT_PIN);
  
  #if defined(__AVR_ATmega328P__)  
    configOutput(STATUS_LED_OUT_PIN);
    
  #elif defined(__AVR_ATtiny85__)
    // configOutput(STATUS_LED_OUT_PIN);   ******** NOT CONFIGURED ********
  #endif

  #ifdef VERBOSE
    // Setup Serial Monitor
    Serial.begin(9600);
    Serial.println("------------------------------------");
    Serial.print("Pump duty value low threshold := ");
    Serial.println(PUMP_OUT_LOW_THRESHOLD);
    Serial.print("Pressure min raw := ");
    Serial.println(PRESSURE_SENSOR_MIN_RAW);
    Serial.print("Pressure max raw := ");
    Serial.println(PRESSURE_SENSOR_MAX_RAW);
    Serial.print("Pressure range raw = ");
    Serial.println(PRESSURE_SENSOR_RANGE_RAW);

    Serial.print("Actual pressure raw = ");
    Serial.println(readActualPressureRaw());
    readActualPressure(); // prints pressure in psi
  #endif

  updateControllerModeAndState();
  setStatusLED(HIGH);
}

//
// LOOP
//
void loop() { 

  #ifdef INPUT_OUTPUT_TEST_MODE
    loop_io_test_mode();
  #else
    loop_prod_mode();
  #endif
}

void loop_prod_mode() {
  
  uint32_t now = millis();  
   
  if (pumpState() == PUMP_STARTING || pumpState() == PUMP_STOPPING || pumpState() == PUMP_SPEEDING_UP || pumpState() == PUMP_SLOWING_DOWN) {
    #if defined(VERBOSE) && defined(DEBUG_STATES) 
      Serial.println("-- 1 Transitioning");
    #endif
    // finish transitioning before processing pressure inputs!
    controllerState = STATE_TRANSITIONING;
    handlePumpStateTransition();
    if (pumpState() == PUMP_ON) {
      controllerState = controllerMode == MODE_CONTINUOUS ? STATE_CONTINUOUS : STATE_PRESSURE_LOW;
    } else if (pumpState() == PUMP_OFF) {
      controllerState = controllerMode == MODE_CONTINUOUS ? STATE_ERROR : STATE_PRESSURE_UP;
    }
    
  } else if (readMode() != controllerMode) {
    #if defined(VERBOSE) && defined(DEBUG_STATES) 
      Serial.println("-- 2 Mode changed");
    #endif
    updateControllerModeAndState();

  } else if (controllerState == STATE_CONTINUOUS) {
    #if defined(VERBOSE) && defined(DEBUG_STATES) 
      Serial.println("-- 3 Continuous Mode");
    #endif
    if (pumpState() == PUMP_OFF) {
      toggleMode(now);
    }

  } else if (controllerState == STATE_PRESSURE_LOW || controllerState == STATE_PRESSURE_UP) { 
    #if defined(VERBOSE) && defined(DEBUG_STATES) 
      Serial.println("-- 4 Pressure-Driven Mode");
    #endif
    readActualPressure();
    readTargetPressure();
    
    if (targetPressure - actualPressure >= PRESSURE_CHANGE_MIN_DELTA) {
      if (controllerState != STATE_PRESSURE_LOW) {
        controllerState = STATE_PRESSURE_LOW;
        #ifdef VERBOSE
          Serial.print("State := ");
          Serial.println(controllerStateName());
        #endif
      }
      if (pumpState() != PUMP_ON) {
        toggleMode(now);
      }
  
    } else if (actualPressure >= targetPressure) {
      if (controllerState != STATE_PRESSURE_UP) {
        controllerState = STATE_PRESSURE_UP;
        #ifdef VERBOSE
          Serial.print("State := ");
          Serial.println(controllerStateName());
        #endif
      }
      if (pumpState() != PUMP_OFF) {
        toggleMode(now);
      }
    }
    
    upddateTargetWaterFlow(readTargetWaterFlowRaw());  // may set pump state to PUMP_SPEEDING_UP or PUMP_SLOWING DOWN
    
  } else if (controllerState == STATE_ERROR) {
    #if defined(VERBOSE) && defined(DEBUG_STATES) 
      Serial.println("-- 5 Error");
    #endif
    if (controllerMode != MODE_ERROR) {
      controllerMode = MODE_ERROR;
      #ifdef VERBOSE
        Serial.println("Mode := ERROR");
      #endif
    }
    uint32_t d = CONTROL_CYCLE_DURATION / 5;

    bool on = false;
    for (uint8_t i = 0; i<5; i++) {
      setStatusLED(on);
      _delay_ms(d);
      on ^= on;
    }
    
//  } else {
//    #if defined(VERBOSE) && defined(DEBUG_STATES) 
//      Serial.print("-- 6: state = ");
//      Serial.println(controllerStateName());
//    #endif
  }
  
  _delay_ms(CONTROL_CYCLE_DURATION);  
}


#ifdef INPUT_OUTPUT_TEST_MODE

  typedef enum {
      TEST_INIT,
      TEST_PWM_OUTPUT,
      TEST_MODE_INPUT,
      TEST_FLOW_INPUT,
      TEST_TARGET_PRESSURE_INPUT,
      TEST_ACTUAL_PRESSURE_INPUT,
      TEST_ENDED
  } TestStage;
  
  TestStage testStage = TEST_INIT;
  
  #ifdef VERBOSE
    char* testStageName() {
      switch (testStage) {
        case TEST_INIT: return "TEST_INIT";
        case TEST_PWM_OUTPUT: return "TEST_PWM_OUTPUT";
        case TEST_MODE_INPUT: return "TEST_MODE_INPUT";
        case TEST_FLOW_INPUT: return "TEST_FLOW_INPUT";
        case TEST_TARGET_PRESSURE_INPUT: return "TEST_TARGET_PRESSURE_INPUT";
        case TEST_ACTUAL_PRESSURE_INPUT: return "TEST_ACTUAL_PRESSURE_INPUT";
        case TEST_ENDED: return "TEST_ENDED";
        default: return "UNKNOWN";
      }
    }
  #endif 

void blinkPwmOutput() {
  for(int i=0; i<5; i++) {
    setPumpDutyValue(ANALOG_OUT_MAX);
    _delay_ms(500);
    setPumpDutyValue(ANALOG_OUT_MIN);
    _delay_ms(500);
  }
}

void loop_io_test_mode() {
  #ifdef VERBOSE
    Serial.println("");
    Serial.println("*****************");
    Serial.print("Stage = ");
    Serial.println(testStageName());
  #endif
  
  _delay_ms(3000);
  
  switch (testStage) {
    
    case TEST_INIT: 
      flashLED(STATUS_LED_OUT_PIN, 5);
      _delay_ms(1000);
      flashLED(STATUS_LED_OUT_PIN, 5);
      testStage = TEST_PWM_OUTPUT;
      break;
    
    case TEST_PWM_OUTPUT: 
      for(int j=0; j<2; j++) {
        for(int i=1; i<= 4; i++) {
          setPumpDutyValue(ANALOG_OUT_MAX * i / 4);
          _delay_ms(500);
        }
        setPumpDutyValue(ANALOG_OUT_MIN);
        _delay_ms(500);
      }
      blinkPwmOutput();
      testStage = TEST_MODE_INPUT;
      break;
      
    case TEST_MODE_INPUT: readMode();
      for(int i=0; i<50; i++) {
        setPumpDutyValue(readMode() ? ANALOG_OUT_MAX : ANALOG_OUT_MIN);
        _delay_ms(100);
      }
      blinkPwmOutput();
      testStage = TEST_FLOW_INPUT;
      break;
      
    case TEST_FLOW_INPUT: 
      for(int i=1; i<=50; i++) {
        analog_read_t value = readTargetWaterFlowRaw();
        setPumpDutyValue(value / 4);
        _delay_ms(100);
        #ifdef VERBOSE
          if (i % 5 == 0) {
            Serial.print("Flow = ");
            Serial.println(value);
          }
        #endif
      }
      blinkPwmOutput();
      testStage = TEST_TARGET_PRESSURE_INPUT;
      break;
      
    case TEST_TARGET_PRESSURE_INPUT: 
      for(int i=1; i<=50; i++) {
        analog_read_t value = readTargetPressureRaw();
        setPumpDutyValue(value / 4);
        _delay_ms(100);
        #ifdef VERBOSE
          if (i % 5 == 0) {
            Serial.print("Target Pressure = ");
            Serial.println(value);
          }
        #endif
      }
      blinkPwmOutput();
      testStage = TEST_ACTUAL_PRESSURE_INPUT;
      break;
      
    case TEST_ACTUAL_PRESSURE_INPUT: 
      for(int i=1; i<=50; i++) {
        analog_read_t value = readActualPressureRaw();
        setPumpDutyValue(value / 4);
        _delay_ms(100);
        #ifdef VERBOSE
          if (i % 5 == 0) {
            Serial.print("Actual Pressure = ");
            Serial.println(value);
          }
        #endif
      }
      blinkPwmOutput();
      testStage = TEST_ENDED;
      break;

    case TEST_ENDED:
      break;
      
    default:
      break;
  }
}
  
#endif
