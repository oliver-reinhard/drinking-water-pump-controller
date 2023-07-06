#include "pump_control.h"
#include "pins.h"

PumpMode pumpMode_ = MODE_ON;

PumpMode pumpMode() {
  return pumpMode_;
}

PumpState pumpState_ = PUMP_OFF;

PumpState pumpState() {
  return pumpState_;
}

#ifdef VERBOSE
  const char* pumpStateName() {
    switch (pumpState_) {
      case PUMP_OFF: return "OFF";
      case PUMP_STARTING: return "STARTING";
      case PUMP_ON: return "ON";
      case PUMP_SPEEDING_UP: return "SPEEDING_UP";
      case PUMP_SLOWING_DOWN: return "SLOWING_DOWN";
      case PUMP_STOPPING: return "STOPPING";
      default: return "UNKNOWN";
    }
  }
#endif 

void setPumpDutyValue(duty_value_t value);

uint32_t lastPumpStateChangeTime = 0;

//
// DIGITAL OUT
//
boolean statusLEDState = LOW;

//
// ANALOG OUT
//


// Pump soft start and soft stop:
const duty_value_t PUMP_START_INCREMENT = ((uint32_t) ANALOG_OUT_MAX - (uint32_t) PUMP_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / PUMP_START_DURATION;
const duty_value_t PUMP_STOP_INCREMENT = ((uint32_t) ANALOG_OUT_MAX - (uint32_t) PUMP_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / PUMP_STOP_DURATION;

const duty_value_t PUMP_DUTY_VALUE_MIN_CHANGE = 5; // manual potentiometer changes are considered only if delta to current value >= this value

duty_value_t pumpTargetDutyValue = ANALOG_OUT_MIN; // potentiometer value read from input pin
duty_value_t pumpActualDutyValue = ANALOG_OUT_MIN; // value actually set on output pin
uint32_t transitionBeginTime = 0;
duty_value_t transitioningDutyValue = ANALOG_OUT_MIN; // incremented in discrete steps until pump is at its target speed or its low end


/* 
 *  Evaluate if the target water flow of the pump has changed signficantly and record the new value.
 *  
 *  If the function returns true, then the new target values are stored in pumpTargetDutyValue, however, pumpActualDutyValue remains unchanged! 
 */
bool upddateTargetWaterFlow(analog_read_t newFlow) {
  duty_value_t previousPumpTargetDutyValue = pumpTargetDutyValue;
  duty_value_t value = map(newFlow, ANALOG_IN_MIN, ANALOG_IN_MAX, PUMP_OUT_LOW_THRESHOLD , ANALOG_OUT_MAX); // Map the potentiometer value 
  duty_value_t delta = abs((int16_t) value - (int16_t) previousPumpTargetDutyValue);
  
  // Only act if target value moves by at least the min. change amount:
  if (delta >= PUMP_DUTY_VALUE_MIN_CHANGE) {
    // with this rule, we may never get to the extremes, PUMP_OUT_LOW_THRESHOLD and ANALOG_OUT_MAX --> additional rules below
    pumpTargetDutyValue = value;
  } else if (value < PUMP_OUT_LOW_THRESHOLD + PUMP_DUTY_VALUE_MIN_CHANGE && pumpTargetDutyValue != PUMP_OUT_LOW_THRESHOLD) {
    pumpTargetDutyValue = PUMP_OUT_LOW_THRESHOLD;
  } else if (value > ANALOG_OUT_MAX - PUMP_DUTY_VALUE_MIN_CHANGE && pumpTargetDutyValue != ANALOG_OUT_MAX) {
    pumpTargetDutyValue = ANALOG_OUT_MAX;
  } else {
    return false;
  }
  
  if (pumpState_ == PUMP_ON) {
    if (pumpTargetDutyValue > pumpActualDutyValue) {
      pumpState_ = PUMP_SPEEDING_UP;
      transitionBeginTime = 0;
      transitioningDutyValue = pumpActualDutyValue;
        
    } else if (pumpTargetDutyValue < pumpActualDutyValue) {
      pumpState_ = PUMP_SLOWING_DOWN;
      transitionBeginTime = 0;
      transitioningDutyValue = pumpActualDutyValue;
    }
 }
  #ifdef VERBOSE
    Serial.print("New Duty Value (Flow): ");
    printDutyValueAsPercent(pumpTargetDutyValue);
    Serial.println();
  #endif
  return true;
}


/*
 * Switch mode: on <-> off (including transitions).
 */

void toggleMode(uint32_t now) {
  if (now - lastPumpStateChangeTime < MIN_PUMP_STATE_PERSISTENCE) {
    #ifdef VERBOSE
      Serial.println("Toggle Mode - ignored (too soon after last event)");
    #endif
    return;
  }
  
  #ifdef VERBOSE
    Serial.println("Toggle Mode");
  #endif
    
  switch(pumpState_) {
    case PUMP_OFF:
    case PUMP_STOPPING:
      pumpState_ = PUMP_STARTING;
      pumpMode_ = MODE_ON;
      transitioningDutyValue = PUMP_OUT_LOW_THRESHOLD;
      #ifdef VERBOSE
        Serial.println("PUMP STARTING ...");
      #endif
      break;
    
    case PUMP_ON:
    case PUMP_STARTING:
    case PUMP_SPEEDING_UP:
    case PUMP_SLOWING_DOWN:
      pumpState_ = PUMP_STOPPING;
      pumpTargetDutyValue = PUMP_OUT_LOW_THRESHOLD;
      transitioningDutyValue = pumpActualDutyValue;
      #ifdef VERBOSE
        Serial.println("PUMP STOPPING ...");
      #endif
      break;

    default:
      break;
  } 
  lastPumpStateChangeTime = now;
  transitionBeginTime = now;
}

/*
 * A new targetDutyValue has been set in state PUMP_ON
 */
void adaptActualWaterFlow() {
  if (pumpState_ != PUMP_ON) {
    #ifdef VERBOSE
      Serial.print("Cannot change speed in state ");
      Serial.println(pumpStateName());
    #endif
    return;
  }
}


//
// State controller
//
void handlePumpStateTransition() {
  switch (pumpState_) {
    case PUMP_STARTING:
    case PUMP_SPEEDING_UP:
      // ensure we don't overrun the max value of uint8_t:
      if ((int16_t) transitioningDutyValue + (int16_t) PUMP_START_INCREMENT <= (uint8_t) pumpTargetDutyValue) {
        transitioningDutyValue += PUMP_START_INCREMENT;
      } else {
        transitioningDutyValue = pumpTargetDutyValue;
      }
      setPumpDutyValue(transitioningDutyValue);
      invertStatusLED();
      #ifdef VERBOSE
        Serial.print(pumpState_ == PUMP_STARTING ? "Starting: " : "Speeding up: ");
        printDutyValueAsPercent(transitioningDutyValue);
        Serial.println();
      #endif
      
      if (transitioningDutyValue == pumpTargetDutyValue) {
        pumpState_ = PUMP_ON;
        transitionBeginTime = 0;
        transitioningDutyValue = ANALOG_OUT_MIN;
        setStatusLED(LOW);
        #ifdef VERBOSE
          Serial.println("PUMP ON");
        #endif
      }
      break;
    
    case PUMP_STOPPING:
      // ensure transitioningDutyValue will not become < 0 (is an uint8_t!)
      if ((int16_t) transitioningDutyValue - (int16_t) PUMP_STOP_INCREMENT > (uint8_t) PUMP_OUT_LOW_THRESHOLD) {
        transitioningDutyValue -= PUMP_STOP_INCREMENT;
      } else {
        // new value is below PUMP_OUT_LOW_THRESHOLD --> turn pump off completely:
        transitioningDutyValue = ANALOG_OUT_MIN;
      }
      setPumpDutyValue(transitioningDutyValue);
      invertStatusLED();
      #ifdef VERBOSE
        Serial.print("Stopping: ");
        printDutyValueAsPercent(transitioningDutyValue);
        Serial.println();
      #endif
      
      if (transitioningDutyValue == ANALOG_OUT_MIN) {
        pumpState_ = PUMP_OFF;
        pumpMode_  = MODE_OFF;
        transitionBeginTime = 0;
        transitioningDutyValue = ANALOG_OUT_MIN;
        setStatusLED(HIGH);
        #ifdef VERBOSE
          Serial.println("PUMP OFF");
        #endif
      }
      break;

    
    case PUMP_SLOWING_DOWN:
      if (pumpTargetDutyValue < PUMP_OUT_LOW_THRESHOLD) {
        pumpTargetDutyValue = PUMP_OUT_LOW_THRESHOLD;
      }
      // ensure transitioningDutyValue will not become < 0 (is an uint8_t!)
      if ((int16_t) transitioningDutyValue - (int16_t) PUMP_STOP_INCREMENT > (uint8_t) pumpTargetDutyValue) {
        transitioningDutyValue -= PUMP_STOP_INCREMENT;
      } else {
        // new value is below pumpTargetDutyValue --> set to target value:
        transitioningDutyValue = pumpTargetDutyValue;
      }
      setPumpDutyValue(transitioningDutyValue);
      invertStatusLED();
      #ifdef VERBOSE
        Serial.print("Slowing down: ");
        printDutyValueAsPercent(transitioningDutyValue);
        Serial.println();
      #endif
      
      if (transitioningDutyValue == pumpTargetDutyValue) {
        pumpState_ = PUMP_ON;
        transitionBeginTime = 0;
        transitioningDutyValue = ANALOG_OUT_MIN;
        setStatusLED(HIGH);
        #ifdef VERBOSE
          Serial.println("PUMP ON");
        #endif
      }
      break;

    default:
      break;
  }
}

void emergencyStop() {
  if (pumpState_  != PUMP_OFF) {
    pumpState_ = PUMP_OFF;
    transitionBeginTime = 0;
    transitioningDutyValue = ANALOG_OUT_MIN;
    setStatusLED(LOW);
    setPumpDutyValue(ANALOG_OUT_MIN);
  }
}

//
// Utility Routines
//
void setPumpDutyValue(duty_value_t value) {
  pumpActualDutyValue = value;
  analogWrite(PUMP_PWM_OUT_PIN, value); // Send PWM signal
}

void printDutyValueAsPercent(duty_value_t value) {
  Serial.print((uint16_t) value * 100 / ANALOG_OUT_MAX);
  Serial.print("%");
}

void setStatusLED(boolean value) {
  statusLEDState = value;
  if (STATUS_LED_OUT_PIN != PIN_NOT_WIRED) {
    digitalWrite(STATUS_LED_OUT_PIN, value);
  }
}

void invertStatusLED() {
  setStatusLED(statusLEDState == HIGH ? LOW : HIGH);
}
