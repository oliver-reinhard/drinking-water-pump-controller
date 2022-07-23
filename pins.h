#ifndef PINSL_H_INCLUDED
  #define PINSL_H_INCLUDED
  
  #include <Arduino.h> 

  const pin_t PIN_NOT_WIRED = 99;
  
  #if defined(__AVR_ATmega328P__)
  
    #define VERBOSE
    
    // Pin A0 is kaput
    // Pin A1 is kaput
    const pin_t WATER_FLOW_POTI_IN_PIN = A2;        // analog in, pump power demand
    const pin_t TARGET_PRESSURE_POTI_IN_PIN = A3;   // analog in, water pressure demand
    const pin_t PRESSURE_SENSOR_IN_PIN = A4;        // analog in, pressure transducer (0.5 .. 4.5V --> 0 .. 80 psi)
    const pin_t OPERATION_MODE_IN_PIN = 12;         // digital in with pull-up configration
    
    const pin_t PUMP_PWM_OUT_PIN = 11;             // PWM @ native frequency
    const pin_t STATUS_LED_OUT_PIN = 13;           // digital out; is on when pump is off, blinks while transitioning
      
  #elif defined(__AVR_ATtiny85__)
    const pin_t WATER_FLOW_POTI_IN_PIN = A2;       // Pin 3 (PB4) - analog in, pump power demand
    const pin_t TARGET_PRESSURE_POTI_IN_PIN = A3;  // Pin 2 (PB3) - analog in, water pressure demand
    const pin_t PRESSURE_SENSOR_IN_PIN = A1;       // Pin 7 (PB2) - analog in, pressure transducer (0.5 .. 4.5V --> 0 .. 80 psi)
    const pin_t OPERATION_MODE_IN_PIN = PB0;       // Pin 5 - digital in with pull-up configration
    
    const pin_t PUMP_PWM_OUT_PIN = PB1;             // Pin 6 - PWM @ native frequency
    const pin_t TINY_AVR_PROGRAMMER_LED_PIN = PB0;  // Temporary use of PB0 (=OPERATION_MODE_IN_PIN) as a status indicator during TESTING ONLY
    const pin_t STATUS_LED_OUT_PIN = PIN_NOT_WIRED;   //*** ATtiny85: NOT WIRED / CONFIGURED ***     // digital out; is on when pump is off, blinks while transitioning
      
  #endif 

#endif 
