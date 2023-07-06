#ifndef IO_UTIL_H_INCLUDED
  #define IO_UTIL_H_INCLUDED
 
  #include <Arduino.h>


  // MISC DATA TYPES
  
  typedef uint8_t pin_t;
  
  typedef uint16_t time16_ms_t;
  typedef uint32_t time32_ms_t;
  typedef uint16_t time16_s_t;

  typedef uint16_t duration16_ms_t;
  typedef uint32_t duration32_ms_t;

  typedef uint16_t percentage_t;
  
  typedef uint16_t millivolt_t;
  
  typedef int16_t millibar_t;
  
  const millibar_t MILLIBAR_PER_PSI = 69;

  //
  // ANALOG IN
  //
  typedef uint16_t analog_read_t;
  
  const analog_read_t ANALOG_IN_MIN = 0;        // Arduino constant
  const analog_read_t ANALOG_IN_MAX = 1023;   // Arduino constant
  
  const millivolt_t ANALOG_IN_MAX_VOLTAGE = 5000; // Arduino constant


  void configInput(pin_t pin);
  void configInputWithPullup(pin_t pin);
  
  const duration16_ms_t SWITCH_DEBOUNCE_WAIT_MS = 10;
  void debounceSwitch();
  

 // ANALOG OUT

  typedef uint8_t duty_value_t;
 
  const duty_value_t ANALOG_OUT_MIN = 0;        // Arduino constant
  const duty_value_t ANALOG_OUT_MAX = 255;      // PWM control
  
  void configOutput(pin_t pin);

  void turnOnLED(pin_t pin, duration32_ms_t duration);
    
  void flashLED(pin_t pin, uint8_t times);
  
#endif
