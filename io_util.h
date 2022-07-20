#ifndef IO_UTIL_H_INCLUDED
  #define IO_UTIL_H_INCLUDED
 
  #include <Arduino.h>


  // MISC DATA TYPES
  
  typedef uint16_t time16_ms_t;
  typedef uint32_t time32_ms_t;
  typedef uint16_t time16_s_t;

  typedef int16_t duration16_ms_t;
  typedef int32_t duration32_ms_t;
  
  typedef uint16_t millivolt_t;
  
  typedef int16_t millibar_t;
  
  const millibar_t MILLIBAR_PER_PSI = 69;
  
  typedef uint8_t pin_t;

  //
  // ANALOG IN
  //
  typedef uint16_t analog_read_t;
  
  const analog_read_t ANALOG_IN_MIN = 0;        // Arduino constant
  
  #if defined(__AVR_ATmega328P__)
    const analog_read_t ANALOG_IN_MAX = 1023;   // Arduino constant
      
  #elif defined(__AVR_ATtiny85__)
    const analog_read_t ANALOG_IN_MAX = 255;    // Arduino constant
  #endif
  
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
