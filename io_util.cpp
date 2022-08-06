//
// IMPORTANT NOTE
// 
// On the ATtiny85, this controller is supposed to run at 1 MHz. If not then adjust F_CPU (see util/delay.h) accordingly:
//  - in the main program file
//  - in io_util.cpp
//
// and replace the use the millis() function against a clock-corrected source.


#include <util/delay.h>
#include "io_util.h"
  
void configInput(pin_t pin) {
  pinMode(pin, INPUT);
}

void configInputWithPullup(pin_t pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);              // Activate pull-up resistor on pin (input)
}

void configOutput(pin_t pin) {
  pinMode(pin, OUTPUT);
}

void turnOnLED(pin_t pin, duration32_ms_t duration) {
  digitalWrite(pin, HIGH);
  _delay_ms(duration);
  digitalWrite(pin, LOW);
}
  
void flashLED(pin_t pin, uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    _delay_ms(100);
    digitalWrite(pin, LOW);
    _delay_ms(100);
  }
}
  
static inline void debounceSwitch() {
  _delay_ms(SWITCH_DEBOUNCE_WAIT_MS);
}
