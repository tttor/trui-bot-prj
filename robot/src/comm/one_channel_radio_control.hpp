// @author vektor dewanto
#ifndef FOUR_CHANNEL_RADIO_CONTROL
#define FOUR_CHANNEL_RADIO_CONTROL

#include <arduino-core/Arduino.h>
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <arduino-core/wiring_private.h>// for voidFuncPtr

// TODO @tttor: fail to encapsulate the RC routines into a class; interrupts do not work properly

// http://arduino.cc/en/Reference/AttachInterrupt#.UwHlZ9_LXCQ
//Inside the attached function, delay() won't work and the value returned by millis() will not increment. Serial data received while in the function may be lost. You should declare as volatile any variables that you modify within the attached function. See the section on ISRs below for more information.

// http://rcarduino.blogspot.com/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/04/how-to-read-multiple-rc-channels-draft.html

namespace crim{

bool rc_init();
uint16_t rc_read();

void rc_ch_1_int_handler();
void rc_update_ch_value(const uint8_t& pin, volatile uint16_t* ch_timer_begin, volatile uint16_t* ch_value);

const uint8_t kOneChannelRCMaxNChannel = 1;
volatile uint16_t g_rc_ch_values[kOneChannelRCMaxNChannel] = {0};
volatile uint16_t g_rc_ch_timer_begins[kOneChannelRCMaxNChannel] = {0};

uint8_t g_rc_ch_pins[kOneChannelRCMaxNChannel] = {2};// NOTE: utilize ext int0 in pin 2
voidFuncPtr g_rc_ch_int_handlers[kOneChannelRCMaxNChannel] = {(voidFuncPtr) &rc_ch_1_int_handler};

bool rc_init() {
  pinMode(g_rc_ch_pins[0], INPUT);
  attachInterrupt(0, g_rc_ch_int_handlers[0], CHANGE);// NOTE: utilize ext int0 in pin 2
  
  return true;
}

uint16_t rc_read() {
 return g_rc_ch_values[0];
}

void rc_update_ch_value(const uint8_t& pin, volatile uint16_t* ch_timer_begin, volatile uint16_t* ch_value) {
  if (digitalRead(pin) == 1) {
    *ch_timer_begin = micros();
  } 
  else {
    if (*ch_timer_begin != 0) {
      *ch_value = micros() - *ch_timer_begin;
      *ch_timer_begin = 0;
    }
  }   
}  

void rc_ch_1_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[0], &g_rc_ch_timer_begins[0], &g_rc_ch_values[0]);
}

}// namespace crim
#endif
