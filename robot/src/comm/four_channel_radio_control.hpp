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
uint16_t rc_read(uint8_t ch);

void rc_ch_1_int_handler();
void rc_ch_2_int_handler();
void rc_ch_3_int_handler();
void rc_ch_4_int_handler();
void rc_update_ch_value(const uint8_t& pin, volatile uint16_t* ch_timer_begin, volatile uint16_t* ch_value);

const uint8_t kFourChannelRCMaxNChannel = 4;
volatile uint16_t g_rc_ch_values[kFourChannelRCMaxNChannel] = {0};
volatile uint16_t g_rc_ch_timer_begins[kFourChannelRCMaxNChannel] = {0};

uint8_t g_rc_ch_pins[kFourChannelRCMaxNChannel] = {21, 20, 19, 18};
voidFuncPtr g_rc_ch_int_handlers[kFourChannelRCMaxNChannel] = {(voidFuncPtr) &rc_ch_1_int_handler, (voidFuncPtr) &rc_ch_2_int_handler, (voidFuncPtr) &rc_ch_3_int_handler, (voidFuncPtr) &rc_ch_4_int_handler};

bool rc_init(uint8_t n_ch) {
  if (n_ch > kFourChannelRCMaxNChannel) 
    return false;
  
  for (uint8_t i=0; i<n_ch; ++i) {
    pinMode(g_rc_ch_pins[i], INPUT);
    attachInterrupt(i+2, g_rc_ch_int_handlers[i], CHANGE);// plus two as we use external int2, int3, int4, int5
  }
  
  return true;
}

uint16_t rc_read(uint8_t ch) {
  if (ch > kFourChannelRCMaxNChannel)
    return 0;
  else  
    return g_rc_ch_values[ch-1];
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

void rc_ch_2_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[1], &g_rc_ch_timer_begins[1], &g_rc_ch_values[1]);
}

void rc_ch_3_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[2], &g_rc_ch_timer_begins[2], &g_rc_ch_values[2]);
}

void rc_ch_4_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[3], &g_rc_ch_timer_begins[3], &g_rc_ch_values[3]);
}

//class FourChannelRadioControl {
 //public:
  //FourChannelRadioControl(uint8_t n_ch): n_ch_(n_ch) {
    //ch_pins_[0] = 21;
    //ch_pins_[1] = 20;
    //ch_pins_[2] = 19;
    //ch_pins_[3] = 18;
    
    //ch_int_handlers_[0] = (voidFuncPtr) &FourChannelRadioControl::ch_1_int_handler;
    //ch_int_handlers_[1] = (voidFuncPtr) &FourChannelRadioControl::ch_2_int_handler;
    //ch_int_handlers_[2] = (voidFuncPtr) &FourChannelRadioControl::ch_3_int_handler;
    //ch_int_handlers_[3] = (voidFuncPtr) &FourChannelRadioControl::ch_4_int_handler;
    
    //for (uint8_t i=0; i<n_ch_; ++i) {
      //pinMode(ch_pins_[i], INPUT);
      //attachInterrupt(i+2, ch_int_handlers_[i], CHANGE);// plus two as we use external int2, int3, int4, int5
    //}
  //}
  
  //uint16_t read(uint8_t ch) {
    //return ch_values_[ch-1];
  //}
  
 //private:
  //uint8_t n_ch_;
  //uint8_t ch_pins_[4];
  //voidFuncPtr ch_int_handlers_[4];
  
  //static uint16_t ch_values_[4];
  //static uint16_t ch_timer_begins_[4];

  
  //void update_ch_value(const uint8_t& pin, volatile uint16_t* ch_timer_begin, volatile uint16_t* ch_value) {
    //if (digitalRead(pin) == 1) {
      //*ch_timer_begin = micros();
    //} 
    //else {
      //if (*ch_timer_begin != 0) {
        //*ch_value = micros() - *ch_timer_begin;
        //*ch_timer_begin = 0;
      //}
    //}   
  //}  

  //void ch_1_int_handler() {
    //update_ch_value(ch_pins_[0], &ch_timer_begins_[0], &ch_values_[0]);
  //}
  
  //void ch_2_int_handler() {
    //update_ch_value(ch_pins_[1], &ch_timer_begins_[1], &ch_values_[1]);
  //}
  
  //void ch_3_int_handler() {
    //update_ch_value(ch_pins_[2], &ch_timer_begins_[2], &ch_values_[2]);
  //}
  
  //void ch_4_int_handler() {
    //update_ch_value(ch_pins_[3], &ch_timer_begins_[3], &ch_values_[3]);
  //}
//};

//uint16_t FourChannelRadioControl::ch_values_[4] = {0};
//uint16_t FourChannelRadioControl::ch_timer_begins_[4] = {0};

}// namespace crim
#endif
