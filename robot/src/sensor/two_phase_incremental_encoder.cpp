#include "two_phase_incremental_encoder.hpp"

using namespace crim;

int64_t TwoPhaseIncrementalEncoder::counter_ = 0;
bool TwoPhaseIncrementalEncoder::phase_a_state_ = 0;
bool TwoPhaseIncrementalEncoder::phase_b_state_ = 0;

TwoPhaseIncrementalEncoder::TwoPhaseIncrementalEncoder(size_t out_a_pin, size_t out_b_pin, uint64_t resolution)
    : out_a_pin_(out_a_pin), out_b_pin_(out_b_pin), resolution_(resolution) {

  pinMode(out_a_pin_, INPUT);
  digitalWrite(out_a_pin_, HIGH);// turn on pullup resistor

  pinMode(out_b_pin_, INPUT); 
  digitalWrite(out_b_pin_, HIGH);// turn on pullup resistor

  phase_b_state_ = digitalRead(out_b_pin_);
  phase_a_state_ = digitalRead(out_a_pin_);

  attachInterrupt(0, (voidFuncPtr) &TwoPhaseIncrementalEncoder::ext_int0_handler, CHANGE);//Default is RISING
  attachInterrupt(1, (voidFuncPtr) &TwoPhaseIncrementalEncoder::ext_int1_handler, CHANGE); 
}

TwoPhaseIncrementalEncoder::~TwoPhaseIncrementalEncoder() {
  //
}

int64_t TwoPhaseIncrementalEncoder::pos() {
  return counter_;
}

double TwoPhaseIncrementalEncoder::rot() {
  return (double)counter_/resolution_*2*3.14;
}

void TwoPhaseIncrementalEncoder::reset_Enc() {
  counter_ = 0;
  phase_a_state_ = digitalRead(out_a_pin_);
  phase_b_state_ = digitalRead(out_b_pin_);
}

void TwoPhaseIncrementalEncoder::ext_int0_handler() {
  // if (phase_b_state_) 
  //   phase_a_state_--;//counter_--;
  // else
  //   phase_a_state_++;//counter_++;
  phase_a_state_ = !phase_a_state_;
  if(phase_a_state_ == 1){
    if(phase_b_state_ == 0) counter_++;
    else counter_--;
  }
  else {
    if(phase_b_state_ == 1) counter_++;
    else counter_--;
  }
}
  
void TwoPhaseIncrementalEncoder::ext_int1_handler() {
  phase_b_state_ = !phase_b_state_;
  if(phase_b_state_ == 1){
    if(phase_a_state_ == 1) counter_++;
    else counter_--;
  }
  else {
    if(phase_a_state_ == 0) counter_++;
    else counter_--;
  }
}

