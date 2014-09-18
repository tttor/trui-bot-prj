#include "two_phase_incremental_encoder.hpp"

using namespace crim;

int64_t TwoPhaseIncrementalEncoder::phase_a_counter_ = 0;
bool TwoPhaseIncrementalEncoder::phase_b_state_ = 0;

TwoPhaseIncrementalEncoder::TwoPhaseIncrementalEncoder(size_t out_a_pin, size_t out_b_pin, uint64_t resolution)
    : out_a_pin_(out_a_pin), out_b_pin_(out_b_pin), resolution_(resolution) {

  pinMode(out_a_pin_, INPUT);
  digitalWrite(out_a_pin_, HIGH);// turn on pullup resistor

  pinMode(out_b_pin_, INPUT); 
  digitalWrite(out_b_pin_, HIGH);// turn on pullup resistor

  phase_b_state_ = digitalRead(out_b_pin_);

  attachInterrupt(0, (voidFuncPtr) &TwoPhaseIncrementalEncoder::ext_int0_handler, RISING);
  attachInterrupt(1, (voidFuncPtr) &TwoPhaseIncrementalEncoder::ext_int1_handler, CHANGE); 
}

TwoPhaseIncrementalEncoder::~TwoPhaseIncrementalEncoder() {
  //
}

int64_t TwoPhaseIncrementalEncoder::pos() {
  return phase_a_counter_;
}

double TwoPhaseIncrementalEncoder::rot() {
  return (double)phase_a_counter_/resolution_*2*crim::kPhi;
}

void TwoPhaseIncrementalEncoder::ext_int0_handler() {
  if (phase_b_state_) 
    phase_a_counter_--;
  else
    phase_a_counter_++;
}
  
void TwoPhaseIncrementalEncoder::ext_int1_handler() {
  phase_b_state_ = !phase_b_state_;
}
