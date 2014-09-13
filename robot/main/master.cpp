#include <arduino-core/Arduino.h>
#include <arduino-core/wiring_private.h>// for voidFuncPtr

namespace crim {

class IncrementalEncoder {
 public:
  IncrementalEncoder(size_t out_a_pin, size_t out_b_pin, uint64_t resolution)
      : out_a_pin_(out_a_pin), out_b_pin_(out_b_pin), resolution_(resolution) {
  
    pinMode(out_a_pin_, INPUT);
    digitalWrite(out_a_pin_, HIGH);// turn on pullup resistor
  
    pinMode(out_b_pin_, INPUT); 
    digitalWrite(out_b_pin_, HIGH);// turn on pullup resistor
  
    phase_b_state_ = digitalRead(out_b_pin_);
  
    attachInterrupt(0, (voidFuncPtr) &IncrementalEncoder::ext_int0_handler, RISING);
    attachInterrupt(1, (voidFuncPtr) &IncrementalEncoder::ext_int1_handler, CHANGE); 
  }
  
  ~IncrementalEncoder() {
    //
  }
  
  /**
   * @brief return the number of rising-edge of the a phase pulse
   * 
   */
  int64_t pos() {
    return phase_a_counter_;
  }
  
  /**
   * @brief return the number of rotation in double, can be negative
   * 
   */
  double rot() {
    return (double)phase_a_counter_/resolution_;
  }
  
 private:
  size_t out_a_pin_;// external interrupt 0 pin
  size_t out_b_pin_;// external interrupt 1 pin
  uint64_t resolution_;
  static int64_t phase_a_counter_;
  static bool phase_b_state_;
  
  void ext_int0_handler() {
    if (phase_b_state_) 
      phase_a_counter_--;
    else
      phase_a_counter_++;
  }
  
  void ext_int1_handler() {
    phase_b_state_ = !phase_b_state_;
  }
};

}// namespace crim

int64_t crim::IncrementalEncoder::phase_a_counter_ = 0;
bool crim::IncrementalEncoder::phase_b_state_ = 0;

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(9600);
  
  const size_t encoder_out_a_pin = 2;
  const size_t encoder_out_b_pin = 3;
  const uint64_t encoder_resolution = 360;
  crim::IncrementalEncoder encoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);
  
  while (true) {
    Serial.println("===");
    Serial.println(static_cast<long int>(encoder.pos()));
    Serial.println(encoder.rot());
    delay(100);
  }
  
  Serial.end();
  return 0;
}
