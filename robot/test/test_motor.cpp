
int main() {
  const size_t pwm_pin = 11;
  const size_t dir_pin = 12; 
  const float outmax = 100.0; 
  const float outmin = -100.0; 

  const int encoder_out_a_pin = 2;
  const int encoder_out_b_pin = 3;
  const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called
   
  Motor motor(pwm_pin, dir_pin);

  while (true) {
    motor.set_speed(??);
  }
}