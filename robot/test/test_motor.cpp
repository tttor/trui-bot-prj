#include <actuator/motor.h>

int main() {
  init();
  Serial.begin(9600);

  const size_t pwm_pin = 11;
  const size_t dir_pin = 12; 
  const float outmax = 100.0; 
  const float outmin = -100.0; 

  const int encoder_out_a_pin = 2;
  const int encoder_out_b_pin = 3;
  const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called
  
  trui::Motor motor(pwm_pin, dir_pin, encoder_out_a_pin, encoder_out_b_pin, encoder_resolution, outmax, outmin);

  long timeNow = 0, timeOld = 0;
  float cmd_speed = 50.0;

  while (true) {
    timeNow = millis();
    if(timeNow - timeOld > 50){
      timeOld = timeNow;
      motor.set_speed(cmd_speed);

      Serial.print("cmd_speed= "); Serial.println(cmd_speed);
    }  
  }
}