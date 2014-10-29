#include "motor.h"


namespace trui {

  

  Motor::Motor (size_t pwm_pin_, size_t dir_pin_, size_t encoder_out_a_pin, size_t encoder_out_b_pin, uint16_t encoder_resolution, float outmax_, float outmin_, int numerator_, int denominator_) : pwm_pin_(pwm_pin_), dir_pin_(dir_pin_), encoder_out_a_pin(encoder_out_a_pin), encoder_out_b_pin(encoder_out_b_pin), encoder_resolution(encoder_resolution), outmax_(outmax_), outmin_(outmin_), numerator_(numerator_), denominator_(denominator_) {
    omega_=0, omega_input_=0, last_omega_=0;
    tick_= 0;
    tick_enc_= 0;
    last_tick_enc_= 0;
    last2_tick_enc_= 0; 
    mv_=0, iTerm_= 0;    
    delta_=0, error_=0, last_error_=0; 
    data_=0;
    kp_= 0.316, ki_= 0.0528, kd_= 0;

    setup();
    encoder_ = new crim::TwoPhaseIncrementalEncoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);

  }

  Motor::~Motor() {
    delete encoder_;
  }

  void Motor::setup() {
    pinMode(dir_pin_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT);    
  }

  void Motor::testing_encoder(){
    while (true) {
      Serial.println("===");
      Serial.println(static_cast<long int>(encoder_->pos()));
      Serial.println(encoder_->rot());
      delay(100);
    }
  }

  void Motor::set_speed(float cmd_speed) {
    omega_input_ = cmd_speed; //setpoint
      tick_enc_ = encoder_->pos();

      omega_ = (float)(tick_enc_ - last_tick_enc_)*numerator_/denominator_; // Tetha = ((tickEnc - last_tickEnc)/250) * 2 * PI rad
                                             // omega = Tetha / Delta_Time -- Delta_Time = 50ms
                                             // omega = Tetha / 50ms = ((tickEnc - last_tickEnc)/250) * 2 * PI * 1000/50 rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI * (1/(2PI)) rotation/rad rad/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/(1/60) minute
                                             // omega = (tickEnc - last_tickEnc) * 2/25 * 60 rotation/minute
                                             // omega = (tickEnc - last_tickEnc) * 24/5 RPM

      error_ = omega_input_ - omega_;
      iTerm_ = iTerm_ + (float)error_*ki_;       //Integral Term of PID Control 
      
      if(iTerm_ > outmax_) iTerm_ = outmax_;
      else if(iTerm_ < outmin_) iTerm_ = outmin_;
                    
      deriv_comp_ = (tick_enc_ - 2*last_tick_enc_ + last2_tick_enc_)*numerator_/denominator_;
                    
      mv_ =  (float)error_*kp_ + iTerm_ - deriv_comp_*kd_;
      
      if(mv_ > outmax_) mv_ = outmax_;
      else if(mv_ < outmin_) mv_ = outmin_;
                    
      motorPWM_percentage(mv_); 
           
      last_error_ = error_;
      last2_tick_enc_ = last_tick_enc_; 
      last_tick_enc_ = tick_enc_;    
  }

  void Motor::motorPWM_percentage(float pwm) {
      if(pwm>100) pwm = 100;
      else if(pwm<-100) pwm = -100;
      
      ocr_ = (float)pwm*255/100;
      if(pwm>=0)
      {
        digitalWrite(dir_pin_, LOW);
        analogWrite(pwm_pin_, ocr_);
      }
      else if (pwm<0)
      {
        digitalWrite(dir_pin_, HIGH);
        analogWrite(pwm_pin_, -ocr_);
      }
      else
      {
        analogWrite(pwm_pin_, 0);
      }
  }

}// namespace trui
