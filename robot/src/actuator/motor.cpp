#include "motor.h"

namespace trui {

  Motor::Motor(): pwm_pin_(pwm_pin), dir_pin_(dir_pin), encoder_out_a_pin(encoder_out_a_pin), encoder_out_b_pin(encoder_out_b_pin), encoder_resolution(encoder_resolution, outmax_(outmax), outmin_(outmin)) {
    tick_= 0, tick_enc_= 0, last_tick_enc_= 0, last2_tick_enc_= 0; 
    omega_=0, omega_input_=0, last_omega_=0;
    mv_=0, iTerm_= 0;    
    delta_=0, error_=0, last_error_=0; 
    data_=0;
    kp_= 0.316, ki_= 0.0528, kd_= 0;

    pinMode(dir_pin_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT);

    encoder_ = new crim::TwoPhaseIncrementalEncoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);


  }

  Motor::~Motor() {
    delete encoder_;
  }

  void Motor::setup() {
    pinMode(motorCS, OUTPUT);
    pinMode(motorPin, OUTPUT);
  }

  float Motor::set_speed(float cmd_speed) {
    omega_input_= cmd_speed;
    tick_enc_ = encoder_->pos();

    omega_ = (float)(tick_enc_ - last_tick_enc_)*24/5; 

    error_ = omega_input_ - omega_;
    iTerm_ = iTerm_ + (float)error_*ki_;       
        
    if(iTerm_ > outmax_) iTerm_ = outmax_;
    else if(iTerm_ < outmin_) iTerm_ = outmin_;
                      
    deriv_comp_ = (tick_enc_ - 2*last_tick_enc_ + last2_tick_enc_)*24/5;
                      
    mv_ =  (float)error_*kp_ + iTerm_ - deriv_comp_*kd_;
        
    if(mv_ > outmax_) mv_ = outmax_;
    else if(mv_ < outmin_) mv_ = outmin_;
                      
    motorPWM_percentage(mv_); 
             
    last_error_ = error_;
    last2_tick_enc_ = last_tick_enc_; 
    last_tick_enc_ = tick_enc_;
    return omega_;
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
