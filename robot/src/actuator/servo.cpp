#include "servo.h"


namespace trui{

    // size_t pwm_pinServo_;
    // size_t dir_pin1_;
    // size_t dir_pin2_;
    // int64_t tickServo_;
    // int64_t tick_encServo_;
    // int64_t last_tick_encServo_;
    // int64_t last2_tick_encServo_;
    // float e_k_;
    // float e_k_1_;
    // float e_k_2_;
    // float U_t_;
    // float U_t_1_;
    // float outmaxServo_;
    // float outminServo_;
    // float ocrServo_;
  
  Servo::Servo (size_t pwm_pinServo_, size_t dir_pin1,size_t dir_pin2, size_t encoder_out_a_pin, size_t encoder_out_b_pin, uint16_t encoder_resolution, float outmaxServo_, float outminServo_) : pwm_pinServo_(pwm_pinServo_), dir_pin1_(dir_pin1),dir_pin2_(dir_pin2) ,encoder_out_a_pin(encoder_out_a_pin), encoder_out_b_pin(encoder_out_b_pin), encoder_resolution(encoder_resolution), outmaxServo_(outmaxServo_), outminServo_(outminServo_) {

    setup();
    U_t_=0,U_t_1_=0,e_k_=0,e_k_1_=0,e_k_2_=0;    
    encoder_ = new crim::TwoPhaseIncrementalEncoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);
  }

  Servo::~Servo() {
    delete encoder_;
  }

  void Servo::init() {
    pinMode(dir_pin1_, OUTPUT);
    pinMode(dir_pin2_, OUTPUT);
    pinMode(pwm_pinServo_, OUTPUT);    
  }

  int64_t Servo::read_encoder() {
      return encoder_->pos();
  }

  void Servo::reset(){
      encoder_->reset_Enc();
  }


  float Servo::PIDvelocity_algorithm(float error,float Kp,float Ki,float Kd,float delta_T){
      e_k_ = error;
      U_t_ = U_t_1_ + (Kp+Ki*delta_T+Kd/delta_T)*e_k_-(Kp+2*Kd/delta_T)*e_k_1_+(Kd/delta_T)*e_k_2_;
      e_k_2_ = e_k_1_;
      e_k_1_ = e_k_;
      U_t_1_ = U_t_;
      return U_t_;
  }

  void Servo::outSignal(float pwm){
  //COUNTER CLOCK WISE IS MINUS TICKING
  //PLUS SIGNEDPWM IS CLOCKWISE
      if(pwm>255) pwm = 255;
      else if(pwm<-255) pwm = -255;
      
      ocrServo_ = (float)pwm;
      
      if(pwm<0) {
        digitalWrite(dir_pin1_, LOW);
        digitalWrite(dir_pin2_, HIGH);
        analogWrite(pwm_pinServo_, -ocrServo_);
      }
      else if(pwm>0) {
        digitalWrite(dir_pin1_, HIGH);
        digitalWrite(dir_pin2_, LOW);
        analogWrite(pwm_pinServo_, ocrServo_);
      }
      else if (pwm == 0) {
        digitalWrite(dir_pin1_, HIGH);
        digitalWrite(dir_pin2_, HIGH);
        analogWrite(pwm_pinServo_, 0);
      }
  }

}
