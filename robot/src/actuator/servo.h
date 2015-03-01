#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>
#include <arduino/Arduino.h>
#include <sensor/two_phase_incremental_encoder.hpp>

namespace trui {

class Servo {
   public:
    Servo(size_t pwm_pin_, size_t dir_pin1_,size_t dir_pin2_, size_t encoder_out_a_pin, size_t encoder_out_b_pin, uint16_t encoder_resolution, float outmax_, float outmin_);
    ~Servo();
    size_t encoder_out_a_pin;
    size_t encoder_out_b_pin;
    uint16_t encoder_resolution;
    
    void init();
    int64_t read_encoder();
    void reset();
    float PIDvelocity_algorithm(float error,float Kp,float Ki,float Kd,float delta_T);
    void outSignal(float pwm);

   private:
    size_t pwm_pinServo_;
    size_t dir_pin1_;
    size_t dir_pin2_;
    int64_t tickServo_;
    int64_t tick_encServo_;
    int64_t last_tick_encServo_;
    int64_t last2_tick_encServo_;
    float e_k_;
    float e_k_1_;
    float e_k_2_;
    float U_t_;
    float U_t_1_;
    float outmaxServo_;
    float outminServo_;
    float ocrServo_;
    
    crim::TwoPhaseIncrementalEncoder* encoder_;

  };

}// namespace trui

#endif