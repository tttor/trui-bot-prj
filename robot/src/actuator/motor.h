#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include <arduino/Arduino.h>
#include <sensor/two_phase_incremental_encoder.hpp>

namespace trui {

class Motor {
   public:
    Motor(size_t pwm_pin, size_t dir_pin, size_t encoder_out_a_pin, size_t encoder_out_b_pin, uint16_t encoder_resolution, float outmax_, float outmin_);
    ~Motor();
    size_t encoder_out_a_pin;
    size_t encoder_out_b_pin;
    uint16_t encoder_resolution;
    /*!
     * @brief set the speed of motor in rpm
     * \param cmd_speed the commanded speed in rpm
     * \return the actual speed in rpm
     */
    void set_speed(float cmd_speed);
    void setup();
    void motorPWM_percentage(float pwm);
    void testing_encoder();

   private:
    
    
    size_t pwm_pin_;
    size_t dir_pin_;
    int64_t tick_;
    int64_t tick_enc_;
    int64_t last_tick_enc_;
    int64_t last2_tick_enc_;
    int64_t deriv_comp_;
    float omega_;
    float omega_input_;
    float last_omega_;
    float mv_;
    float iTerm_;
    float delta_;
    float error_;
    float last_error_;
    float kp_;
    float ki_;
    float kd_;
    float outmax_;
    float outmin_;
    uint8_t data_;
    uint16_t ocr_;

    crim::TwoPhaseIncrementalEncoder* encoder_;

  };

}// namespace trui

#endif