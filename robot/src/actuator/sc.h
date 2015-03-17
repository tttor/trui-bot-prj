#ifndef SC_H_
#define SC_H_

#include <stdint.h>
#include <arduino/Arduino.h>
#include <sensor/two_phase_incremental_encoder.hpp>


namespace trui {

class Sc {
   public:
    Sc(size_t pwm_pin_, size_t dir_pin1_,size_t dir_pin2_, size_t encoder_out_a_pin, size_t encoder_out_b_pin, uint16_t encoder_resolution, float outmax_, float outmin_);
    
    //Motor(size_t pwm_pin_, size_t dir_pin1_, size_t dir_pin2_,/* size_t en_1_pin, size_t en_2_pin, size_t xy_pin,*/ size_t oe_pin, size_t sel_1_pin, size_t sel_2_pin, size_t reset_x_pin, /*size_t reset_y_pin,*/ size_t bit0, size_t bit1,size_t bit2, size_t bit3, size_t bit4, size_t bit5, size_t bit6, size_t bit7,  float outmax_, float outmin_);
    ~Sc();


    size_t encoder_out_a_pin;
    size_t encoder_out_b_pin;
    uint16_t encoder_resolution;
    /*!
     * @brief set the speed of motor in rpm
     * \param cmd_speed the commanded speed in rpm
     * \return the actual speed in rpm
     */
    float set_speed(float cmd_speed);
    void setup();
    void reset();
    void outSignal(float pwm);
    void PIDvelocity_algorithm(float speed,float Kp,float Ki,float Kd,float delta_T);
    int64_t read_encoder();
    void testing_encoder();

   private:
    
    
    size_t pwm_pin_;
    size_t dir_pin1_;
    size_t dir_pin2_;
    int64_t tick_;
    int64_t tick_enc_;
    int64_t last_tick_enc_;
    int64_t last2_tick_enc_;
    int64_t deriv_comp_;
    float omega_;
    float omega_read_;
    float omega_read_k_;
    float omega_read_k_1_;
    float omega_read_k_2_;
    float omega_read_k_3_;
    float omega_read_k_4_;
    float omega_read_refined;
    float omega_input_;
    float last_omega_;
    float mv_;
    float iTerm_;
    float delta_;
    float error_;
    float last_error_;
    float error_over_time_;
    float kp_;
    float ki_;
    float kd_;
    float outmax_;
    float outmin_;
    uint8_t data_;
    float ocr_;
    float e_k_;
    float e_k_1_;
    float e_k_2_;
    float U_t_;
    float U_t_1_;

    crim::TwoPhaseIncrementalEncoder* encoder_;

  };

}// namespace trui

#endif