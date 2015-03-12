#include "encoder_hctl.h"

using namespace crim;

int64_t EncoderHctl::result_new_ = 0;
int64_t EncoderHctl::result_old_x_ = 0;
int64_t EncoderHctl::result_old_y_ = 0;
int32_t EncoderHctl::result_ = 0;
int8_t  byteRead_ = 0;

int8_t EncoderHctl::result_hi_ =0;
int8_t EncoderHctl::result_2nd_ =0;
int8_t EncoderHctl::result_3rd_ =0;
int8_t EncoderHctl::result_lo_ =0;



EncoderHctl::EncoderHctl(/*size_t en_1_pin, size_t en_2_pin, size_t xy_pin,*/ size_t oe_pin, size_t sel_1_pin, size_t sel_2_pin, size_t reset_x_pin, /*size_t reset_y_pin,*/ size_t bit0, size_t bit1,size_t bit2, size_t bit3, size_t bit4, size_t bit5, size_t bit6, size_t bit7)
    : /*en_1_pin_(en_1_pin), en_2_pin_(en_2_pin), xy_pin_(xy_pin),*/ oe_pin_(oe_pin), sel_1_pin_(sel_1_pin), sel_2_pin_(sel_2_pin), reset_x_pin_(reset_x_pin), /*reset_y_pin_(reset_y_pin),*/ bit0_(bit0), bit1_(bit1), bit2_(bit2), bit3_(bit3), bit4_(bit4), bit5_(bit5), bit6_(bit6), bit7_(bit7) {

  //pinMode(en_1_pin_, OUTPUT);
  //pinMode(en_2_pin_, OUTPUT);
  //pinMode(xy_pin_, OUTPUT);
  pinMode(oe_pin_, OUTPUT);
  pinMode(sel_1_pin_, OUTPUT);
  pinMode(sel_2_pin_, OUTPUT);

  pinMode(bit0_, INPUT);
  pinMode(bit1_, INPUT);
  pinMode(bit2_, INPUT);
  pinMode(bit3_, INPUT);
  pinMode(bit4_, INPUT);
  pinMode(bit5_, INPUT);
  pinMode(bit6_, INPUT);
  pinMode(bit7_, INPUT);
  // DDRB = B00000000;

  //digitalWrite(en_1_pin_, HIGH); //EN1 = 1
  //digitalWrite(en_2_pin_, LOW); //EN2 = 0
  //digitalWrite(xy_pin_, LOW); //Select X-axis

}

EncoderHctl::~EncoderHctl() {

}

int32_t EncoderHctl::pos() {
  digitalWrite(oe_pin_, HIGH); //Disable OE
  delay(25);
  
  digitalWrite(oe_pin_, LOW); //Enable OE
  digitalWrite(sel_1_pin_, LOW); //SEL1 = 0 (MSB)
  digitalWrite(sel_2_pin_, HIGH); //SEL2 = 1 (MSB)
  get_hi(); //Get MSB

  digitalWrite(sel_1_pin_, HIGH); //SEL1 = 1 (2nd Byte) 
  digitalWrite(sel_2_pin_, HIGH); //SEL2 = 1 (2nd Byte) 
  get_2nd(); //Get 2nd Byte

  digitalWrite(sel_1_pin_, LOW); //SEL1 = 0 (3rd Byte) 
  digitalWrite(sel_2_pin_, LOW); //SEL2 = 0 (3rd Byte) 
  get_3rd(); //Get 3rd Byte

  digitalWrite(sel_1_pin_, HIGH); //SEL1 = 1 (LSB) 
  digitalWrite(sel_2_pin_, LOW); //SEL2 = 0 (LSB) 
  get_lo(); //Get LSB

  digitalWrite(oe_pin_, HIGH); //Disable OE
  delay(25);

  result_ = (result_hi_ << 24); //Assign MSB
  result_ |= (result_2nd_ << 16); //Assign 2nd Byte
  result_ |= (result_3rd_ << 8); //Assign 3rd Byte
  result_ |= result_lo_; //Assign LSB

  return result_;
}

int8_t EncoderHctl::readByte(){
  byteRead_  = digitalRead(bit7_);
  byteRead_ |= (digitalRead(bit6_) << 6);
  byteRead_ |= (digitalRead(bit5_) << 5);
  byteRead_ |= (digitalRead(bit4_) << 4);
  byteRead_ |= (digitalRead(bit3_) << 3);
  byteRead_ |= (digitalRead(bit2_) << 2);
  byteRead_ |= (digitalRead(bit1_) << 1);
  byteRead_ |= digitalRead(bit0_);
  //(digitalRead(bit7_)<<7 | digitalRead(bit6_)<<6 | digitalRead(bit5_)<<5 | digitalRead(bit4_)<<4 | digitalRead(bit3_)<<3 | digitalRead(bit2_)<<2 | digitalRead(bit1_)<<1 | digitalRead(bit0_));
  return byteRead_;
}

void EncoderHctl::get_hi() {
  hi_old_ = readByte();
  hi_new_ = readByte();
  if (hi_new_ == hi_old_){
    result_hi_ = hi_new_;
  } else {
   get_hi();
  }
}

void EncoderHctl::get_2nd() {
  second_old_ = readByte();
  second_new_ = readByte();
  if (second_new_ == second_old_){
    result_2nd_ = second_new_;
  } else {
    get_2nd();
  }
}

void EncoderHctl::get_3rd() {
  third_old_ = readByte();
  third_new_ = readByte();
  if (third_new_ == third_old_){
    result_3rd_ = third_new_;
  } else {
    get_3rd();
  }
}

void EncoderHctl::get_lo() {
  lo_old_ = readByte();
  lo_new_ = readByte();
  if (lo_new_ == lo_old_){
    result_lo_ = lo_new_;
  } else {
    get_lo();
  }
}