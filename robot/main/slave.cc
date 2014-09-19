#include <arduino/Arduino.h>
#include <sensor/two_phase_incremental_encoder.hpp>

#define motorPin  13
#define motorCS   12
#define OUTMAX    100.0
#define OUTMIN    -100.0

using namespace std;

long tick = 0, tickEnc = 0, last_tickEnc = 0, last2_tickEnc = 0; 
long derivComp, omega=0, omega_input=0, i = 0;
float MV=0, iTerm = 0;    
int delta=0, error=0, last_error=0, last_omega=0; 
unsigned char data=0;
float Kp = 0.316, Kd = 0.0, Ki = 0.0528;

void setup() {
  pinMode(motorCS, OUTPUT);
  pinMode(motorPin, OUTPUT);
  Serial.begin(115200);
}

class Motor {
  private:
    crim::TwoPhaseIncrementalEncoder* encoder_;
    
  public:
    Motor() {
      const int encoder_out_a_pin = 2;
      const int encoder_out_b_pin = 3;
      const int encoder_resolution = 360;
      encoder_ = new crim::TwoPhaseIncrementalEncoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);
    }
    ~Motor() {
      delete encoder_;
    }

    // while (true) {
    //   Serial.println("===");
    //   Serial.println(static_cast<long int>(encoder.pos()));
    //   Serial.println(encoder.rot());
    //   delay(100);
    // }

    void set_rpm(int rpm_motor) {
      omega_input = rpm_motor; //setpoint
      //tickEnc = tick;
      tickEnc = encoder_->pos();
      omega = (tickEnc - last_tickEnc)*10/3; // in RPM *10/3; // deltaCount/10e-3 = deltaCount*100
      error = omega_input - omega;        
            
      iTerm = iTerm + (float)error*Ki; 
      
      if(iTerm > OUTMAX) iTerm = OUTMAX;
      else if(iTerm < OUTMIN) iTerm = OUTMIN;
      //i = i + error;    
                    
      derivComp = (tickEnc - 2*last_tickEnc + last2_tickEnc)*10/3;
                    
      MV =  (float)error*Kp + iTerm - derivComp*Kd;
      
      if(MV > OUTMAX) MV = OUTMAX;
      else if(MV < OUTMIN) MV = OUTMIN;
                    
      motorPWM(MV); 
           
      last_error = error;
      last2_tickEnc = last_tickEnc; 
      last_tickEnc = tickEnc;
    }

    void motorPWM(signed int pwm)
    {
      int ocr;
      if(pwm>100) pwm = 100;
      else if(pwm<-100) pwm = -100;
      
      ocr = (float)pwm*255/100;
      
      if(pwm>=0)
      {
        digitalWrite(motorCS, HIGH);
        analogWrite(motorPin, ocr);
      }
      else
      {
        digitalWrite(motorCS, LOW);
        analogWrite(motorPin, ocr);
      }
    }
};

int main() {
  init();
  setup();
  Motor motor;

  float x;// a speed order from master
  long timeNow = 0, timeOld = 0;

  while (1) {
    timeNow = millis();
    if(timeNow - timeOld > 50){
      timeOld = timeNow;
      motor.set_rpm(10);
    }  
  }
  
}