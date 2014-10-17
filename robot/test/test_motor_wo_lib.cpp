#include <arduino/Arduino.h>
#include <sensor/two_phase_incremental_encoder.hpp>

#define motorPin  11
#define motorCS   12
#define OUTMAX    100.0
#define OUTMIN    -100.0

using namespace std;

long tick = 0, tickEnc = 0, last_tickEnc = 0, last2_tickEnc = 0; 
long derivComp;
float omega=0, omega_input=0, last_omega=0;
float MV=0, iTerm = 0;    
float delta=0, error=0, last_error=0; 
unsigned char data=0;
float Kp = 0.315, Ki = 0.0528, Kd = 0;


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
      const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called
      encoder_ = new crim::TwoPhaseIncrementalEncoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);
    }
    ~Motor() {
      delete encoder_;
    }

    void testing_encoder(){
      while (true) {
        Serial.println("===");
        Serial.println(static_cast<long int>(encoder_->pos()));
        Serial.println(encoder_->rot());
        delay(100);
      }
    }

    void set_rpm(float rpm_motor) {
      omega_input = rpm_motor; //setpoint
      tickEnc = encoder_->pos();

      omega = (float)(tickEnc - last_tickEnc)*24/5; // Tetha = ((tickEnc - last_tickEnc)/250) * 2 * PI rad
                                             // omega = Tetha / Delta_Time -- Delta_Time = 50ms
                                             // omega = Tetha / 50ms = ((tickEnc - last_tickEnc)/250) * 2 * PI * 1000/50 rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI * (1/(2PI)) rotation/rad rad/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/(1/60) minute
                                             // omega = (tickEnc - last_tickEnc) * 2/25 * 60 rotation/minute
                                             // omega = (tickEnc - last_tickEnc) * 24/5 RPM

      error = omega_input - omega;
      iTerm = iTerm + (float)error*Ki;       //Integral Term of PID Control 
      
      if(iTerm > OUTMAX) iTerm = OUTMAX;
      else if(iTerm < OUTMIN) iTerm = OUTMIN;
                    
      derivComp = (tickEnc - 2*last_tickEnc + last2_tickEnc)*24/5;
                    
      MV =  (float)error*Kp + iTerm - derivComp*Kd;
      
      if(MV > OUTMAX) MV = OUTMAX;
      else if(MV < OUTMIN) MV = OUTMIN;
                    
      motorPWM_percentage(MV); 
           
      last_error = error;
      last2_tickEnc = last_tickEnc; 
      last_tickEnc = tickEnc;

      //Serial.println(omega);
      //Serial.println(tickEnc);
      //Serial.println(rpm_motor);
      //Serial.println(MV);
    }

    void motorPWM_percentage(float pwm)
    {
      int ocr;
      if(pwm>100) pwm = 100;
      else if(pwm<-100) pwm = -100;
      
      ocr = (float)pwm*255/100;
      
      if(pwm>=0)
      {
        digitalWrite(motorCS, LOW);
        analogWrite(motorPin, ocr);
      }
      else if (pwm<0)
      {
        digitalWrite(motorCS, HIGH);
        analogWrite(motorPin, -ocr);
      }
      else
      {
        analogWrite(motorPin, 0);
      }

      //Serial.println(ocr);
    }
};

int main() {
  init();
  setup();
  Motor motor;

    float speed;
  int counter=0;
  bool countSetP=0;
  long timeNow = 0, timeOld = 0;

  //motor.testing_encoder();

 while (1) {
    timeNow = millis();
    if(timeNow - timeOld > 50){
      
      timeOld = timeNow;
      motor.set_rpm(-50);
      //Serial.print(speed);
//      Serial.print(" ");      
//      Serial.println(countSetP);
    }  
 }
  
}
