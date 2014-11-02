#include <arduino/Arduino.h>
#include <sensor/two_phase_incremental_encoder.hpp>
#include <Wire.h>
#include "stdint.h" 
#include <pid/pid.h>


#define K_P     1.00
#define K_I     0.00
#define K_D     0.00
#define motorPin  9//11
#define motorCS   10//12
#define OUTMAX    100.0
#define OUTMIN    -100.0
#define initSetpoint 0;

//! Parameters for regulator
struct PID_DATA pidData;

/*! \brief Flags for status information
 */
struct GLOBAL_FLAGS {
  //! True when PID control loop should run one time
  uint8_t pidTimer:1;
  uint8_t dummy:7;
} gFlags = {0, 0};

using namespace std;

long tick = 0, tickEnc = 0, last_tickEnc = 0, last2_tickEnc = 0; 
long derivComp, omega=0, omega_input=0, i = 0;
int tickDirFlag = 1,dirFlag = 1,movingFlag = 0,baseSub;
float MV=0, iTerm = 0;    
float delta=0, error=0, last_error=0, last_omega=0; 
unsigned char data=0;
float Kp = 0.750, Ki = 0.005, Kd = 0;
float Gain = 250;
float tetha = 0,tethaEnc=0,tethaMem=0,degMoved=0;
long now_tickTest=0, tickTest=0,last_tickTest=0;
long timeOtherNew = 0, timeOtherOld = 0;
int lastDir = 0;
uint8_t buffer[10];
int received_data;


void setup() {
  pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);
  pinMode(motorCS, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(12, INPUT);
  digitalWrite(12, HIGH);// turn on pullup resistor
  delay(500);
  Serial.begin(115200);
  Serial.println("Let's Rock!");
  delay(500);
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
    int read_encoder(){
      return encoder_->pos();
    }

    void testing_encoder(){
      while (true) {
        Serial.println("===");
        Serial.println(static_cast<long int>(encoder_->pos()));
        Serial.println(encoder_->rot());
        delay(100);
      }
    }


    void set_rpm(int rpm_motor) {
      if(rpm_motor < 0) dirFlag = -1; else dirFlag = 1;
      omega_input = abs(rpm_motor); //setpoint
      tickEnc = encoder_->pos();
      omega = abs(tickEnc - last_tickEnc)*2/3; // Tetha = ((tickEnc - last_tickEnc)/250) * 2 * PI rad
                                             // omega = Tetha / Delta_Time -- Delta_Time = 50ms
                                             // omega = Tetha / 50ms = ((tickEnc - last_tickEnc)/250) * 2 * PI * 1000/50 rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI * (1/(2PI)) rotation/rad rad/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/(1/60) minute
                                             // omega = (tickEnc - last_tickEnc) * 2/25 * 60 rotation/minute
                                             // omega = (tickEnc - last_tickEnc) * 24/5 RPM
      //Serial.println(omega);
      error = omega_input - omega;
      iTerm = iTerm + (float)error*Ki;       //Integral Term of PID Control 
      
      if(iTerm > OUTMAX) iTerm = OUTMAX;
      else if(iTerm < OUTMIN) iTerm = OUTMIN;
      //i = i + error;    
                    
      derivComp = (tickEnc - 2*last_tickEnc + last2_tickEnc)*10/3; //Wonder why the derivative differentiates encoders instead of error
                    
      MV =  (float)error*Kp + iTerm - derivComp*Kd;
      
      if(MV > OUTMAX) MV = OUTMAX;
      else if(MV < OUTMIN) MV = OUTMIN;
                    
      motorPWM_percentage(dirFlag * MV); 
           
      last_error = error;
      last2_tickEnc = last_tickEnc; 
      last_tickEnc = tickEnc;

      //printf("RPM yang dikirim: %d\n", rpm_motor);
      //printf("Nilai MV: %d\n", MV);
      //Serial.print(error);
      //Serial.print(" ");
      //Serial.print(tickEnc);
      //Serial.print(" ");
      //Serial.print(rpm_motor);
      //Serial.print(" ");
      //Serial.println(MV);

    }

    int get_rpm() {
      if(omega < 0) dirFlag = -1; else dirFlag = 1;
      omega_input = abs(rpm_motor); //setpoint
      tickEnc = encoder_->pos();
      omega = abs(tickEnc - last_tickEnc)*2/3; // Tetha = ((tickEnc - last_tickEnc)/250) * 2 * PI rad
                                             // omega = Tetha / Delta_Time -- Delta_Time = 50ms
                                             // omega = Tetha / 50ms = ((tickEnc - last_tickEnc)/250) * 2 * PI * 1000/50 rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI rad/s
                                             // omega = (tickEnc - last_tickEnc) * 4/25 * PI * (1/(2PI)) rotation/rad rad/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/s
                                             // omega = (tickEnc - last_tickEnc) * 2/25 rotation/(1/60) minute
                                             // omega = (tickEnc - last_tickEnc) * 2/25 * 60 rotation/minute
                                             // omega = (tickEnc - last_tickEnc) * 24/5 RPM
      //Serial.println(omega);
      error = omega_input - omega;
      iTerm = iTerm + (float)error*Ki;       //Integral Term of PID Control 
      
      if(iTerm > OUTMAX) iTerm = OUTMAX;
      else if(iTerm < OUTMIN) iTerm = OUTMIN;
      //i = i + error;    
                    
      derivComp = (tickEnc - 2*last_tickEnc + last2_tickEnc)*10/3; //Wonder why the derivative differentiates encoders instead of error
                    
      MV =  (float)error*Kp + iTerm - derivComp*Kd;
      
      if(MV > OUTMAX) MV = OUTMAX;
      else if(MV < OUTMIN) MV = OUTMIN;
                    
      motorPWM_percentage(dirFlag * MV); 
           
      last_error = error;
      last2_tickEnc = last_tickEnc; 
      last_tickEnc = tickEnc;

      //printf("RPM yang dikirim: %d\n", rpm_motor);
      //printf("Nilai MV: %d\n", MV);
      //Serial.print(error);
      //Serial.print(" ");
      //Serial.print(tickEnc);
      //Serial.print(" ");
      //Serial.print(rpm_motor);
      //Serial.print(" ");
      //Serial.println(MV);

    }

    void set_degPControl(int deg_motor) {
      tethaEnc = encoder_->pos();
      if(tickDirFlag == -1) degMoved = baseSub - tethaEnc;
      else if(tickDirFlag == 1) degMoved = tethaEnc - baseSub;
      error = tickDirFlag*(deg_motor - degMoved);
      iTerm = iTerm + (float)error*Ki;       //Integral Term of PID Control 
      
      if(iTerm > OUTMAX) iTerm = OUTMAX;
      else if(iTerm < OUTMIN) iTerm = OUTMIN;
                    
      if (error > 5){MV = (float)(error*Kp+iTerm)*tickDirFlag;}
      else if(error < -5){MV =  (float)(error*Kp+iTerm)*tickDirFlag;}
      else MV = 0;
      if(MV > 255) MV = 255;
      else if(MV < 13 && MV > 0) MV = 13;
      else if(MV > -13 && MV < 0) MV = -13; //Minimum PWM so make the motor move, identified experimentally
      else if(MV < -255) MV = -255;
      motorPWM_percentage(MV);
        
      // Serial.print("tickTest: ");
      // Serial.print(tickTest);
      // Serial.print(" now_tickTest: ");
      // Serial.print(now_tickTest);
      // Serial.print(" DirFlag: ");
      // Serial.print(tickDirFlag);
      // Serial.print(" MV: ");
      // Serial.print(MV);
      // Serial.print("Error: ");
      // Serial.print(error);
      // Serial.print(" Base: ");
      // Serial.print(baseSub);
      // Serial.print("Tetha: ");
      // Serial.print(tetha);
      // Serial.print(" DegMoved: ");
      // Serial.print(degMoved);
      // Serial.print(" tethaEnc: ");
      // Serial.println(tethaEnc);
   
    }

    void set_degPTOS(int deg_motor) {

      //fPTOS = 

      tethaEnc = encoder_->pos();
      if(tickDirFlag == -1) degMoved = baseSub - tethaEnc;
      else if(tickDirFlag == 1) degMoved = tethaEnc - baseSub;
      error = tickDirFlag*(deg_motor - degMoved);
      


                    
      if(error >= 250) MV = (float)(Gain*tickDirFlag);
      else if(error <= -250) MV = (float)(Gain*tickDirFlag*(-1));
      else if (error < 250 && error > 5){MV = (float)(error*Kp/*+iTerm*/)*tickDirFlag;}
      else if(error < -5 && error > -250){MV =  (float)(error*Kp/*+iTerm*/)*tickDirFlag;}
      else if(error <= 5 && error >= -5) MV = 0;
      
      if(MV > 255) MV = 255;
      else if(MV < 15 && MV > 0) MV = 15;
      else if(MV > -15 && MV < 0) MV = -15; //Minimum PWM so make the motor move, identified experimentally
      else if(MV < -255) MV = -255;
      motorPWM_percentage(MV);
        
      Serial.print("tickTest: ");
      Serial.print(tickTest);
      Serial.print(" now_tickTest: ");
      Serial.print(now_tickTest);
      Serial.print(" DirFlag: ");
      Serial.print(tickDirFlag);
      Serial.print(" MV: ");
      Serial.print(MV);
      Serial.print("Error: ");
      Serial.print(error);
      Serial.print(" Base: ");
      Serial.print(baseSub);
      Serial.print("Tetha: ");
      Serial.print(tetha);
      Serial.print(" DegMoved: ");
      Serial.print(degMoved);
      Serial.print(" tethaEnc: ");
      Serial.println(tethaEnc);
   
    }


    void servoInit()
    {
      long data;
      int timeNow,timeOld;
      data = digitalRead(12);
      
      motorPWM_percentage(-25);
      while(data)
      {
      data = digitalRead(12);
      delay(5);
      Serial.print("Pin 12 :"); Serial.print(data);
      
      Serial.print(" tethaEnc :"); Serial.println(read_encoder());
      if(digitalRead(12) == 0)
        {
          Serial.println("Hit origin");
          baseSub = read_encoder();
          motorPWM_percentage(0);
        }//motor.set_rpm(0);//analogWrite(11,0);
      }
      Serial.println("Starting encoder direction test");
      motorPWM_percentage(25); //move the motor
      tickTest = read_encoder(); //read tick
 
      while (1) {
        timeNow = millis();
        if(timeNow - timeOld > 50){
        timeOld = timeNow;
        now_tickTest = read_encoder();
        if(abs(now_tickTest - tickTest) >=100) break;
        }
      }
 
      motorPWM_percentage(0);
      if((now_tickTest - tickTest) > 0) {
        tickDirFlag = 1;
        }//right way
        else {
          tickDirFlag = -1;
          }//wrong way

      data = digitalRead(12);
      motorPWM_percentage(-25);
      while(data){//Return back to origin before doing anything
        data = digitalRead(12);
        delay(1);
        if(digitalRead(12) == 0)
        {
          movingFlag = 1;
          motorPWM_percentage(0);
        }
      }

    }

    
};





void receiveEvent(int howMany)
{
  int i = 0;
  while(0 < Wire.available()) // loop through all
  {
    buffer[i] = Wire.read(); // receive byte as a character
    i++;
  }

  int checksum= (buffer[8] << 8);
  checksum |= buffer[7];
  if(checksum == (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[9])) Serial.println("OK");
  else Serial.println("ERROR");

  // received_data = (buffer[6] << 8);
  // received_data |= (buffer[5]);

  // Serial.println(received_data);

}


void motorPWM_percentage(signed int pwm)
    {
      int ocr;
      
      if(pwm>255) pwm = 255;
      else if(pwm<-255) pwm = -255;
      
      ocr = (float)pwm;
      
      if(pwm>0)
      {
        digitalWrite(motorCS, LOW);
        analogWrite(motorPin, ocr);
        lastDir = 1;
      }
      else if(pwm<0)
      {
        digitalWrite(motorCS, HIGH);
        analogWrite(motorPin, -ocr);
        lastDir = 0;
      }
      else if (pwm == 0) {
        if(lastDir == 1) {digitalWrite(motorCS,HIGH);}
        else if(lastDir == 0) {digitalWrite(motorCS,LOW);}
          analogWrite(motorPin, 0);
        }
      
      //Serial.println(ocr);
    }

void setup()
{
  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 780;
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect)        // interrupt service routine 
{
  //interrupts();
  gFlags.pidTimer = TRUE;
}

int main() {
  
  init();
  // setup();
  // Motor motor;

  Serial.begin(9600);
  Wire.begin(15); // 15 for slave1, 9 for slave2
  Wire.onReceive(receiveEvent);


  // float x;// a speed order from master
  // long timeNow = 0, timeOld = 0;
  // long setPOINT;
  // int counter=0, countSetP=0;
  // long timeStamp = 0;
   motor.servoInit();
   setPOINT = initSetpoint;
  // delay(1000);

  while (1) {
    // timeNow = millis();
    
    // if(timeNow - timeOld > 50){
    //   timeOld = timeNow;
    //   counter++;
    //   if(counter == 100) {setPOINT = random(10,900); counter = 0;}
    //   if(movingFlag == 1){
    //   motor.set_degPControl(setPOINT);}
      
    //   Serial.print("setPOINT : "); Serial.print(setPOINT);
    //   Serial.print(" counter : "); Serial.println(counter);    
   if(gFlags.pidTimer)
    {
      referenceValue = Get_Reference();
      measurementValue = Get_Measurement();

      inputValue = pid_Controller(referenceValue, measurementValue, &pidData);

      Set_Input(inputValue);

      gFlags.pidTimer = FALSE;
    }
    // timeNow = millis();
    // if(timeNow - timeOld > 1){
    //   timeOld = timeNow;
    //   counter++;
    //   if(counter == 200) {
    //     // if(setPOINT >= 900) {setPOINT = 300;}
    //     // else  setPOINT=setPOINT+100;
    //     if(countSetP == 1) setPOINT = 900;
    //     else setPOINT = 300;
    //     counter = 0;
    //     countSetP = !countSetP;
    //     }
    //   if(movingFlag == 1){
    //   timeStamp = millis();
    //   motor.set_degPControl(setPOINT);}
    //   timeStamp = millis() - timeStamp;
    //   Serial.print("timeStamp : "); Serial.print(timeStamp);
    //   Serial.print(" setPOINT : "); Serial.print(setPOINT);
    //   Serial.print(" counter : "); Serial.println(counter);

    // }


  }
  
}