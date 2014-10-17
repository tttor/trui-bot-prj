#include <arduino/Arduino.h>

int timer1_counter;

void setup()
{
  pinMode(13, OUTPUT);

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 15624;
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
  digitalWrite(13, digitalRead(13) ^ 1);
}

int main() {
  init();
  setup();
  
  while(1);
}

