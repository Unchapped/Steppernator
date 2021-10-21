
#include <avr/interrupt.h>

#define IN0 9
#define IN0_MASK _BV(PORTB1)
#define IN1 10
#define IN1_MASK _BV(PORTB2)

#define PWM_MIN 820
#define PWM_MAX 1524

int map_pwms(int time_micros) {
  int value = map(time_micros, PWM_MIN, PWM_MAX, 0, 100);
  if (value < 0) return 0;
  if (value > 100) return 100;
  return value;
}

volatile uint8_t prev_portb_value, debug;
volatile struct {unsigned long start; unsigned long pulselen;} in0, in1;

void setup() {
  Serial.begin(9600);
  pinMode(IN0, INPUT);
  pinMode(IN1, INPUT);

  //enable pin change interrupts pwm inputs
  cli();
  PCICR |= _BV(PCIE0); //0b00000001;    //enable pin change interrupts for Port B:
  PCMSK0 |= _BV(PCINT1) | _BV(PCINT2); // turn on pin PB1, and PB2, which are physical pins 9 and 10
  sei();

}

//Pin Change Interrupt
ISR(PCINT0_vect){
  uint8_t portb_value = PINB;
  uint8_t mask = portb_value ^ prev_portb_value;
  if (mask & IN0_MASK) { //pin9
    if(portb_value & IN0_MASK) in0.start = micros(); //high, pulse start
    else in0.pulselen = micros() - in0.start; //low, pulse stop
  }
  if (mask & IN1_MASK) { //pin10
    if(portb_value & IN1_MASK) in1.start = micros(); //high, pulse start
    else in1.pulselen = micros() - in1.start; //low, pulse stop
  }
  prev_portb_value = portb_value;
}

void loop() {
  Serial.print(in0.pulselen);
  Serial.print(", ");
  Serial.println(in1.pulselen);
  delay(10);
}
