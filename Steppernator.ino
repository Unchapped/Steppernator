

//Pin Change Interrupt settings, for reading 2 channels of servo PWM in on pins 9 and 10
#include <avr/interrupt.h>
#define IN_X 9
#define IN_X_MASK _BV(PORTB1)
#define IN_Y 10
#define IN_Y_MASK _BV(PORTB2)
volatile uint8_t prev_portb_value, debug;
volatile struct {unsigned long start; unsigned long pulselen;} inX, inY;

//Servo PWM Calibration
#define PWM_MIN 820
#define PWM_MAX 1524

//Motor Pins
#define X_DIR 5
#define X_STEP 2
#define Y_DIR 6
#define Y_STEP 3 

#define MOTOR_MAX 800
#define MOTOR_MID 400
#define MOTOR_MIN 0

int x_pos, y_pos, x_target, y_target; 

//Helper function to clip and map PWM values to motor positions
int pwm_to_steps(int time_micros) {
  int value = map(time_micros, PWM_MIN, PWM_MAX, MOTOR_MIN, MOTOR_MAX);
  if (value < MOTOR_MIN) return MOTOR_MIN;
  if (value > MOTOR_MAX) return MOTOR_MAX;
  return value;
}


void setup() {
  Serial.begin(9600);
  pinMode(IN_X, INPUT);
  pinMode(IN_Y, INPUT);

  //enable pin change interrupts pwm inputs
  cli();
  PCICR |= _BV(PCIE0); //0b00000001;    //enable pin change interrupts for Port B:
  PCMSK0 |= _BV(PCINT1) | _BV(PCINT2); // turn on pin PB1, and PB2, which are physical pins 9 and 10
  sei();
  delay(40); //wait for initial PWM readings to come in.
    
  //Configure motor pins
  pinMode(X_DIR, OUTPUT);
  pinMode(X_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  x_pos, y_pos, x_target, y_target = MOTOR_MID; //initialize in the middle of the range.
}

//Pin Change Interrupt
ISR(PCINT0_vect){
  uint8_t portb_value = PINB;
  uint8_t mask = portb_value ^ prev_portb_value;
  if (mask & IN_X_MASK) { //pin9
    if(portb_value & IN_X_MASK) inX.start = micros(); //high, pulse start
    else inX.pulselen = micros() - inX.start; //low, pulse stop
  }
  if (mask & IN_Y_MASK) { //pinY0
    if(portb_value & IN_Y_MASK) inY.start = micros(); //high, pulse start
    else inY.pulselen = micros() - inY.start; //low, pulse stop
  }
  prev_portb_value = portb_value;
}

void loop() {
  x_target = pwm_to_steps(inX.pulselen);
  uint8_t dir = (x_pos > x_target);
  uint8_t step = (x_pos != x_target);
  if(dir) x_pos -= step;
  else x_pos += step;
  digitalWrite(X_DIR, dir);
  digitalWrite(X_STEP, step);
  
  y_target = pwm_to_steps(inY.pulselen);
  dir = (y_pos > y_target);
  step = (y_pos != y_target);
  if(dir) y_pos -= step;
  else y_pos += step;
  digitalWrite(Y_DIR, dir);
  digitalWrite(Y_STEP, step);
  
  delayMicroseconds(100);
  digitalWrite(X_STEP, LOW);
  digitalWrite(Y_STEP, LOW);
}
