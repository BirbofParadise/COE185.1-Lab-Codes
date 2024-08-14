// DUSONG, ADELINO JR. J.
// COE185.1 LAB 3: CHECKOFF #2 SERVO MOTOR

#include <avr/io.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define FRONT_LED PB0 // Pin 8 GREEN_LED
#define RIGHT_LED PB1 // Pin 9 WHITE LED

#define SERVO PB2 // Pin10

#define TRIGGER_PIN PB4 // Pin 12
#define ECHO_PIN PB5 // Pin 13

void setupServo() {
    // Initialize servo
    DDRB |= (1 << SERVO);

    // Set Timer1 for Fast PWM mode with OCR1A as Top
    TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1); // Fast PWM mode 15 and output on OC1B (digital Pin 10) 
    TCCR1B = (1 << WGM13) | (1 << WGM12);

    // Set prescaler for Timer1 to 64
    TCCR1B |= (1 << CS11) | (1 << CS10);
    TCCR1B &= ~(1 << CS12);
    OCR1B = 375;
    OCR1A = 4999; // (15000000 / (64 * 50)) - 1
}

void moveServo( int pulseWidth) {
  OCR1B = pulseWidth; // Set the pulse width on OC1RB
}

void work() {
    moveServo(125); // right
    _delay_ms(1000);
    moveServo(375); // front
    _delay_ms(1000);
    moveServo(625); // left
    _delay_ms(1000);
    moveServo(375); // front
    _delay_ms(1000);
}

void setup(){
  Serial.begin(9600);
  sei();
  setupServo();

  // Serial.println("hello");
  moveServo(625); // Move servo to left
  _delay_ms(1500);
}

void loop(){
  work();
}