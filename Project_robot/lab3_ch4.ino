#include <avr/io.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define FRONT_LED PB0 // Pin 8 GREEN_LED
#define RIGHT_LED PB1 // Pin 9 WHITE LED
#define LEFT_LED PB3 // Pin 11 GREEN_LED

#define SERVO_PIN PB2 // Pin10

#define TRIGGER_PIN PB4 // Pin 12
#define ECHO_PIN PB5 // Pin 13

float front_distance = 0;
float right_distance = 0;
float left_distance = 0;
float proximity_distance_front = 2.5;
float proximity_distance_right = 4;
float proximity_distance_left = 4;

void initServo() {
    // Initialize servo
    DDRB |= (1 << SERVO_PIN);

    // Set Timer1 for Fast PWM mode with OCR1A as Top
    TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1); // Fast PWM mode 15 and output on OC1B (digital Pin 10) 
    TCCR1B = (1 << WGM13) | (1 << WGM12);

    // Set prescaler for Timer1 to 64
    TCCR1B |= (1 << CS11) | (1 << CS10);
    TCCR1B &= ~(1 << CS12);
    OCR1A = 4999; // (15000000 / (64 * 50)) - 1
}

void initUltrasonic() { //reached and corrected
    // Set trigger pin as output
    DDRB |= (1 << TRIGGER_PIN);
    // Clear Trigger
    PORTB &= ~(1 << TRIGGER_PIN);
    // Set echo pin as input
    DDRB &= ~(1 << ECHO_PIN);
}

void initLEDs() {
    DDRB |= (1 << FRONT_LED);
    DDRB |= (1 << RIGHT_LED);
    DDRB |= (1 << LEFT_LED);    
    PORTB &= ~(1 << FRONT_LED);
    PORTB &= ~(1 << RIGHT_LED);
    PORTB &= ~(1 << LEFT_LED);        
}

void servoMove( int pulseWidth) {
  OCR1B = pulseWidth;
}

float checkDistance() {

    PORTB |= (1 << PINB4);
    delay(10);
    PORTB &= ~(1 << PINB4);

    while (!(PINB & (1 << PINB5)));
    TCNT1 = 0;

    while (PINB & (1 << PINB5)) {
    if (TCNT1 > 60000) {
      // Timeout to prevent infinite loop
      return -1.0; // Return -1.0 to indicate timeout
      }
    }

    float distance = (float) TCNT1 / 58.4;

    return (distance / 2);
}

// double checkDistance() { // reached and corrected
//     double duration, distance;
//     // Trigger the sensor
//     PORTB |= (1 << PINB4);
//     _delay_us(40);
//     PORTB &= ~(1 << PINB4);
    
//     // Wait for echo start
//     while(!(PINB & (1 << PINB5)));

//     TCNT1 = 0; // timer
//     // Wait for echo end
//     while(PINB & (1 << PINB5)); // Wait while the pin is high
//     duration = TCNT1; // The current value of TCNT1 is stored in ICR1

//     distance = duration * 0.02700787; // this is measured in cm; the formula should match the prescaler of the servo w/c is 64
 
//     return distance;
// }

void work() {

    servoMove(360);
    _delay_ms(800);
    left_distance = checkDistance();  

    Serial.print("Left_Distance:");
    Serial.print(left_distance);
    Serial.println(",");    
    if (left_distance <= proximity_distance_left) {
      PORTB |= (1 << FRONT_LED);
    }
    else {
      PORTB &= ~(1 << FRONT_LED);
    }   
    _delay_ms(2000);
    servoMove(360);
    _delay_ms(800);
    front_distance = checkDistance();  
    Serial.print("Front_Distance:");
    Serial.print(front_distance);
    Serial.print(",");    
    if (front_distance <= proximity_distance_front) {
      PORTB |= (1 << FRONT_LED);
      PORTB |= (1 << RIGHT_LED);
    }
    else {
      PORTB &= ~(1 << FRONT_LED);
      PORTB &= ~(1 << RIGHT_LED);
    }   

    _delay_ms(2000);
    // servoMove(90);
    _delay_ms(800);
    right_distance = checkDistance();  

    Serial.print("Right_Distance:");
    Serial.print(right_distance);
    Serial.println(",");    
    if (right_distance <= proximity_distance_right) {
      PORTB |= (1 << RIGHT_LED);
    }
    else {
      PORTB &= ~(1 << RIGHT_LED);
    }   
    _delay_ms(2000);
}

void setup(){
  Serial.begin(9600);
  sei();
  initUltrasonic();
  initServo();
  initLEDs();

  front_distance = 0;
  right_distance = 0;

  // Serial.println("hello");
  servoMove(360); // Move servo to 90 degrees
  _delay_ms(1500);
}

void loop(){
  work();
}