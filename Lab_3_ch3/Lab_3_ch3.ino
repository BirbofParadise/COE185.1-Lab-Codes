// DUSONG, ADELINO JR. J.
// COE185.1 LAB 3: CHECKOFF #3 PROXIMITY DETECTION

#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>

#define TRIGGER_PIN PB4 // PIN 12 trigger on sensor
#define ECHO_PIN PB5     // PIN 13 echo on sensor

#define WHITE_LED PB0 // Pin 8 GREEN_LED LEFT LED
#define BLUE_LED PB1 // Pin 9 WHITE LED RIGHT LED

#define SERVO PB2 // PIN 10 

double distance = 0;

double left_threshold = 4.0;
double right_threshold = 4.0;
double front_threshold = 7.0;

void setup() {
  Serial.begin(9600);
  setupServo();
  setupLEDs();
  setupUltrasonic();
  Serial.println("done setup");
  proximityRead(125); // Start Right

}

void setupLEDs() {
    DDRB |= (1 << WHITE_LED);
    DDRB |= (1 << BLUE_LED);
    PORTB &= ~(1 << WHITE_LED);
    PORTB &= ~(1 << BLUE_LED);     
}

void loop() { // CHECKOFF 3
  resetLED();
  Serial.print("Left Distance:" );
  Serial.println(proximityRead(625)); // LEFT
  delay(500);

  resetLED();
  Serial.print("Center Distance:" );
  Serial.println(proximityRead(375)); // CENTER
  delay(500);

  resetLED();
  Serial.print("Right Distance:" );
  Serial.println(proximityRead(125)); // RIGHT
  delay(500);
}

void setupUltrasonic() {
    // Set trigger pin as output
    DDRB |= (1 << TRIGGER_PIN);
    // Clear Trigger
    PORTB &= ~(1 << TRIGGER_PIN);
    // Set echo pin as input
    DDRB &= ~(1 << ECHO_PIN);
}

double measureDistance() {
    double duration, distance;
    // Trigger the sensor
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);
    
    // Wait for echo start
    while(!(PINB & (1 << ECHO_PIN)));

    TCNT1 = 0; // timer
    // Wait for echo end
    while(PINB & (1 << ECHO_PIN)); // Wait while the pin is high
    duration = TCNT1; // The current value of TCNT1 is stored in ICR1

    // distance = duration * 0.02700787; // this is measured in cm; the formula should match the prescaler of the servo w/c is 64

    distance = duration * 0.02700787; // this is measured in inches; the formula should match the prescaler of the servo w/c is 64
    
    return (distance);
}

void setupServo() {
    DDRB |= (1 << SERVO);
    
    // Configure Timer1 for Fast PWM mode with OCR1A as TOP
    TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1); // Set Fast PWM mode 15 and output on OC1B (Digital Pin 10)
    TCCR1B = (1 << WGM13) | (1 << WGM12);
    
    // Set prescaler for Timer1 to 64
    TCCR1B |= (1<<CS11) | (1<<CS10);
    TCCR1B &= ~(1<<CS12);
    OCR1B = 375;
    OCR1A = 4999; //(16000000 / (64 * 50)) - 1
}


void moveServo(int pulseWidth) {
  OCR1B = pulseWidth; // Set the pulse width on OC1RB
}

void resetLED() {
  PORTB &= ~(1 << WHITE_LED);
  PORTB &= ~(1 << BLUE_LED);
}

void turnOnLEDBlue() {
  PORTB |= (1 << BLUE_LED);
}

void turnOnLEDWhite() {
  PORTB |= (1 << WHITE_LED);
}

double proximityRead(int direction) {
  moveServo(direction);
  delay(300);

  distance = measureDistance();

  delay(60);
  if (direction == 625) { // left
    if (distance <= left_threshold){
      turnOnLEDWhite();
    }
  } else if (direction == 375) { // front
    if (distance <= front_threshold){
      turnOnLEDWhite();
      turnOnLEDBlue();
    }
  } else if (direction == 125) { // right
    if (distance <= right_threshold){
      turnOnLEDBlue();
    }
  }
  return measureDistance();
}