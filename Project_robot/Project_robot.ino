// DUSONG, ADELINO JR. J.
// COE185.1 PROJECT: WALLFOLLOWING ROBOT


#include <Servo.h>
#include <avr/io.h>
#include <util/delay.h>

#define SERVO_PIN 10
Servo servo;

volatile int front_distance;
volatile int right_distance;
volatile bool has_move_back;

float checkdistance() {

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

void setMotorSpeed(int leftWheel, int rightWheel) {
  for (int i = 0; i < 256; ++i) {
    
    if (i < rightWheel) {
      PORTD |= (1 << PORTD6);
      _delay_us(1000);
      PORTD &= ~(1 << PORTD6);
      _delay_us(1000);
    } else {
      PORTD &= ~(1 << PORTD6);
    }

    if (i < leftWheel) {
      PORTD |= (1 << PORTD5);
      _delay_us(1000);
      PORTD &= ~(1 << PORTD5);
      _delay_us(1000);
    } else {
      PORTD &= ~(1 << PORTD5);
    }

  }
}

void moveforward(uint8_t pwm) {
    PORTD |= (1 << PD2);
    PORTD &= ~(1 << PD4);
    setMotorSpeed(pwm, pwm);
}

void movebackward(uint8_t pwm) {
    PORTD &= ~(1 << PD2);
    PORTD |= (1 << PD4);
    setMotorSpeed(pwm, pwm);
}

void rotateleft(uint8_t pwm) {
    PORTD &= ~(1 << PD2) & ~(1 << PD4);
    setMotorSpeed(pwm, pwm);
}

void rotateright(uint8_t pwm) {
    PORTD |= (1 << PD2) | (1 << PD4);
    setMotorSpeed(pwm, pwm);
}

void stop() {
    PORTD &= ~(1 << PD2);
    PORTD |= (1 << PD4);
    setMotorSpeed(0, 0);    
}

void setup(){
  Serial.begin(9600);
  sei();

  DDRD |= (1 << DDD2);
  DDRD |= (1 << DDD4);
  DDRD |= (1 << DDD5);
  DDRD |= (1 << DDD6);
  DDRB |= (1 << DDB4);
  DDRB &= ~(1 << DDB5);

  servo.attach(SERVO_PIN);

  front_distance = 0;
  right_distance = 0;

  servo.write(90);
  // servo.write(0);

  _delay_ms(2000);
}

void loop(){

  // SCAN DISTANCE //

  // right_distance = checkdistance();
  // _delay_ms(500);
  // servo.write(90);
  // _delay_ms(500);
  // front_distance = checkdistance();
  // _delay_ms(500);
  // servo.write(0);
  // _delay_ms(500);

  right_distance = checkdistance();
  _delay_ms(250);
  servo.write(90);
  _delay_ms(600);
  front_distance = checkdistance();
  _delay_ms(250);
  servo.write(0);
  _delay_ms(250);

  Serial.print("Front Distance: ");
  Serial.print(front_distance);
  Serial.println(" cm");
  Serial.print("Right Distance: ");
  Serial.print(right_distance);
  Serial.println(" cm");
  Serial.println("");  

  // CHECK IF ALREADY CLOSE TO WALL //

  has_move_back = false;
  if (front_distance <= 4) {
    movebackward(210);
    stop();
    if (right_distance > 40) {
      rotateleft(135);
      _delay_ms(100);
      stop();
    }
    has_move_back = true;
  } else if (front_distance > 4 && front_distance <= 8) {
    movebackward(140);
    stop();
    if (right_distance > 40) {
      rotateleft(135);
      _delay_ms(100);
      stop();
    }
    has_move_back = true;
  }

  // PRIMARY MOVEMENT //

  if (has_move_back == false) {
    if (right_distance > 60 ) { // no wall on right
      moveforward(160);
      rotateright(115);
      _delay_ms(100);
      rotateleft(10);
      moveforward(30);      
      moveforward(200);
      _delay_ms(200);
      moveforward(200);
      _delay_ms(200);
    } 
    else if (right_distance > 45 && right_distance <= 60) { // far from wall, get closer
      rotateright(120);
      _delay_ms(100);
      moveforward(145);
      _delay_ms(100);
      rotateleft(115);
    } else if (right_distance <= 10) {
      movebackward(35);
      rotateright(20);
      movebackward(35);
      rotateright(20); 
      movebackward(35);
      rotateleft(120);
    } else if (right_distance > 10 && right_distance <= 45 ) { // maintaining distance

      if (front_distance > 20) {
        moveforward(150);
        _delay_ms(100);
      } else if (front_distance <= 20 && front_distance > 8) {
        rotateleft(120);
        _delay_ms(100);
      } else if (front_distance <= 8) {
        movebackward(100);
        _delay_ms(100);
        rotateleft(160);
        _delay_ms(100);
      }

    }
  }

  _delay_ms(150);
  // END MOVEMENT AND REPEAT AGAIN //

}