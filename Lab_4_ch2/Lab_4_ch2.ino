// DUSONG, ADELINO JR. J.
// COE185.1 LAB 4: CHECKOFF #2 ROBOT MOTOR MOVEMENTS

#include <avr/io.h>
#include <util/delay.h>

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

  _delay_ms(2000);
}

void loop() {
  // Forward
  moveforward(200);
  _delay_ms(500);

  // Backward
  movebackward(200);
  _delay_ms(500);

  // Small Turn Left
  rotateleft(90);
  _delay_ms(500);
  
  // Small Turn Right
  rotateright(90);
  _delay_ms(500);
  
  // Large Turn Left
  rotateleft(160);
  _delay_ms(500);

  // Large Turn Right
  rotateright(160);
  _delay_ms(500);  

}
