#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
NEW SKETCH
34567891011121314151617182019
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#define PWM_TOP (39999u)
#define PWM_MIN (1999u)
#define PWM_MX (3999u)


#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#define PWM_TOP (39999u)
#define PWM_MIN (1999u)
#define PWM_MX (3999u)


// void servoRun(uint16_t pwm) {

//   OCR4C = (pwm*22.22) + 1000;
//   decimel0(OCR4C);

//   // if (pwm = 0) {

//   //   PORTB |= (1 << PB2);
//   //   _delay_us(1500);
//   //   PORTB &= ~(1 << PB2);
//   //   _delay_us(18500);
//   // } else if (pwm = 90) {
//   //   PORTB |= (1 << PB2);
//   //   _delay_us(2000);
//   //   PORTB &= ~(1 << PB2);
//   //   _delay_us(18000);
//   // } else if (pwm = -90) {
//   //   PORTB |= (1 << PB2);
//   //   _delay_us(1000);
//   //   PORTB &= ~(1 << PB2);   
//   //   _delay_us(19000);
//   // }

//   // for (int i = 0; i < 256; ++i) {
//   //   if (i < pwm) {
//   //     PORTB |= (1 << PB2);
//   //     _delay_us(1000);
//   //     PORTB &= ~(1 << PB2);
//   //     _delay_us(1000);
//   //   } else {
//   //     PORTB &= ~(1 << PB2);
//   //   }
//   // }
//   // int pulseWidth = map(pwm, 0, 180, 1000, 2000);

//   // PORTB |= (1 << PB2);
//   // Serial.println(pulseWidth);
//   // delayMicroseconds(pulseWidth);
//   // // for (int i = 0; i < pulseWidth; i++) {
//   // //   Serial.println(i);
//   // //   _delay_us(1);
//   // // }

//   // PORTB &= ~(1 << PB2);
//   // delayMicroseconds(20000 - pulseWidth);
// }

ISR(TIMER1_0VF_vect) {
  volatile static uint8_t update_pwm_ready = 0;
}

ISR(TIMER1_COMPA_vect) {

}

void update_pwm(uint16_t i) {
  update_pwm_ready = 1;
  while(update_pwm_ready != 0);
}

void setup() {
  Serial.begin(9600);
  sei();

  DDRB |= (1 << DDB1);

  TIMSK1 = (1 << TOIE1) | (1 << OCIE1A);
  ICR1H = (PWM_TOP && 0xFF00) >> 8;
  ICR1L = (PWM_TOP && 0x00FF);

  OCR1AH = (PWM_MIN && 0xFF00) >> 8;
  OCR1AL = (PWM_MIN && 0xFF00);

  TCCR1A = (0b10 << COM1A0) | (0b00 << COM1B0) | (0b10 << WGM10);

  TCCR1B = (0b11 << WGM12) | (0b010 << CS10);

  // DDRB |= (1 << DDB2);
  // DDRB |= (1 << DDB4);
  // DDRB &= ~(1 << DDB5);

  // TCCR4A |= (1 << WGM41);
  // TCCR4B |= (1 << WGM42) | (1 << WGM43);

  // TCCR4B |= (4 << CS41);
  // ICR4 = 40000;

  // TCCR4A |= (1 << COM4C1);

  Serial.println("Starting");
  servoRun(0);
  _delay_ms(100);
}

void loop() {

  Serial.println("Restart");
}
