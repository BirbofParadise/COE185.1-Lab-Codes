// DUSONG, ADELINO JR. J.
// COE185.1 LAB 2

#include <Arduino.h>
#include <avr/io.h>

// Definitions for Port B & D registors
#define PINB_Reg (*((volatile uint8_t *) 0x23))
#define DDRB_Reg (*((volatile uint8_t *) 0x24))
#define PORTB_Reg (*((volatile uint8_t *) 0x25))
#define PIND_Reg (*((volatile uint8_t *) 0x29))
#define DDRD_Reg (*((volatile uint8_t *) 0x2a))
#define PORTD_Reg (*((volatile uint8_t *) 0x2b))
// Definitions for LED assignments:
#define BOARD_LED 5   //pin 13 is PortB bit 5
#define RED_LED 1     //pin 9 is PortB bit 1  
#define GREEN_LED 2  //pin 10 is PortB bit 2
#define BLUE_LED 3   //pin 11 is PortB bit 3

#define BUTTON 5 
#define TCNTBUT 8
int lastTCNT1 = 0;

bool buttonPressed = false;
bool buttonPressed2 = true;
uint32_t buttonPressTime = 0;
uint32_t buttonBounceEndTime = 0;
uint32_t buttonReleaseTime = 0;
int buttonPressCount = 0;

uint32_t buttonReleaseTimeBefore = 0;
uint32_t buttonReleaseTimeAfter = 0;
uint32_t bounceDuration = TCNT1;

uint32_t tcntstart = 0;
uint32_t tcntend = 0;
unsigned long freq = 16000000;

void LEDInit(){
 //Set pinmode for LEDs to output 
  DDRB_Reg |= (1 << BOARD_LED);
  DDRB_Reg |= (1 << RED_LED);
  DDRB_Reg |= (1 << GREEN_LED);
  DDRB_Reg |= (1 << BLUE_LED);

  //Turn all off
  PORTB_Reg &= ~(1 << BOARD_LED); //clear output
  PORTB_Reg &= ~(1 << RED_LED);   //clear output
  PORTB_Reg &= ~(1 << GREEN_LED); //clear output
  PORTB_Reg &= ~(1 << BLUE_LED);  //clear output

  //Test LEDs
  Serial.println("Testing LEDs...");
  PORTB_Reg |= (1 << BOARD_LED);  //set output
  PORTB_Reg |= (1 << RED_LED);    //set output
  delay(400);
  PORTB_Reg &= ~(1 << RED_LED);   //clear output
  PORTB_Reg |= (1 << GREEN_LED);  //set output
  delay(400);
  PORTB_Reg &= ~(1 << GREEN_LED); //clear output
  PORTB_Reg |= (1 << BLUE_LED);   //set output
  delay(400);
  PORTB_Reg &= ~(1 << BLUE_LED);   //clear output
  PORTB_Reg &= ~(1 << BOARD_LED);   //clear output
  Serial.println("Finished LED testing!");
  }
void setup() {                
  Serial.begin(9600);
  Serial.println("Starting up...");
  LEDInit();
  //Set pinmode for Button as input
  DDRD_Reg &= ~(1 << BUTTON);
  //Enable pullup 
  PORTD_Reg |= (1 << BUTTON);  //set output to enable pullup resistor
  //Set pinmode for Button as input
  pinMode(TCNTBUT, INPUT_PULLUP);
  //Init counter1
  TCCR1A = 0; //Normal mode 0xffff top, rolls over

  //CHECKOFF 1
  // TCCR1B |= (1 << CS12) | (1 << CS11); //Clock T1 falling edge #CHECKOFF1!!!
  //
  // TCCR1B = (1 << CS10); //Clock T1 falling edge, x1 Prescaler
  // TCCR1B = (1 << CS11); //Clock T1 falling edge, x8 Prescaler
  TCCR1B = (1 << CS11) | (1 << CS10); //Clock T1 falling edge, x64 Prescaler #MAINPRESCALER!!!
  // TCCR1B = (1 << CS12); //Clock T1 falling edge, x256 Prescaler
  // TCCR1B = (1 << CS12) | (1 << CS10); //Clock T1 falling edge, x1024 Prescaler
  TCCR1C = 0;
  //Set counter to zero, high byte first
  TCNT1H = 0;
  TCNT1L = 0;  
  //Make sure interrupts are disabled 
  TIMSK1 = 0;
  TIFR1 = 0;
  // Clear Input Capture Register 1 (ICR1)
  ICR1 = 0;
  
  Serial.println("Finished setup!");
}

void loop() { // Insert function
  checkoffFive();
}

// CHECKOFF 1

bool detectTCNT1Changed(int currentTCNT1) {           // Checks if TCNT1's value changed by comparing the current TCNT1 from the last
  return currentTCNT1 != lastTCNT1;
}
bool detectButtonBounced(int currentTCNT1, int lastValue) { // Checks if a bounce has occured by checking if the 
  return (currentTCNT1 - lastValue) > 1;                    // difference of the current and last TCNT1 is greater than 1.
}

void checkoffOne() {
  int currentTCNT1 = TCNT1;
  if (detectTCNT1Changed(currentTCNT1) ) {
    if (detectButtonBounced(currentTCNT1, lastTCNT1)) {
      Serial.println("Bounce Detected!");
    } else {
      Serial.print("TCNT1: ");
      Serial.println(currentTCNT1);
    }
    lastTCNT1 = currentTCNT1;
  }
  _delay_ms(100);
}

// CHECKOFF 2
// Remember to change Prescaler
void checkoffTwo() {

  TCCR1B = 1;

  Serial.print("TCNT1: ");
  Serial.println(TCNT1);

  while ((TIFR1 & (1 << TOV1)) == 0);

  TCCR1B = 0;
  float overflowTime = (float) (65535 + 1) / (freq/1024)*1000;

  Serial.print("Time: ");
  Serial.println(overflowTime);

  TIFR1 |= (1 << TOV1);

} // Through the Timer-Prescaler equation:
// Time = [(Max Timer Value) / (Clock Frequency)] * (Prescaler Value)

// Prescaler 1: Time = (65535 / 16,000,000) * 1 = 0.0041 seconds
// Pescaler 8: Time = (65535 / 16,000,000) * 8 = 0.033 seconds
// Prescaler 64: Time = (65535 / 16,000,000) * 64 = 0.262 seconds
// Prescaler 256: Time = (65535 / 16,000,000) * 256 = 1.049 seconds
// Prescaler 1024: Time = (65535 / 16,000,000) * 1024 = 4.194 seconds

// The Max Timer Value which is 65535 is the maximum value for Timer1 which is a 16-bit timer 
// Obtained through 2^16 - 1 = 65535. The Atmega328P Arduino Uno is specified to have a clock frequency of 16 MHz (16,000,000 Hz).

// CHECKOFF 3

void checkoffThree() {
  if (digitalRead(TCNTBUT) == LOW) { // Digital Read function detects if button is pressed
    if (!buttonPressed) { // Button is pressed
      buttonPressed = true;
      buttonPressTime = millis(); // Counter for when button is pressed
      TCNT1 = 0; // Clear the timer
      ICR1 = 0;  // Clear Input Capture Register 1 (ICR1)

      delay(100); // Delay a bit before reading ICR1
      buttonBounceEndTime = millis(); // Counter for when the bounce signal ends

      // Bounce sensitivity
      uint32_t bounceDuration = ICR1;

      if (bounceDuration > 0) { // Bounce occurred, print the duration of the bounce
        Serial.print("Bounce Time (ms): ");
        Serial.println(buttonBounceEndTime - buttonPressTime); 
      }
    }
  } else { // Button is released
    if (buttonPressed) {
      buttonPressed = false;
      buttonPressCount++;
      Serial.print("Button Press Count: ");
      Serial.println(buttonPressCount);
      
    }
  }
}

// CHECKOFF 4

void checkoffFour() {
  if (digitalRead(TCNTBUT) == HIGH) { // Digital Read function detects if button is pressed
    if (!buttonPressed2) { // Button is released
      buttonPressed2 = true;
      // buttonPressTime = millis(); // Counter for when button is pressed
      TCNT1 = 0; // Clear the timer
      ICR1 = 0;  // Clear Input Capture Register 1 (ICR1)
      buttonPressTime = ICR1;

      delay(10); // Delay a bit before reading ICR1
      // Bounce sensitivity
      bounceDuration = ICR1;   

      // Check for bounce during pressed
      if (bounceDuration != 0) { // Bounce occurred, print the duration of the bounce
        buttonBounceEndTime = ICR1;
        Serial.print("Released Bounce Time (ms): ");
        Serial.println(buttonBounceEndTime - buttonPressTime); 
      } else {
        buttonPressCount++;
        Serial.print("Button Press Count: ");
        Serial.println(buttonPressCount);   
      }
    }
  } else { 
    if (buttonPressed2) { // Button is pressed 
      buttonPressed2 = false;
    }
  }

}

// CHECKOFF 5

void checkoffFive() {
  if (digitalRead(TCNTBUT) == LOW) { // Digital Read function detects if button is pressed
    if (!buttonPressed) { // Button is pressed
      buttonPressed = true;
      buttonPressTime = millis(); // Counter for when button is pressed
      TCNT1 = 0; // Clear the timer
      ICR1 = 0;  // Clear Input Capture Register 1 (ICR1)
    }
  } else { // Button is released
    if (buttonPressed) {
      buttonPressed = false;
      delay(100); // Delay a bit before reading ICR1
      buttonReleaseTime = millis(); // Counter for when button is released

      // Bounce sensitivity at pressed
      uint32_t bounceDuration = ICR1;   
      
      // Delay for a short time to detect release bounce
      delay(10);

      // Capture bounce sensitivity again at release
      uint32_t newbounceDuration = ICR1;

      // Comparing the two will take into account if a bounce occured when the button is released. Hence this can detect if bounce has
      // occured in both when pressed and when released.
      if (newbounceDuration != bounceDuration) { // Bounce occurred, print the duration of the bounce
        Serial.print("Bounce Time (ms): ");
        Serial.println(buttonReleaseTime - buttonPressTime); 
      
      } else { // No bounce occurred, print the number of button presses
        buttonPressCount++;
        Serial.print("Button Press Count: ");
        Serial.println(buttonPressCount);
      }

      // The duration of the button while being pressed
      Serial.print("Pressed Time (ms): ");
      Serial.println(buttonReleaseTime - buttonPressTime);
      
    }
  }
}