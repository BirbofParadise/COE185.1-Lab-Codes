// Define pin number for the servo motor
const int servoPin = 9; // Digital pin for controlling the servo motor

void setup() {
    // Set pin mode for the servo pin
    pinMode(servoPin, OUTPUT);

    // Initialize Timer1 for PWM generation
    TCCR1A = 0; // Reset Timer1 control register A
    TCCR1B = 0; // Reset Timer1 control register B

    // Set Timer1 to Fast PWM mode with non-inverted output (Clear on Compare Match - COM1A1)
    // Prescaler of 8 (CS11)
    TCCR1A |= (1 << WGM10) | (1 << WGM11) | (1 << COM1A1);
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
    
    // Set initial servo position to 90 degrees (midpoint)
    OCR1A = 128; // 50% duty cycle (for 8-bit Timer, 0-255 range)

    // Initialize Serial communication
    Serial.begin(9600);
}

void loop() {
    // Sweep the servo from 0 to 180 degrees in steps of 90 degrees
    for (int angle = 0; angle <= 180; angle += 90) {
        int dutyCycle = map(angle, 0, 180, 51, 205);
        OCR1A = dutyCycle; // Set PWM duty cycle
        
        delay(1500); // Delay for servo to reach the position
    }
}