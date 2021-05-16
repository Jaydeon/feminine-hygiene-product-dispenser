/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);
const int buttonPin=A1;
const int LIGHT_PIN = A0;  // Pin connected to voltage divider output
const int LED_PIN = 13; // Use built-in LED as dark indicator
// Measure the voltage at 5V and the actual resistance of the 4.7k ohm (can be 1k or 10k too) resistor, and enter them below:
const float VCC = 4.98;  // Measured voltage of Arduino 5V line
const float R_DIV = 4660.0; // Measured resistance of 3.3k resistor

// Set this to the minimum resistance required to turn an LED on:
const float DARK_THRESHOLD = 10000.0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(200);  // 10 rpm   
  pinMode(buttonPin, INPUT);
   Serial.begin(9600);
  pinMode(LIGHT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
}

void loop() { 
  
 
  //delay(2000);
 int buttonState = analogRead(buttonPin);
  Serial.println(String(buttonState));
    // Read the ADC, and calculate voltage and resistance from it
  int lightADC = analogRead(LIGHT_PIN);
  digitalWrite(LED_PIN, HIGH);
  if (lightADC > 0)
  {
    float lightV = lightADC * VCC / 1023.0;
    float lightR = R_DIV * (VCC / lightV - 1.0);
    Serial.println("Voltage: " + String(lightV) + " V");
    Serial.println("Resistance: " + String(lightR) + " ohms");
    Serial.println("button V: " + String(buttonState) + "V");

     if (lightR < 1000) 
      {
      }
     if (buttonState > 10) 
      {
        myMotor->step(256, FORWARD, DOUBLE);
        
        Serial.println("Double coil steps");
        delay(2000);
        myMotor->step(256, BACKWARD, DOUBLE);
        
      }

    Serial.println();
    delay(500);
  }
}
