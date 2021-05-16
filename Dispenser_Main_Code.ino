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
Adafruit_StepperMotor *myMotor2 = AFMS.getStepper(200, 1);
const int buttonPin=A1;
const int LIGHT_PIN=A0;
const int buttonPin2=A2;
const int LIGHT_PIN2 = A3;  // Pin connected to voltage divider output
const int LEDpin = 12;
const int LEDpin2 = 13; // Use built-in LED as dark indicator
const int thermo = 11;
const int thermo2 = 10;
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
  myMotor2->setSpeed(200);   
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
   Serial.begin(9600);
  pinMode(LIGHT_PIN, INPUT);
  pinMode(LEDpin, OUTPUT);
  pinMode(LEDpin2, OUTPUT);
  pinMode(thermo, OUTPUT);
  pinMode(thermo2, OUTPUT);
}

void loop() { 
  
 
  //delay(2000);
 int buttonState = analogRead(buttonPin);
 int buttonState2 = analogRead(buttonPin2);
  Serial.println(String(buttonState));
  Serial.println(String(buttonState2));
    // Read the ADC, and calculate voltage and resistance from it
     if (buttonState > 100) 
  {
        myMotor->step(256, FORWARD, DOUBLE);
        Serial.println("Tampon motor");
        delay(2000);
        myMotor->step(256, BACKWARD, DOUBLE);
        digitalWrite(LEDpin, HIGH);
        int lightADC = analogRead(LIGHT_PIN);
  if (lightADC > 0)
    {
      float lightV = lightADC * VCC / 1023.0;
      float lightR = R_DIV * (VCC / lightV - 1.0);
      Serial.println("Voltage1: " + String(lightV) + " V");
      Serial.println("Resistance1: " + String(lightR) + " ohms");
      Serial.println("button V: " + String(buttonState) + "V");
      if (lightR < 1000)
      {
        digitalWrite(thermo, HIGH);
        delay(500);
        digitalWrite(thermo, LOW);
      }
      delay(1000); 
      digitalWrite(LEDpin, LOW); 
      delay(7000);  
    }
  }     
     if (buttonState2 >100)
      {
        myMotor2->step(256, FORWARD, DOUBLE);
        Serial.println("Pad motor");
        delay(2000);
        myMotor2->step(256, BACKWARD, DOUBLE);
        digitalWrite (LEDpin2, HIGH);
        int lightADC2 = analogRead(LIGHT_PIN2);
     if (lightADC2 > 0)
      {
        float lightV2 = lightADC2 * VCC / 1023.0;
        float lightR2 = R_DIV * (VCC / lightV2 - 1.0);
        Serial.println("Voltage2: " + String(lightV2) + "V");
        Serial.println("Resitance2: " + String(lightR2) + " ohms");
        Serial.println("button2 V: " + String(buttonState2) + "V");
        if (lightR2 < 1000)
        {
          digitalWrite(thermo2, HIGH);
          delay(1000);
          digitalWrite(thermo2, LOW);
        }
        delay(500);
        digitalWrite(LEDpin2, LOW);
        delay(7000);
      }
      }
    Serial.println();
    delay(500);
}
