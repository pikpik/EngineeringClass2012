// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 

// The speed can be 71 - 160.
// 60 resets the ESC (which is very important).
#define ESCResetSpeed  60
#define ESCEndSpeed  158

int val = 0;    // variable to read the value from the analog pin 

#define resetESC() \
  myservo.write(ESCResetSpeed); \
  delay(500);

#define chooseSpeed(v) \
  Serial.println(v); \
  myservo.write(v); \
  delay(500);

void setup() 
{
  
  Serial.begin(9600); // Talk to the computer.
  
  myservo.attach(9);  // The servo is on pin 9.
  
  resetESC(); // Start the ESC at 0.
  
} 
 
void loop() 
{
  
  for ( int i = 80; i < 150; i += 10 ) {
    chooseSpeed(i);
  }
  
  for ( int i = 150; i > 80; i -= 10 ) {
    chooseSpeed(i);
  }
  
} 
