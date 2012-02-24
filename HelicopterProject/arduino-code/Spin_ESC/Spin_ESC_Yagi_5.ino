// This program makes the simpler ESC gradually
// increase and then decrease its speed
// indefinitely.

#include <Servo.h> 
 
Servo myservo;  // Create a servo object to control a servo.


// The speed can be 71 - ~150.
// 60 resets the ESC (which is very important).

#define ESCResetSpeed  60
#define ESCEndSpeed  158


// This was used to maintain speed control throughout the program.
// It isn't used anymore.

int val = 0;


// "Functions" for motor control.

// I need to figure out how to write functions in
// the Arduino language.

#define resetESC() \
  myservo.write(ESCResetSpeed); \
  delay(500);

#define chooseSpeed(v) \
  Serial.println(v); \
  myservo.write(v); \
  delay(500);


// Start

void setup () {
  
  Serial.begin(9600); // Talk to the computer.
  
  myservo.attach(9);  // The servo is on pin 9.
  
  resetESC(); // Start the ESC at a speed of 0.
              // This resolves any problems,
              // but makes the motor stop.
  
}


// This loops until the Arduino is turned off.

void loop () {
  
  // Gradually increase speed.
  
  for ( int i = 80; i < 150; i += 10 ) {
    
    chooseSpeed(i);
    
  }
  
  
  // Gradually decrease speed.
  
  for ( int i = 150; i > 80; i -= 10 ) {
    
    chooseSpeed(i);
    
  }
  
}
