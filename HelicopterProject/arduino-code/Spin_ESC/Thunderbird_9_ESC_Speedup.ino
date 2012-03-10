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

int speed = 0;

#define resetESC() \
  chooseSpeed(0);

#define chooseSpeed(v) \
  if ( v > speed ) { \
    rampUpSpeed(v); \
  } else { \
    rampDownSpeed(v); \
  }

#define rampUpSpeed(v) \
  for (; speed < v; speed++) { \
    useSpeed(); \
  }

#define rampDownSpeed(v) \
  for (; speed > v; speed--) { \
    useSpeed(); \
  }

#define useSpeed() \
  Serial.println(speed); \
  myservo.write(speed); \
  delay(15);

// Start

void setup () {
  
  Serial.begin(9600); // Talk to the computer.
  
  myservo.attach(9);  // The servo is on pin 9.
  
  //resetESC(); // Start the ESC at a speed of 0.
              // This resolves any problems,
              // but makes the motor stop.
  
  resetESC();
  
  //chooseSpeed(10);
  
}


// This loops until the Arduino is turned off.
void loop () {
  
  //chooseSpeed(10);
  
  //delay(5000);
  
  for (int j = 0; j <= 5000; j++) {
    
    chooseSpeed(j);
    
    delay(10);
    
  }
  
}
