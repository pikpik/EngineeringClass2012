// This program makes the simpler ESC gradually
// increase and then decrease its speed
// indefinitely.

#include <Servo.h> 
 
// Create a servo objects to control the servos.

Servo servoA;
Servo servoB;


// Individual servo speed variables.

int servoASpeed = 0;
int servoBSpeed = 0;


// The speed can be 71 - ~150.
// 60 resets the ESC (which is very important).

#define ESCResetSpeed  60
#define ESCEndSpeed    158


// Functions for motor control.

void resetESCs() {

  servoASpeed = 0;
  
  servoBSpeed = 0;
  
  setSpeeds ();

}


void rampSpeeds () {
  
  
  // Send the ESC's the speeds for their motors.
  
  servoA.write ( servoASpeed );
  
  servoB.write ( servoBSpeed );
  
  
  // There must be at least a 15 millisecond delay between updates of the ESC's speed setting.
  // If there isn't, the ESC will be too busy listening to be able to tell the motor what to do.
  
  //delay ( 15 );
  
}


void setSpeeds () {
  
  debugSpeeds();
  
  
  // Send the ESC's the speeds for their motors.
  
  servoA.write ( servoASpeed );
  
  servoB.write ( servoBSpeed );
  
  
  // There must be at least a 15 millisecond delay between updates of the ESC's speed setting.
  // If there isn't, the ESC will be too busy listening to be able to tell the motor what to do.
  
  //delay ( 15 );
  
}


void debugSpeeds () {
  
  // Tell us what's going on.
  
  // (Use the Serial Monitor to see messages.
  // The Arduino will restart when it is opened.)
  
  Serial.print ( "Speeds: " );
  Serial.print ( servoASpeed );
  Serial.print ( ", " );
  Serial.print ( servoBSpeed );
  Serial.println ( );
  
}


// Start.

// This happens only once: when the Arduino is turned on or restarted.

void setup () {
  
  Serial.begin ( 9600 ); // Talk to the computer.
  
  
  // Assign the servos to digital pins on the Arduino.
  
  servoA.attach ( 8 );
  servoB.attach ( 9 );
  
  
  // Start the ESC at the reset position.
  
  // This resolves any problems,
  // but makes the motor stop.
  
  // This is necessary whenever the ESC goes out of its safe range
  // and enters its fail-safe mode to protect itself.
  
  resetESCs ( );
  
  
  //chooseSpeed(10);
  
}


// This loops until the Arduino is turned off.

void loop () {
  
  //chooseSpeed(10);
  
  //delay(5000);
  
  for ( servoASpeed = 0, servoBSpeed = 160; servoASpeed <= 160; servoASpeed++, servoBSpeed-- ) {
    
    //rampSpeeds ();
    
    setSpeeds ();
    
    delay ( 500 );
    
  }
  
}
