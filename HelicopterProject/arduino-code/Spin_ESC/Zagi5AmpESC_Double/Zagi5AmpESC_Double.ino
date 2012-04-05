#include <Servo.h>

//int servos [1] = { 0 };
int servos [2] = { 0, 0 };
//int servos [4] = { 0, 0, 0, 0 };

Servo s0;
Servo s1;
//Servo s2;
//Servo s3;

#define ESCResetSpeed 60
#define ESCEndSpeed 158


void resetESC ( int servo ) {
  
  setSpeed ( servo, 0 );
  
  useSpeeds ();
  
}


void setSpeed ( int servo, int speed ) {
  
  Serial.print ( "Servo, speed: " );
  Serial.print ( servo );
  Serial.print ( ", " );
  Serial.println ( speed );
  
  servos [ servo ] = speed;
  
}


void useSpeeds () {
  
  s0.write ( servos [ 0 ] );
  s1.write ( servos [ 1 ] );
  //s2.write ( servos [ 2 ] );
  //s3.write ( servos [ 3 ] );
  
  delay ( 15 );
  
}


void setup () {
  
  Serial.begin ( 9600 );
  
  s0.attach ( 8 );
  s1.attach ( 9 );
  //s2.attach ( 10 );
  //s3.attach ( 11 );
  
  resetESC ( 0 );
  resetESC ( 1 );
  //resetESC ( 2 );
  //resetESC ( 3 );
  
}


// This loops until the Arduino is turned off.

void loop () {
  
  for ( int s = 0; s <= 100; s++ ) {
    
    setSpeed ( 0, s );
    setSpeed ( 1, s );
    
    useSpeeds ();
    
    delay ( 10 );
    
  }
  
}
