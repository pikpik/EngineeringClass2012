#include <Servo.h>

int gyroX, gyroY, gyroOffsetX, gyroOffsetY;

int servos [ 2 ] = { 0, 0 };

Servo s0;
Servo s1;


/*void updateSpeeds () {
  
  servos [ 0 ] += speedUpOrDown ( servos [ 0 ], steps [ step ] [ 0 ] );
  servos [ 1 ] += speedUpOrDown ( servos [ 1 ], steps [ step ] [ 1 ] );
  
  //saySpeeds ();
  useSpeeds ();
  
}*/


int speedUpOrDown ( int start, int finish ) {
  
  return (finish == start) ?
    0
  :
    ( finish - start ) < 0 ?
      -1
    :
      1
  ;
  
}


void resetESC ( int servo ) {
  
  setSpeed ( servo, 0 );
  
  useSpeeds ();
  
}


void setSpeed ( int servo, int speed ) {
  
  servos [ servo ] = speed;
  
}

void saySpeeds () {
  
  Serial.print ( "Servo speeds: " );
  Serial.print ( servos [ 0 ] );
  Serial.print ( ", " );
  Serial.print ( servos [ 1 ] );
  Serial.print ( ", " );
  Serial.print ( servos [ 2 ] );
  Serial.print ( ", " );
  Serial.println ( servos [ 3 ] );
  
}


void useSpeeds () {
  
  saySpeeds ();
  
  s0.write ( servos [ 0 ] );
  s1.write ( servos [ 1 ] );
  
  delay ( 15 );
  
}


void resetGyro () {
  
  gyroOffsetX = analogRead ( 0 );
  gyroOffsetY = analogRead ( 1 );
  
}

void readGyro () {
  
  gyroX = analogRead ( 0 ) - gyroOffsetX - 1;
  gyroY = analogRead ( 1 ) - gyroOffsetY - 1;
  
  /*
  Serial.print ( "Gyro: " );
  Serial.print ( gyroX, DEC );
  Serial.print ( "," );
  Serial.println ( gyroY, DEC );
  */
  
}


void adjustOrientation () {
  
  if ( gyroX < 0 ) {
    
    servos [ 0 ]++;
    servos [ 1 ]--;
    
  } else if ( gyroX > 0) {
    
    servos [ 0 ]--;
    servos [ 1 ]++;
    
  }
  
  useSpeeds ();
  
}


void setup () {
  
  Serial.begin ( 9600 );
  
  s0.attach ( 8 );
  s1.attach ( 9 );
  
  resetESC ( 0 );
  resetESC ( 1 );
  
  resetGyro ();
  
}


// This loops until the Arduino is turned off.

void loop () {
  // This finds out what our orientation is.
  readGyro ();
  
  // This attempts to control the motors using the orientation.
  adjustOrientation ();
  
  // This code allows for synchronized steps.
  //doStep ();
  
}
