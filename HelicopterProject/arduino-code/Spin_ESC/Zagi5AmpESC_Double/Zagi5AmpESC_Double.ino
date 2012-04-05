#include <Servo.h>

int servos [4] = { 0, 0, 0, 0 };

Servo s0;
Servo s1;
//Servo s2;
//Servo s3;

#define ESCResetSpeed 60
#define ESCEndSpeed 158

int step = 0;
int totalSteps = 7;

// Each row is a step.
// The first four columns are servo speeds.
// The fifth column is the duration to hold that speed.

int steps [7][5] = {
  { 0, 0, 0, 0, 1000 },
  { 70, 70, 0, 0, 500 },
  { 100, 100, 0, 0, 600 },
  { 120, 80, 0, 0, 600 },
  { 70, 140, 0, 0, 600 },
  { 150, 150, 0, 0, 600 },
  { 0, 0, 0, 0, 4000 }
};


void doStep () {
  
  Serial.print ( "Step #" );
  Serial.println ( step );
  
  if (
  
    servos [ 0 ] == steps [ step ] [ 0 ] &&
    servos [ 1 ] == steps [ step ] [ 1 ] &&
    servos [ 2 ] == steps [ step ] [ 2 ] &&
    servos [ 3 ] == steps [ step ] [ 3 ]
    
  ) {
    
    delay ( steps [ step ] [ 4 ] );
    
    step++;
    
    if ( step >= totalSteps ) step = 0;
    
  } else {
    
    servos [ 0 ] += speedUpOrDown ( servos [ 0 ], steps [ step ] [ 0 ] );
    servos [ 1 ] += speedUpOrDown ( servos [ 1 ], steps [ step ] [ 1 ] );
    servos [ 2 ] += speedUpOrDown ( servos [ 2 ], steps [ step ] [ 2 ] );
    servos [ 3 ] += speedUpOrDown ( servos [ 3 ], steps [ step ] [ 3 ] );
    
    //saySpeeds ();
    useSpeeds ();
    
  }
  
}


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
  
  //saySpeeds ();
  
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
  resetESC ( 2 );
  resetESC ( 3 );
  
}


// This loops until the Arduino is turned off.

void loop () {
  
  /*for ( int s = 0; s <= 100; s++ ) {
    
    setSpeed ( 0, s );
    setSpeed ( 1, s );
    setSpeed ( 2, s );
    setSpeed ( 3, s );
    
    useSpeeds ();
    
    delay ( 10 );
    
  }*/
  
  doStep ();
  
}
