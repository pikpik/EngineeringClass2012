#include <Servo.h>
#include <PID_v1.h>

double gyroX = 0.00,
       gyroY = 0.00;

double goalAngleX = 0.30,
       goalAngleY = 0.30;

double PIDControllerOutputA,
       PIDControllerOutputB;

int speedBase = 55;

int speeds [ 4 ] = { 0, 0, 0, 0 };
int servos [ 4 ] = { 0, 0, 0, 0 };

Servo s0;
Servo s1;

// Sampling every 15 ms
// Originally: 2,5,1
// Fast: 2,100,1
// Almost balancing:
// 4,2,1 good
// 4,0.5,1 better
// 4,0.5,0.5 better
// 8,0.5,0.5 worse
// 2,0.5,0.5 smoother, but not better
// 3,0.5,0.5 great! But it's a little slow at times.
// 3,0.1,0.1 worse!
// 3,0.7,0.7 better
// 3,0.6,0.6 better
// 3,0.5,0.6 not really better
// 3,0.6,0.5 better!
// 3,0.7,0.4 not so good.
// 3,0.4,0.4 it wobbles.
// 3,0.5,0.5 it stabilizes sort of quickly, and then wobbles some.
// 4,0.5,0.5

// Sampling every 1 ms
// 3,0.7,0.1 good, but wobbly
// 4,1,0.1 good, but a little slow and wobbly
// 5,2,0 slow, unable to stabilize
// 5,0,0 slow, slow to stabilize, wobbly
// 3,0,0 even slower
// 3,1,0 slower to stabilize, wobbles less
// 1,0,0 balances and then goes to one side
// 0,1,0 it goes to one side
// 0,0,1 wobbles erratically
// 1,0,1 tries to balance but wobbles erratically
// 1,0,0.5 same
// 1,0,0.25 it stays almost balanced but wobbles, it also can get mostly stuck on one side
// 1,0,0.1 it wobbles nearly stable, but is easily pushed and stays unbalanced
// 1,0.25,0.25

// 1.0, 0.05, 0.05 is pretty good, but slow, wobbly, and off to one side
// 2.0, 0.05, 0.05 is good, but slower, wobbly, and also off to one side
// 2.0, 0.05, 0.1 is faster, but has larger wobbles, and is off to one side

// speed 55
// 2.0, 0.1, 0.1

double Kp = 2.0,
       Ki = 0.1,
       Kd = 0.1;

PID PIDControllerA ( &gyroX, &PIDControllerOutputA, &goalAngleX, Kp, Ki, Kd, REVERSE );
PID PIDControllerB ( &gyroX, &PIDControllerOutputB, &goalAngleX, Kp, Ki, Kd, DIRECT );


void initializePIDControllers () {
  
  // Set up the PID controllers.
  
  PIDControllerA.SetMode ( AUTOMATIC );
  PIDControllerB.SetMode ( AUTOMATIC );
  
  
  // What how large of a reaction should we allow?
  
  PIDControllerA.SetOutputLimits ( -10, 10 );
  PIDControllerB.SetOutputLimits ( -10, 10 );
  
  
  // How many milliseconds should we wait between samples?
  
  PIDControllerA.SetSampleTime ( 1 );
  PIDControllerB.SetSampleTime ( 1 );
  
}


void initializeESCs () {
  
  s0.attach ( 8 );
  s1.attach ( 9 );
  
  resetESC ( 0 );
  resetESC ( 1 );
  
  useSpeeds ();
  
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
  
  servos [ servo ] = speed;
  
}


void useSpeeds () {
  
  s0.write ( servos [ 0 ] );
  s1.write ( servos [ 1 ] );
  
  delay ( 15 );
  
}


void rampSpeeds () {
  
  speeds [ 0 ] += speedUpOrDown ( speeds [ 0 ], speedBase );
  speeds [ 1 ] += speedUpOrDown ( speeds [ 1 ], speedBase );
  
}


/* 
IMU Calculations Demo, by Starlino
IMU Guide: http://starlino.com/imu_guide.html
Acc_Gyro Board:  http://www.gadgetgangster.com/213

Hardware Setup:

Acc_Gyro <--->  Arduino
5V       <--->  5V (Or 3.3V)
GND      <--->  GND
AX       <--->  AN0
AY       <--->  AN1
AZ       <--->  AN2
GX4      <--->  AN3  
GY4      <--->  AN4    

*/

#define INPUT_COUNT 5     //number of analog inputs
#define VDD 5000.0f       //Analog reference voltage in milivolts
#define PI 3.14159265358979f

int an[INPUT_COUNT];      //analog inputs  
char firstSample;        //marks first sample

struct {					
  char inpInvert[INPUT_COUNT];    // bits 0..5 invert input
  int zeroLevel[INPUT_COUNT];     // 0..2 accelerometer zero level (mV) @ 0 G
                                  // 3..5 gyro zero level (mV) @ 0 deg/s
  int inpSens[INPUT_COUNT];       // 0..2 acceleromter input sensitivity (mv/g)
                                  // 3..5 gyro input sensitivity (mV/deg/ms) 
  float wGyro;		          // gyro weight/smooting factor
} config;	

//Notation "w" stands for one of the axes, so for example RwAcc[0],RwAcc[1],RwAcc[2] means RxAcc,RyAcc,RzAcc
//Variables below must be global (their previous value is used in getEstimatedInclination)
float RwEst[3];     //Rw estimated from combining RwAcc and RwGyro
unsigned long lastMicros;  

//Variables below don't need to be global but we expose them for debug purposes
unsigned long interval; //interval since previous analog samples
float RwAcc[3];         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float RwGyro[3];        //Rw obtained from last estimated value and gyro movement
float Awz[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)


void getEstimatedInclination () {
  
  static int i,w;
  static float tmpf,tmpf2;  
  static unsigned long newMicros; //new timestamp
  static char signRzGyro;  

  //get raw adc readings
  newMicros = micros();       //save the time when sample is taken
  for ( i = 0; i < INPUT_COUNT; i++ ) an[i] = analogRead ( i );
  
  //compute interval since last sampling time 
  interval = newMicros - lastMicros;    //please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2 
  lastMicros = newMicros;               //save for next loop, please note interval will be invalid in first sample but we don't use it
  
  
  //get accelerometer readings in g, gives us RwAcc vector
  for ( w = 0; w <= 2; w++ ) RwAcc[w] = getInput ( w );
  
  //normalize vector (convert to a vector with same direction and with length 1)
  normalize3DVector ( RwAcc );
  
  if ( firstSample ) {
    
    for ( w = 0; w <= 2; w++ ) RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
    
  } else {
    
    //evaluate RwGyro vector
    if ( abs ( RwEst[2] ) < 0.1 ) {
      
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for ( w = 0; w <= 2; w++ ) RwGyro[w] = RwEst[w];
      
    } else {
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
      for ( w = 0; w <= 1; w++ ) {
        
        tmpf = getInput ( 3 + w );                          //get current gyro rate in deg/ms
        tmpf *= interval / 1000.0f;                         //get angle change in deg
        Awz[w] = atan2 ( RwEst[w], RwEst[2] ) * 180 / PI;   //get angle and convert to degrees        
        Awz[w] += tmpf;                                     //get updated angle according to gyro movement
        
      }
      
      //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
      signRzGyro = ( cos ( Awz[0] * PI / 180) >= 0 ) ? 1 : -1;
      
      //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
      for ( w = 0; w <= 1; w++ ) {
        
        RwGyro[0] = sin ( Awz[0] * PI / 180 );
        RwGyro[0] /= sqrt ( 1 + squared ( cos ( Awz[0] * PI / 180 ) ) * squared ( tan ( Awz[1] * PI / 180 ) ) );
        RwGyro[1] = sin ( Awz[1] * PI / 180 );
        RwGyro[1] /= sqrt ( 1 + squared ( cos ( Awz[1] * PI / 180 ) ) * squared ( tan ( Awz[0] * PI / 180 ) ) );
        
      }
      
      RwGyro[2] = signRzGyro * sqrt ( 1 - squared ( RwGyro[0] ) - squared ( RwGyro[1] ) );
      
    }
    
    //combine Accelerometer and gyro readings
    for ( w = 0; w <= 2; w++ ) RwEst[w] = ( RwAcc[w] + config.wGyro * RwGyro[w] ) / ( 1 + config.wGyro );

    normalize3DVector ( RwEst );
   
  }
  
  firstSample = 0;
  
  
  // Store the X axis more accessibly.
  
  gyroX = RwEst[0];
  
}

void normalize3DVector ( float* vector ) {
  
  static float R;
  R = sqrt ( vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2] );
  vector[0] /= R;
  vector[1] /= R;
  vector[2] /= R;
  
}

float squared ( float x ) {
  
  return x * x;
  
}

//Convert ADC value for to physical units see http://starlino.com/imu_guide.html for explanation.
//For accelerometer it will return g (acceleration), applies when xyz = 0..2
//For gyro it will return deg/ms (rate of rotation), applies when xyz = 3..5

float getInput ( char i ) {
  
  static float tmpf;             //temporary variable
  tmpf = an[i] * VDD / 1023.0f;  //voltage (mV)
  tmpf -= config.zeroLevel[i];   //voltage relative to zero level (mV)
  tmpf /= config.inpSens[i];     //input sensitivity in mV/G(acc) or mV/deg/ms(gyro)
  tmpf *= config.inpInvert[i];   //invert axis value according to configuration 
  return tmpf;
  
}

void initializeIMU () {
  
  static int i;
  
  //Setup parameters for Acc_Gyro board, see http://www.gadgetgangster.com/213
  for ( i = 0; i <= 2; i++ ) {        // X,Y,Z axis
    config.zeroLevel[i] = 1650;       // Accelerometer zero level (mV) @ 0 G
    config.inpSens[i] = 478;          // Accelerometer Sensisitivity mV/g
  }        
  
  for ( i = 3; i <= 4; i++ ) {
    config.inpSens[i] = 2000;	    // Gyro Sensitivity mV/deg/ms    
    config.zeroLevel[i] = 1230;     // Gyro Zero Level (mV) @ 0 deg/s  
  }
  
  config.inpInvert[0] = 1;  //Acc X
  config.inpInvert[1] = 1;  //Acc Y
  config.inpInvert[2] = 1;  //Acc Z
  
  //Gyro readings are sometimes inverted according to accelerometer coordonate system
  //see http://starlino.com/imu_guide.html for discussion
  //also see http://www.gadgetgangster.com/213 for graphical diagrams
  config.inpInvert[3] = 1;  //Gyro X  
  config.inpInvert[4] = 1;  //Gyro Y
  
  config.wGyro = 10;
  
  firstSample = 1;
  
}


void orientToAngle () {
  
  // Gradually have all motors follow the base speed.
  
  rampSpeeds ();
  
  
  // Update the PID controllers.
  
  PIDControllerA.Compute();
  PIDControllerB.Compute();
  
  
  // Add the PID controllers' reactions.
  
  servos [ 0 ] = speeds [ 0 ] + PIDControllerOutputA;
  servos [ 1 ] = speeds [ 1 ] + PIDControllerOutputB;
  
  
  // Keep the speeds within the safe ranges ( 0 - 150 ).
  
  servos [ 0 ] = min ( 150, max ( 0, servos [ 0 ] ) );
  servos [ 1 ] = min ( 150, max ( 0, servos [ 1 ] ) );
  
}


void debug () {
  
  Serial.print ( "IMU: " );
  Serial.print ( interval );  //microseconds since last sample, please note that printing more data will increase interval    
  Serial.print ( "," );
  Serial.print ( RwAcc[0] );  //Inclination X axis (as measured by accelerometer)
  Serial.print ( "," );
  Serial.print ( RwEst[0] );  //Inclination X axis (estimated / filtered)
  Serial.print ( ". " );
  
  Serial.print ( "X angle: " );
  Serial.print ( gyroX );
  Serial.println ( "" );
  
  Serial.print ( "Servos: " );
  Serial.print ( servos [ 0 ] );
  Serial.print ( ", " );
  Serial.print ( servos [ 1 ] );
  Serial.print ( ", " );
  Serial.print ( servos [ 2 ] );
  Serial.print ( ", " );
  Serial.print ( servos [ 3 ] );
  Serial.print ( ". " );
  
}


void setup () {
  
  Serial.begin ( 57600 ); 
  
  initializeIMU ();
  
  initializeESCs ();
  
  initializePIDControllers ();
  
}

void loop () {
  
  getEstimatedInclination();
  
  orientToAngle ();
  
  useSpeeds ();
  
  debug ();
  
}