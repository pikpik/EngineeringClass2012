// This is to control the Thunderbird 9 ESC controller.


// We need to know about servos.
#include <Servo.h>

// Shortened things.
#define m(v)  map ( v, 0, 1023, 0, 179 )

#define ESCBatteryModeAutoLiPo()  \
  Serial.println ( "escAutoLipoMode: begin" ); \
  servo.write ( 0 ); \
  delay ( 2 * 1000 ); \
  servo.write ( 90 ); \
  delay ( 2 * 1000 ); \
  servo.write ( 0 ); \
  delay ( 2 * 1000 ); \
  servo.write ( 90 ); \
  delay ( 2 * 1000 ); \
  Serial.println ( "escAutoLipoMode: done" );

#define ESCBatteryModeAutoLiPo2()  \
  Serial.println ( "escAutoLipoMode: begin" ); \
  servo.write ( 0 ); \
  delay ( 2 * 1000 ); \
  servo.write ( 90 ); \
  delay ( 15 ); \
  servo.write ( 0 ); \
  delay ( 15 ); \
  servo.write ( 90 ); \
  Serial.println ( "escAutoLipoMode: done" );

// Create a servo.
Servo servo;

// Values for the servo's setting.
int value;

void setup () {
  
  // Go slowly for the serial port.
  Serial.begin(9600);
  
  // The servo is on pin 2.
  servo.attach ( 2 );
  
  // It would be best to have a way of turning the ESC's power on and off.
  
  ESCBatteryModeAutoLiPo2();
  
  value = 180;
  
}

void loop () {
  
  // Set battery mode.
  //ESCBatteryModeAutoLiPo();
  
  /*value = 90;
  
  // Send the servo the value.
  servo.write ( value );
  
  // Wait for the servo to get there.
  delay ( 15 );*/
}
