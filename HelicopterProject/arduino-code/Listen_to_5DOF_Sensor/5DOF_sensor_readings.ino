int x, y;  // X and Y in-plane sensing

void setup () {
  
  Serial.begin ( 9600 );      // sets the serial port to 9600
  
}


void loop () {
  
  x = analogRead(0);       // read analog input pin 0
  y = analogRead(1);       // read analog input pin 1
  
  Serial.print(x, DEC);    // print the rotational rate in the X axis
  Serial.print(",");       // prints a space between the numbers
  Serial.println(y, DEC);  // print the rotational rate in the Y axis
  
  delay(100);              // wait 100ms for next reading
  
}
