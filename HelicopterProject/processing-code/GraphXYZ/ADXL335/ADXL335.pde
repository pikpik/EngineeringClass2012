 // This program takes ASCII-encoded strings
 // from the serial port at 9600 baud and graphs them. It expects values in the
 // range 0 to 1023, followed by a newline, or newline and carriage return
 
 import processing.serial.*;
 
 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
 
 int w = 400;
 int h = 300;
 
 void setup () {
   // set the window size:
   size(w, h);        
   
   // List all the available serial ports
   println(Serial.list());
   // I know that the first port in the serial list on my mac
   // is always my  Arduino, so I open Serial.list()[0].
   // Open whatever port is the one you're using.
   myPort = new Serial(this, Serial.list()[1], 9600);
   // don't generate a serialEvent() unless you get a newline character:
   myPort.bufferUntil('\n');
   // set inital background:
   background(0);
 }
 
 void draw () {
   // everything happens in the serialEvent()
 }
 
 void serialEvent (Serial myPort) {
   // get the ASCII string:
   String inString = myPort.readStringUntil('\n');
   
   if (inString != null) {
     // trim off any whitespace:
     inString = trim(inString);
     
     // Separate the three points.
     String[] xyz = split(inString, ',');
     
     // convert to an int and map to the screen height:
     float x = float(xyz[0]); // Use x.
     x = map(x, 0, 1023, 0, 100);
     
     float y = float(xyz[1]); // Use y.
     y = map(y, 0, 1023, 0, 100);
     
     float z = float(xyz[2]); // Use z.
     z = map(z, 0, 1023, 0, 100);
     
     // draw the line:
     stroke(255,0,0);
     line(xPos, 100, xPos, 100 - x);
     
     stroke(0,255,0);
     line(xPos, 200, xPos, 200 - y);
     
     stroke(0,0,255);
     line(xPos, 300, xPos, 300 - z);
     
     // at the edge of the screen, go back to the beginning:
     if (xPos >= width) {
       xPos = 0;
       background(0); 
     } 
     else {
       // increment the horizontal position:
       xPos++;
     }
   }
 }
