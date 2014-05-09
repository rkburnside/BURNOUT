#include <PulsePosition.h>

// Simple loopback test: create 1 output to transmit
// test pulses, and 1 input to receive the pulses
PulsePositionOutput myOut;
//PulsePositionInput myIn;
int pos = 0;    // variable to store the servo position 

void setup() {
  myOut.begin(21);  // connect pins 5 and 6 together...
  myOut.write(1, 1700);
  delay(1000);
}

int count=0;

void loop() {
  
  for(pos = 1200; pos < 1800; pos += 50)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    //myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    myOut.write(1, pos);
    delay(1000);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 1800; pos>=1200; pos-=50)     // goes from 180 degrees to 0 degrees 
  {                                
    //myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    myOut.write(1, pos);
    delay(1000);                       // waits 15ms for the servo to reach the position 
  } 

  

  
}
