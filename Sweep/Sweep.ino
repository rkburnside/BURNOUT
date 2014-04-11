// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 

Servo throttle, steering;  // create servo object to control a servo 
// a maximum of eight servo objects can be created 

int pos = 0;    // variable to store the servo position 

void setup() 
{ 
	delay(3000);
	steering.attach(21);  // attaches the servo on pin 9 to the servo object 
	throttle.attach(20);  // attaches the servo on pin 9 to the servo object 
	throttle.write(70);  
} 


void loop() 
{ 
	for(pos = 30; pos < 150; pos += 1)  // goes from 0 degrees to 180 degrees 
	{                                  // in steps of 1 degree 
		steering.write(pos);              // tell servo to go to position in variable 'pos' 
		delay(15);                       // waits 15ms for the servo to reach the position 
	} 
	for(pos = 150; pos>=30; pos-=1)     // goes from 180 degrees to 0 degrees 
	{                                
		steering.write(pos);              // tell servo to go to position in variable 'pos' 
		delay(15);                       // waits 15ms for the servo to reach the position 
	} 
} 
