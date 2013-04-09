/* 2013 Sparkfun Autonomous Vehicle Competition Code
Vehicle: Wile E. Coyote
Author: Richard Burnside
Verion: 1.0
Date: 4/4/2013

Microcontroller: Arduino nano
Sensors to add: steering servo, speed control servo, GPS unit, bluetooth, magnetometer
Vehicle: Turnigy Beetle RC car with 7.2V battery and 2.4-GHz TX/RX
*/

/* TO DO LIST
- compare GPS points at stake center between bing maps and my own gps. drive the car, with the data logger attached, along hard, physical routes to compare its accuracy.
- determine GPS output rate. print milis(), get and print gps reading, print milis(), do the math
- clean up gps code so that i only get the information that i need
- output the gps info i need to serial and then bluetooth
- create serial driven menu system using serial UI
- add manual over ride switch (i.e. competition mode)
- add ~.5 second delay after flipping switch
- have the location for all the waypoints i need
- incorporate cross track error handling
- write routine that calculates the distance and direction to next waypoint
- write routine to determine when i have reached a way point
- write a routine to proceed to the next waypoint
- use latitude/longitude for waypoint acceptance
- verify copmass heading is actually working correctly
- write a routine to turn right or whatever direction i need
- incorporate steering limits so i don't roll the car again
- set gain for steering (i.e. more vs. less agressive steering)
- set gain for throttle (i.e. more vs. less acceleration)
- mail in AVC video
- create shut down function for when the car passes the finish line
- create shut down or reverse function if the car hasn't moved for ~15 seconds from its current spot
- build a roll cage
- hot glue everything
- zipties everything
- delete all of the unneeded GPS items that I have listed (alt, speed, course, time, date, etc)
- use an array to determine GPS points, distances and angles using arrays and get averages of them
- path forward: ignore gps heading correct between corners of building or when it is > 20-ft of waypoint
- develop a correct method/procedure for programming waypoints and driving them consistently
*/

/* Servo Positions
LOW		NEUTRAL		HIGH
1250	1500		1750
0-deg	90-deg		180-deg
*/

/*Autonomous Vehicle Order of Operation - speed
- am i within the range of the next waypoint?
	if the distance difference is... (use the SWITCH CASE function)
		am i starting at waypoint[0] (home)...set speed to 10% to start to ramp up
		0-5-ft			- yes - proceed to next waypoint, add 1 to the waypoint array counter to become next waypoint, and set speed for turning
		5-15-ft			- set speed to 25%
		>15-ft			- set speed to 50%
- is there an object too close?
	if the distance difference is... (use the SWITCH CASE function)
		0-10-in			- use an array to determine the avearage difference, if it stays that way for ~5 seconds, then reverse or change course
		10-30-in		- RAM INTO THE SUCKER!
		>30-in			- proceed on
- have i been stationary (not moved more than 5 feet) for more than 7 seconds (use an array to get the average)?
	use the SWITCH CASE function
		reverse for 3 seconds
		reverse for 3 seconds while turning to the right
		reverse for 3 seconds while turning to the left

*/

//Included Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include <HMC5883L.h>
#include <Servo.h> 
#include <TinyGPS.h>
#include <math.h>

//Function and Object Declarations
SoftwareSerial serial_bluetooth(3,2); // RX, TX
SoftwareSerial serial_gps(4,5); // RX, TX
HMC5883L compass;
TinyGPS gps;

//GPS Variables
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;

int print_time = 0;

//Compass
#define compass_x_cal -20.5
#define compass_y_cal 139.0
#define declinationAngle -0.15126187
int XAxis = 0, YAxis = 0;
double compass_heading = 0;
double angle_diff;				// for the compass


void setup(){
	serial_bluetooth.begin(115200);
	serial_bluetooth.println("bluetooth initialized");
	serial_gps.begin(9600);
	serial_bluetooth.println("gps initialized");

	//compass initialization
	Wire.begin(); // Start the I2C interface.
	compass = HMC5883L(); // Construct a new HMC5883 compass.
	compass.SetScale(0.88); // Set the scale of the compass.
	Serial.println("Setting measurement mode to continous.");
	compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous


	
	delay(500);
}

void loop(){
	compass_measurement();
	gps_data();

	serial_data_log();

	if(print_time > 31){
		serial_bluetooth.println("XAxis\tYAxis\tHeading\t\tLat\t\t\tLon\t\t\t\tFailed");
		serial_bluetooth.println("------------------------------------------------------------------");
		print_time = 0;
	}
	print_time++;
}

//GPS data update

void gps_data(){
	serial_gps.listen();
	bool newdata = false;
	if(feedgps()) newdata = true;
	if(newdata) gpsdump(gps);
	return;
}

bool feedgps(){
	while (serial_gps.available()){
		if (gps.encode(serial_gps.read()))
		return true;
	}
	return false;
}

void gpsdump(TinyGPS &gps){		// GPS Calculation
	gps.f_get_position(&flat, &flon, &age);
	gps.stats(&chars, &sentences, &failed);	//should be removed later...just a waste of time
	return;
}


void compass_measurement(){
	MagnetometerRaw raw = compass.ReadRawAxis();	//get raw 
	YAxis = raw.YAxis + compass_y_cal;	//adjust YAxis with calibration factor
	XAxis = raw.XAxis + compass_x_cal;	//adjust XAxis with calibration factor
	compass_heading = atan2(YAxis, XAxis);	//calculate compass_heading

	compass_heading += declinationAngle;

	if(compass_heading < 0) compass_heading += 2*PI;	// Correct for when signs are reversed.
	if(compass_heading > 2*PI) compass_heading -= 2*PI;	// Check for wrap due to addition of declination.

	compass_heading = compass_heading * 180.0/PI;	// Convert radians to degrees for readability.

//	delay(100); //delay shouldn't be needed because of all the other stuff going on

	return;
}

void serial_data_log(){			// Serial Data Logging
	serial_bluetooth.print(XAxis);	serial_bluetooth.print("\t\t");   
	serial_bluetooth.print(YAxis);	serial_bluetooth.print("\t\t");
	serial_bluetooth.print(compass_heading,2);	serial_bluetooth.print("\t\t");
	serial_bluetooth.print(flat,8);	serial_bluetooth.print("\t");
	serial_bluetooth.print(flon,8); serial_bluetooth.print("\t");
	serial_bluetooth.println(failed);

	return;
}