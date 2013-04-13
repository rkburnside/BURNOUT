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
- determine GPS output rate. print millis(), get and print gps reading, print millis(), do the math
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

/*
http://letsmakerobots.com/node/28278
http://arduino.cc/forum/index.php?topic=54036.0;wap2
PWM and timer
There is fixed relation between the timers and the PWM capable outputs. When you look in the data sheet or the pinout of the processor these PWM capable pins have names like OCRxA, OCRxB or OCRxC (where x means the timer number 0..5). The PWM functionality is often shared with other pin functionality. 
The Arduino has 3Timers and 6 PWM output pins. The relation between timers and PWM outputs is:
Pins 5 and 6: controlled by timer0
Pins 9 and 10: controlled by timer1
Pins 11 and 3: controlled by timer2
*/

//Included Libraries
#include <Wire.h>
#include <HMC5883L.h>
#include <Servo.h> 
#include <TinyGPS.h>
#include <math.h>

//Function and Object Declarations
HMC5883L compass;
TinyGPS gps;
Servo steering;
Servo speed;

//Steering and Throttle
#define SERVO 2				// pin # for steering servo - green	
#define THROTTLE 3			// pin # for throttle - yellow wire

//GPS Variables
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
double waypoint_distance, waypoint_heading = 0;

//Compass
#define compass_x_cal -92
#define compass_y_cal 157
#define declinationAngle 0.15126187
int XAxis = 0, YAxis = 0;
double compass_heading = 0;
double angle_diff;				// for the compass


//GPS Waypoints
int waypoint_num = 0;
const int waypoint_total = 5;	// <- should always be the same number of GPS waypoints
// 0 = lat use, 1 = south lat use, 2 = north lat use
double gps_array[5][3] = {{39.538696506815334, -105.01680727005721, 0},
{39.53873270565525, -105.01672948599578, 0},
{39.53847827912335, -105.01641834975005, 0},
{39.53861066377666, -105.01659805775405, 0},
{39.53863031460211, -105.01675362587692, 0}};

double print_delay = 0;

void setup(){
	Serial1.begin(115200);		// bluetooth serial
	Serial2.begin(9600);		// gps serial
	steering.attach(SERVO);		//Servo Initialization
	speed.attach(THROTTLE);		//Speed control initialization

	//compass initialization
	Wire.begin(); // Start the I2C interface.
	compass = HMC5883L(); // Construct a new HMC5883 compass.
	compass.SetScale(0.88); // Set the scale of the compass.
	compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

	delay(1000);
}

void loop(){
	compass_measurement();
	gps_data();
	waypoint();
	set_turn();
	set_speed();

	if((millis() - print_delay) > 250){
		serial_data_log();
		print_delay = millis();
	}	
}

/* these functions need work. consider incorporating XXX code ideas.*/


void set_speed(void){		// test this and delete the delays after it works
	// if(waypoint_distance <= 20)		speed.write(13);
	// if(waypoint_distance > 20)		speed.write(13);

	// while(waypoint_num >= waypoint_total) speed.write(0);		// shuts off the vehicle by setting speed to 0
	speed.write(105);
	return;
}



void set_turn(void){				// Set servo to steer in the direction of the next waypoint
	double servo_angle;
//	right is 0, left is 180.
	compass_heading = -(compass_heading - 90.0);	// switches heading to normal x,y coordinates

	if(compass_heading < 0.0)	compass_heading += 360;
	if(compass_heading > 360.0)	compass_heading -= 360;

	angle_diff = compass_heading - waypoint_heading;

	if(angle_diff < -180)	angle_diff += 360.0;
	if(angle_diff >= 180)	angle_diff -= 360.0;

	servo_angle = angle_diff/2.0 + 90.0;		//changes domain from -180...180 to 0...180
	servo_angle = 180.0 - servo_angle;
	
//	this section sets the servo/turning limits
	if(servo_angle > 130.0)		steering.write(130);
	else if(servo_angle < 50.0)	steering.write(50);
	else						steering.write(servo_angle);


	return;
}

void waypoint(void){				// Distance and angle to next waypoint
	double x, y;

	x = 69.1*(gps_array[waypoint_num][1] - flon) * cos(flat/57.3);
	y = 69.1*(gps_array[waypoint_num][0] - flat);
	
	waypoint_distance = sqrt(pow(x,2) + pow(y,2))*5280.0;	// converts distance to feet
	waypoint_heading = atan2(y,x)*180.0/M_PI;				// 180/pi converts from rads to degrees

	if(waypoint_heading < 0)	waypoint_heading += 360.0;	// ensures heading is ALWAYS positive

// this is the waypoint acceptance section. consider breaking it out into another function
	if((gps_array[waypoint_num][2] == 1) && (gps_array[waypoint_num][0] > flat)) waypoint_num++; // going south
	else if((gps_array[waypoint_num][2] == 2) && (gps_array[waypoint_num][0] < flat)) waypoint_num++; // going north
	else if(waypoint_distance < 10) waypoint_num++;

	return;
}



/*Working Functions That Are Working Well*/


//GPS data update
void gps_data(){
	bool newdata = false;
	if(feedgps()) newdata = true;
	if(newdata) gpsdump(gps);
	return;
}

bool feedgps(){
	while(Serial2.available()){
		if(gps.encode(Serial2.read()))
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
	XAxis = raw.XAxis + compass_x_cal;	//adjust XAxis with calibration factor
	YAxis = raw.YAxis + compass_y_cal;	//adjust YAxis with calibration factor

	compass_heading = atan2(YAxis, XAxis);	//calculate compass_heading

	compass_heading = PI - compass_heading;
	compass_heading = compass_heading + declinationAngle;
	if(compass_heading < 0.0) compass_heading += 2.0*PI;
	if(compass_heading > (2.0*PI)) compass_heading -= 2.0*PI;

	compass_heading = compass_heading * 180.0/PI;

	return;
}



void serial_data_log(){			// Serial Data Logging
	Serial1.print(waypoint_heading);	Serial1.print("\t\t");   
	Serial1.print(compass_heading);		Serial1.print("\t\t");
	Serial1.print(flat,8);				Serial1.print("\t\t");
	Serial1.print(flon,8); 				Serial1.print("\t\t");
	Serial1.println(failed);
	
	return;
}




/* GPS Routes

SC - Parking Lot - old
{{39.538298770540344,-105.01172583175334},
{39.53849527942753,-105.01150857282313},
{39.538635938078826,-105.01140396667155},
{39.538669034190654,-105.01133154702815},
{39.53855319773023,-105.01111160588893},
{39.53844356661642,-105.01092921567592},
{39.53837323713148,-105.0108541138235},
{39.538269811288885,-105.01089702916774},
{39.53810019257341,-105.0110874660078},
{39.537924367857656,-105.01124303413066},
{39.53787058609094,-105.01135568690928},
{39.5379740125284,-105.01154344154033},
{39.53809398700279,-105.0117365605894},
{39.53815914891089,-105.01183747870347},
{39.53820179479863,-105.01186061275624},
{39.53826074760557,-105.01180830968045}};
*/