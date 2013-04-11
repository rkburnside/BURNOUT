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
SoftwareSerial serial_bluetooth(3,2);	// RX, TX
SoftwareSerial serial_gps(4,5);			// RX, TX
HMC5883L compass;
TinyGPS gps;
Servo steering;
Servo speed;

//Steering and Throttle
#define SERVO 6					// pin # for steering servo
#define THROTTLE 7				// pin # for throttle

//GPS Variables
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
double waypoint_distance, waypoint_heading = 0;

//Compass
#define compass_x_cal -20.5
#define compass_y_cal 139.0
#define declinationAngle -0.15126187
int XAxis = 0, YAxis = 0;
double compass_heading = 0;
double angle_diff;				// for the compass


//GPS Waypoints
int waypoint_num = 0;
const int waypoint_total = 15;	// <- should always be the same number of GPS waypoints
// 0 = lat use, 1 = south lat use, 2 = north lat use
double gps_array[15][3] = {{40.0651517950864528, -105.2097273131420, 0}, // 1st corner
	{40.0650036900410714, -105.2097272812302, 1}, // 1st line up point
	{40.0649066081190785, -105.2097109401257, 1}, // 2nd line up point or start of straight line
	{40.06485, -105.20975, 1}, // just added today
	{40.064528, -105.2097763511701, 1}, // lat point for start of turn
	{40.06448686920148, -105.20985212357533, 0}, // 2nd corner, just inside of south parking lot
	{40.06445080617122, -105.21016393037269, 0}, // bottom middle of south parking lot
	{40.06449104008855, -105.21041203470733, 0}, // inbetween sidewalks for 3rd corner
	{40.06450670940305, -105.2104463610966, 0}, // 3rd corner
	{40.06465758628262, -105.21046433778312, 2}, // line up for inbetween next bottleneck
	{40.06475817068338, -105.21046970220115, 2}, // past the bottleneck
	{40.06493060073908, -105.21048043103721, 2}, // middle of the 3rd side
	{40.06514305860027, -105.21048311324623, 2}, // end of 3rd side
	{40.06519745591719, -105.21041069360282, 0}, // 4th corner
	{40.06518985782108, -105.21001238556421, 0}}; // way past the finish line

int header_line = 0;				// counter that reprints the serial header line

void setup(){
	serial_bluetooth.begin(115200);
	serial_bluetooth.println("bluetooth initialized");
	serial_gps.begin(9600);
	serial_bluetooth.println("gps initialized");

	steering.attach(SERVO);		//Servo Initialization
	speed.attach(THROTTLE);		//Speed control initialization

	//compass initialization
	Wire.begin(); // Start the I2C interface.
	compass = HMC5883L(); // Construct a new HMC5883 compass.
	compass.SetScale(0.88); // Set the scale of the compass.
	Serial.println("Setting measurement mode to continous.");
	compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

	delay(1000);
}

void loop(){
	compass_measurement();
	gps_data();

	waypoint();
	set_turn();
	set_speed();

	serial_data_log();

	if(header_line > 31){
		serial_bluetooth.println("");
		serial_bluetooth.println("XAxis\tYAxis\tHeading\t\tLat\t\t\tLon\t\t\t\tFailed");
		serial_bluetooth.println("------------------------------------------------------------------");
		header_line = 0;
	}
	header_line++;
}

/* these functions need work. consider incorporating XXX code ideas.*/

void set_speed(void){		// test this and delete the delays after it works
	if(waypoint_distance <= 20)		speed.write(112);
	if(waypoint_distance > 20)		speed.write(150);

	while(waypoint_num >= waypoint_total) speed.write(90);		// shuts off the vehicle by setting speed to 0

	return;
}

void set_turn(void){				// Set servo to steer in the direction of the next waypoint
	double servo_angle;

	angle_diff = compass_heading - waypoint_heading;

	if(angle_diff < -180)	angle_diff += 360.0;
	if(angle_diff >= 180)	angle_diff -= 360.0;

	servo_angle = angle_diff/2.0 + 90.0;		//changes domain from -180...180 to 0...180
	servo_angle = 180.0 - servo_angle;
	
	// this section sets the servo/turning limits
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

	return;
}

void serial_data_log(){			// Serial Data Logging
	serial_bluetooth.print(XAxis);	serial_bluetooth.print("\t\t");   
	serial_bluetooth.print(YAxis);	serial_bluetooth.print("\t\t");
	serial_bluetooth.print(compass_heading,2);	serial_bluetooth.print("\t\t");
	serial_bluetooth.print(flat,8);	serial_bluetooth.print("\t");
	serial_bluetooth.print(flon,8); serial_bluetooth.print("\t");
	serial_bluetooth.println(failed);

	// i should include waypoint heading, distance, angle diff, etc
	// latitude_array = gps_array[waypoint_num][0]*1000000;
	// longitude_array = gps_array[waypoint_num][1]*1000000;
	// wp_distance = waypoint_distance;
	// wp_heading = waypoint_heading;
	// long angle_string;
	// angle_string = angle * 360.0 / GYRO_CAL;

	
	
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