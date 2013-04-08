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
SoftwareSerial serial_gps(4,5); // RX, TX
SoftwareSerial serial_bluetooth(3,2); // RX, TX
HMC5883L compass;
Servo steering;					// steering servo setup and direction
Servo speed;
TinyGPS gps;

//Compass
#DEFINE compass_x_cal -20.5
#DEFINE compass_y_cal 139.0
double compass_heading = 0;
double angle_diff;				// for the compass

//Steering and Throttle
#define SERVO 12				//pin # for steering servo
#define THROTTLE 7				// pin # for throttle

//GPS Variables
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
int gps_speed, max_speed = 0, course;
double waypoint_distance, waypoint_heading = 0, lat_dist[5] ={0,0,0,0,0}, lon_dist[5] ={0,0,0,0,0};

//GPS Waypoints
int waypoint_num = 0;
int special_waypoint_num = 4; // ALWAYS 1 LESS 'cause arrays start at 0, not 1!
const int waypoint_total = 15;	// <- should always be the same number of GPS waypoints
// 0 = lat use, 1 = south lat use, 2 = north lat use
double gps_array[15][3] ={{40.0651517950864528, -105.2097273131420, 0}, // 1st corner
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


void setup(){
	steering.attach(SERVO);		//Servo Initialization
	speed.attach(THROTTLE);		//Speed control initialization

	serial_gps.begin(9600);
	serial_bluetooth.begin(115200);
	serial_bluetooth.println("bluetooth initialized");

	//compass initialization
	Wire.begin(); // Start the I2C interface.
	serial_bluetooth.println("Constructing new HMC5883L");
	compass = HMC5883L(); // Construct a new HMC5883 compass.
	Serial.println("Setting scale to +/- 0.88 Ga");
	compass.SetScale(0.88); // Set the scale of the compass.
	Serial.println("Setting measurement mode to continous.");
	compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	
	delay(500);
}

void loop(){
	compass();
	waypoint();
	set_turn();
	set_speed();
	gps_data();
}

void compass(){
	MagnetometerRaw raw = compass.ReadRawAxis();	//get raw 
	raw.YAxis = raw.YAxis + compass_y_cal;	//adjust YAxis with calibration factor
	raw.XAxis = raw.XAxis + compass_x_cal;	//adjust XAxis with calibration factor
	float heading = atan2(raw.YAxis, raw.XAxis);	//calculate heading

	float declinationAngle = -0.15126187;	//adjust for declination
	heading += declinationAngle;

	// Correct for when signs are reversed.
	if(heading < 0)
	heading += 2*PI;
	
	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
	heading -= 2*PI;

	float headingDegrees = heading * 180.0/M_PI;	// Convert radians to degrees for readability.


	Output(raw, scaled, heading, headingDegrees);	// Output the data via the serial port.

//	delay(100); //delay shouldn't be needed because of all the other stuff going on
}

// Output the data down the serial port.
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
	serial_bluetooth.print("Raw:\t");
	serial_bluetooth.print(raw.XAxis);
	serial_bluetooth.print("   ");   
	serial_bluetooth.print(raw.YAxis);
	serial_bluetooth.print("   ");   
	serial_bluetooth.print(raw.ZAxis);

	serial_bluetooth.print("   \tHeading:\t");
	serial_bluetooth.print(heading);
	serial_bluetooth.print(" Radians   \t");
	serial_bluetooth.print(headingDegrees);
	serial_bluetooth.println(" Degrees   \t");
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
	gps_speed = gps.f_speed_mph();	//not needed also
	if(gps_speed > max_speed) max_speed = gps_speed;
	return;
}

/* FROM GPS.PDE. DETERMINE IF THE OTHER GPS FUNCTION WORKS OR NOT
void gpsdump(TinyGPS &gps){
	long lat, lon;
	unsigned long age, date, time, chars;
	unsigned short sentences, failed;

	gps.get_position(&lat, &lon, &age);
	// gps.get_datetime(&date, &time, &age);
	gps.stats(&chars, &sentences, &failed);

	serial_bluetooth.print("Lat/Long(10^-5 deg): "); serial_bluetooth.print(lat); serial_bluetooth.print(", "); serial_bluetooth.println(lon); 
	// serial_bluetooth.print("Date(ddmmyy): "); serial_bluetooth.print(date); serial_bluetooth.print(" Time(hhmmsscc): "); serial_bluetooth.println(time);
	// serial_bluetooth.print("Alt(cm): "); serial_bluetooth.println(gps.altitude());
	// serial_bluetooth.print("Course(10^-2 deg): "); serial_bluetooth.println(gps.course());
	// serial_bluetooth.print("Course(float): "); serial_bluetooth.println(gps.f_course());
	// serial_bluetooth.print("Speed (mph): ");  serial_bluetooth.println(gps.f_speed_mph());
	//	serial_bluetooth.print("Stats: characters: "); serial_bluetooth.print(chars);
	//	serial_bluetooth.print("Sentences: "); serial_bluetooth.println(sentences);
	serial_bluetooth.print("Failed checksum: "); serial_bluetooth.println(failed);

	serial_bluetooth.print("Lat/Long(10^-5 deg): "); serial_bluetooth.print(lat); serial_bluetooth.print(", "); serial_bluetooth.println(lon); 
	
}
*/




/*----------unfinished code----------*/
void set_speed(void){	// test this and delete the delays after it works
	if(waypoint_distance <= 20)		speed.write(112);
	if(waypoint_distance > 20)		speed.write(150);
	return;
}

void waypoint(void){				// Distance and angle to next waypoint
	double x, y;

	x = 69.1*(gps_array[waypoint_num][1] - flon) * cos(flat/57.3);
	y = 69.1*(gps_array[waypoint_num][0] - flat);
	
	waypoint_distance = sqrt(pow(x,2) + pow(y,2))*5280.0;	// converts distance to feet
	waypoint_heading = atan2(y,x)*180.0/M_PI;				// 180/pi converts from rads to degrees
	if(waypoint_heading < 0)	waypoint_heading += 360.0;

	if(waypoint_num == special_waypoint_num) waypoint_heading = 289.0;

	if((gps_array[waypoint_num][2] == 1) && (gps_array[waypoint_num][0] > flat)) waypoint_num++; // going south
	else if((gps_array[waypoint_num][2] == 2) && (gps_array[waypoint_num][0] < flat)) waypoint_num++; // going north
	else if(waypoint_distance < 10) waypoint_num++;

	while(waypoint_num >= waypoint_total){		// shuts off the vehicle by disabling autonomous mode & MUXER & sets speed to 0
		autonomous = false;
		digitalWrite(STROBE, LOW);
		digitalWrite(SELECT, LOW);
		speed.write(90);
	}

	return;
}

/*---------completed functions---------*/
void set_turn(void){				// Set servo to steer in the direction of the next waypoint
	double servo_angle;

	angle_diff = compass_heading - waypoint_heading;

	if(angle_diff < -180)	angle_diff += 360.0;
	if(angle_diff >= 180)	angle_diff -= 360.0;

	servo_angle = angle_diff/2.0 + 90.0;		//changes domain from -180...180 to 0...180
	servo_angle = 180.0 - servo_angle;
	
	if(servo_angle > 130.0)		steering.write(130);
	else if(servo_angle < 50.0)	steering.write(50);
	else						steering.write(servo_angle);
	return;
}

void compass(void){				// Determine compass heading with 0-deg matching standard graph
	int xParam, yParam;
	double x, y, northFi;

	northFi = atan2(y,x);
	compass_heading = (northFi * 180.0) / M_PI + 90.0 - 10.0; // 9 is to correct for magnetic north
	if(compass_heading < 0) compass_heading += 360.0;

	return;
}


void Serial_data_log(){			// Serial Data Logging
	//String() will not handle float/doubles...this will convert these to arrays
	long latitude, longitude, wp_distance, wp_heading, latitude_array, longitude_array;
	latitude = flat*1000000;
	longitude = flon*1000000;
	latitude_array = gps_array[waypoint_num][0]*1000000;
	longitude_array = gps_array[waypoint_num][1]*1000000;
	wp_distance = waypoint_distance;
	wp_heading = waypoint_heading;
	long angle_string;
	angle_string = angle * 360.0 / GYRO_CAL;

	//String creation
	dataString += String(millis());
	dataString += ","; dataString += String(time);
	dataString += ","; dataString += String(gps_speed);
	dataString += ","; dataString += String(gyro_null);
	dataString += ","; dataString += String(angle);
	dataString += ","; dataString += String(angle_string);
	dataString += ","; dataString += String(latitude);
	dataString += ","; dataString += String(longitude);
	dataString += ","; dataString += String(latitude_array);
	dataString += ","; dataString += String(longitude_array);	
	dataString += ","; dataString += String(wp_distance);
	dataString += ","; dataString += String(wp_heading);
	dataString += ","; dataString += String(failed);

	if(autonomous)
	dataString += ",Auto";
	else
	dataString += ",Manual";

	Serial.println(compass_heading,1);
	Serial.println(waypoint_heading,1);
	//	servo_angle is not a global variable Serial.println(servo_angle,1);
	Serial.println(angle_diff,1);

	//Serial printing of string
	Serial.println(dataString);
	dataString = "";				//resets the datastring to 0
	
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