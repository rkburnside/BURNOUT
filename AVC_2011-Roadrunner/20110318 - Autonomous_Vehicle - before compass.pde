/* TO DO LIST
- figure out how to set the initial conditions
- incorporate the throttle on the car!
- make gps location based on an array
- mail in AVC video
- clean up lcd outputs
- set up the ultrasonic sensor on a interrupt
- add a button that will tell the arduino to start going AFTER the switch is flipped
- delete all of the unneeded GPS items that I have listed (alt, speed, course, time, date, etc)
- use an array to determine distances and angles using arrays and get averages of them
- clean up serial outputs of data

x1 - add sd card initialization in setup() - column headings, new line, etc
x2 - add time stamp to sd card data output
x3 - clean up gps code so that i only get the information that i need
x4 - output the gps info i need to serial, lcd, and sd card
x5 - add a 3 second delay into the beginning of the program 'cause that is what sparkfun wants
x6 - have the location for all the waypoints i need
x7 - write routine that calculates the distance and direction to next waypoint
x8 - write routine to determine when i have reached a way point
x9 - write a routine to proceed to the next waypoint
x10 - write to SD card once a second and combine the others into one single string
x11 - change the variable name of SELECT to SELECT
x12 - ensure that strobe is working correctly
x13 - hook up steering servo
x14 - setup servos so the radio is in control unless a switch is flipped giving the arduino control
x15 - hook up gyro and make sure it is working correctly
x16 - verify gryo heading is actually working correctly
x17 - write a routine to turn right or whatever direction i need
x18 - remove LCD and serial writing to determine many clock cycles they each take
*/

/* Sparkfun Autonomous Vehicle Competition Vehicle Code
Author: Richard Burnside
Verion: .04
Date: 03/04/2011

Microcontroller: chinduino (arduino mega)
Sensors to add: steering servo, speed control servo, GPS unit, SD card data logger, 40x4 LCD
Completed Sensors: ultrasonic range finder, IR range sensor, gyroscope

Vehicle: RC car with 7.2V battery and 75mhz receiver
*/

/* How servos work
pulse every 20ms, pulse lenght is between 1.25ms (0-deg) and 1.75ms (180-deg) with 1.50ms (90-deg) being neutral
arduion reads pulses from Hitec radio as follows:

		low		neutral	high
ch1 -	1145	1515	1884
ch2 -	1145	1515	1884
ch3 -	1145	1515	1884 
ch4 -	1145	1515	1884
ch5 -	1130			1884 <-- push switch toward batteries is 1130, pull switch to front is 1884	
ch6 -	1395	1510	1615
*/

/*Autonomous Vehicle Order of Operation - Steering
		
		- what is the current heading compared to the correct heading?
	- heading difference options:
		if the heading difference is... (use the SWITCH CASE function)
			0-5 deg		- don't turn wheels
			5-15 deg	- turn wheels x degrees
			15-30 deg	- turn wheels x degrees
			30-45 deg	- turn wheels x degrees
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
#include <Servo.h> 
#include <math.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h>

//Sensors & Servors
#define TRIGGER 35			//pin # for usonic sensor trigger, P*-9
#define ECHO 34				//pin # for usonic sensor echo, P1
#define IR_PIN 15			//pin # for IR range finder
#define SERVO 12			//pin # for steering servo
#define RECEIVER_IN 13		//pin # for receiver IN
#define STROBE 11			//pin # for Strobe
#define SELECT 10			//pin # for Select
Servo steering;								//steering servo setup and direction
bool autonomous = false;					//true is ON, false is OFF
unsigned long temp_time = 0;				// CHANGE THIS SINCE THE SD CARD IS NOT USED ANYMORE
int i_distance, u_distance;					//distances variables

//LCD Setup
#define RS 22
#define ENABLE_1 24
#define ENABLE_2 23
#define D4 25
#define D5 26
#define D6 27
#define D7 28
LiquidCrystal lcd1(RS, ENABLE_2, D4, D5, D6, D7);
LiquidCrystal lcd2(RS, ENABLE_1, D4, D5, D6, D7);
String dataString = "";

// Gyro Setup
#define GYRO_CAL 8741543
#define GYRO_LIMIT 1000					// defines how many gyro samples are taken between angle calculations
volatile boolean new_cycle = false, cal_flag = false;
boolean aux=1;
volatile long gyro_sum = 0, gyro_count = 0, gyro_null=0, angle=0;
long angle_diff;

//GPS is connected to Serial1 RX pin of arduino mega -- YELLOW/GREEN IS TX FROM THE GPS TO RX ON THE ARDUINO
TinyGPS gps;
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
int gps_speed, course;
double waypoint_distance, waypoint_heading = 0, lat_dist[5] = {0,0,0,0,0}, lon_dist[5] = {0,0,0,0,0};


/* LAT/LON SET - HOUSE TO MAILBOX WAYPOINTS */
// to corner, mailbox, corner, driveway
double lat_array[4] = {39.53844, 39.53818, 39.53844, 39.53871};
double lon_array[4] = {-105.01642, -105.0166, -105.01642, -105.01674};
double waypoint_angle[ ] = {0, 90.0, 
int waypoint_num = 0;
const int total_wps = 4;

/* LAT/LON SET - SPARKFUN PARKING LOT
//Waypoints		0-start		1-corner 1	2-corner 2	3-corner 3	4-corner 4	5-finish
double lat_array[6] = {40.065155, 40.065160, 40.064462, 40.0644720, 40.0651600, 40.0651550};
double lon_array[6] = {-105.21007, -105.209732, -105.209743, -105.210435, -105.210413, -105.210116};
int waypoint_num = 0;
const int total_wps = 5;
*/

/* LAT/LON SET - STAKE CENTER PARKING LOT
0-start		1-corner 1		2-corner 2		3-corner 3		4-corner 4		5-finish
double lat_array[5] = {39.538194, 39.538383, 39.538207, 39.537994, 39.538576};
double lon_array[5] = {-105.011854, -105.011609, -105.011319, -105.011543, -105.016975};
int waypoint_num = 0;
const int total_wps = 4;
*/

void setup() {
	//Set the car to manual mode
	autonomous = false;
	pinMode(RECEIVER_IN, INPUT);
	pinMode(STROBE, OUTPUT);
	pinMode(SELECT, OUTPUT);
	digitalWrite(STROBE, LOW);
	digitalWrite(SELECT, HIGH);

	//Serial Output Initialization
	Serial.begin(115200);
	Serial.println("Time,IR Distance,Ultrasonic Distance, Gyro Heading, Lat, Lon, Date, Time, Course, Speed, Failed");
	Serial1.begin(4800);		//GPS serial port initialization

	steering.attach(SERVO);		//Servo Initialization

	//Ultrasonic Range Finder Initialization
	pinMode(TRIGGER, OUTPUT);
	digitalWrite(TRIGGER, LOW);
	pinMode(ECHO, INPUT);
	digitalWrite(ECHO, LOW);

	//LCD Setup
	lcd1.begin(40, 2);
	lcd2.begin(40, 2);
	lcd1.clear();
	lcd2.clear();
	lcd1.setCursor(0, 0); // top left
	lcd1.print("THE ARDUINO AUTONOMOUS VEHICLE PROJECT");
	lcd2.setCursor(0, 0); // top left
	lcd2.print("DESIGNED BY RICHARD BURNSIDE, 9763 COVE CREEK DR, HIGHLANDS, CO 80129");
	delay(500);

	lcd1.clear();
	lcd2.clear();
	set_gyro_adc();						// sets up free running ADC for gyro
	lcd1.print("calculating gyro null");
	Serial.println("gyro null calculation begin");
	calculate_null();
	Serial.println("gyro null calibration complete");
	lcd2.print("calculation complete");
	delay(500);
	/*	Countdown timer
	lcd1.clear();
	lcd2.clear();
	lcd1.print("The car will start in: ");

	for (int i=9; i >= 0; i--)
	{
		delay(1000);
		lcd1.setCursor(23,0);
		lcd1.print(i);
	}
*/
}

void loop() {
	//GPS data update
	bool newdata = false;
	if(feedgps())
		newdata = true;
	if(newdata)
		gpsdump(gps);

//	IR_sensor();    //<- FIX THIS!!!
//	u_sonic();
	waypoint();
	set_turn();

	if((millis() - temp_time) > 250) {
		auto_control();					// check for autonomous mode 4x / second
		LCD_data_log();
//		Serial_data_log();
		temp_time = millis();
	}

	if((millis() > 10000) && (millis() < 20000))
		initial_conditions();
}

/*----------unfinished code----------*/
void initial_conditions(void) {
	lcd1.clear();
	lcd2.clear();
	lcd1.print("setting initial heading");
	delay(10000);					// give ample time for the GPS to acquire lock and calculate waypoint heading
	while (!new_cycle) ;			// wait for start of new cycle
	angle = (waypoint_heading+90.0)/360.0*GYRO_CAL;	// set's initial angle to match the waypoint heading
	lcd2.print("initial heading complete");
	delay(500);
}

void waypoint(void) {

	double x, y;

	x = 69.1*(lon_array[waypoint_num] - flon) * cos(flat/57.3);
	y = 69.1*(lat_array[waypoint_num] - flat);

	waypoint_distance = sqrt(pow(x,2) + pow(y,2))*5280.0;	// converts distance to feet
//	waypoint_heading = atan2(y,x)*180.0/3.1417;				// 180/pi converts from rads to degrees
	
	waypoint_heading = waypoint_angle[waypoint_num];

	if (waypoint_distance < 6)
		waypoint_num++;
// add code to stop vehicle once the route is complete
	
	return;
}

/*---------completed functions---------*/
void set_turn(void) {
	angle_diff = angle - (waypoint_heading*GYRO_CAL)/360.0;
	if (angle_diff < -GYRO_CAL/2)
		angle_diff += GYRO_CAL;
	if (angle_diff > GYRO_CAL/2)
		angle_diff -= GYRO_CAL;
	
	if ((angle_diff*360.0/GYRO_CAL) >= 130)
		steering.write(130);
	else if ((angle_diff*360.0/GYRO_CAL) <= 50)
		steering.write(50);
	else
		steering.write(angle_diff*360.0/GYRO_CAL);
	return;
}

//Gyro Functions
void set_gyro_adc() {
	ADMUX = B01000001;					//completely reset the MUX. should be sampling only on A1, set vref to internal
	ADCSRA = B11101111;					//set scaler, auto trigger, sampling, etc
	Serial.println("gyro adc set");
	return;
}

ISR(ADC_vect) {        //ADC interrupt
	gyro_sum += ADCL | (ADCH << 8);
	gyro_count++;        //iterate the counter

	if (gyro_count == GYRO_LIMIT) {
		angle += (gyro_sum - gyro_null);
		if ((angle > GYRO_CAL) && (!cal_flag))
			angle -= GYRO_CAL; //if we are calculating null, don't roll-over
		if ((angle < 0) && (!cal_flag))
			angle += GYRO_CAL;
		gyro_sum = 0;
		gyro_count = 0;
		new_cycle = true;
	}
} 

void calculate_null() {				// only used once for cal of null...should be used at each waypoint
	cal_flag = true;
	new_cycle = false;				// this will be set, already, but need to begin on new cycle
	while (!new_cycle) ;			// wait for start of new cycle
	angle = 0;						// reset the angle. angle will act as accumulator for null calculation
	gyro_null = 0;					// make sure to not subract any nulls here
	for (int i=0; i <= 50; i++) {
		while (!new_cycle);
		new_cycle = false;			// start another round
	}
	gyro_null = angle/50;			// calculate the null
	cal_flag = false;
	angle = 0;
	Serial.println("gyro null calculation complete");
	Serial.print("gyro null: ");
	Serial.println(gyro_null);
	return;
}

//GPS functions
bool feedgps() {
	while (Serial1.available()) {
		if(gps.encode(Serial1.read()))
			return true;
	}
	return false;
}

void gpsdump(TinyGPS &gps) {
	gps.f_get_position(&flat, &flon, &age);
	gps.get_datetime(&date, &time, &age);
	gps.stats(&chars, &sentences, &failed);	//should be removed later...just a waste of time
	gps_speed = gps.f_speed_mph();	//not needed also
	course = gps.course();	//could be useful once i figure out what it does

	return;
}

//Data Logging
void Serial_data_log() {
	//String() will not handle float/doubles...this will convert these to arrays
	long latitude, longitude, wp_distance, wp_heading, latitude_array, longitude_array;
	latitude = flat*1000000;
	longitude = flon*1000000;
	latitude_array = lat_array[waypoint_num]*1000000;
	longitude_array = lon_array[waypoint_num]*1000000;
	wp_distance = waypoint_distance;
	wp_heading = waypoint_heading;
	long angle_string;
	angle_string = angle * 360.0 / GYRO_CAL;

	//String creation
	dataString += String(millis());
	dataString += ","; dataString += String(time);
	dataString += ","; dataString += String(gps_speed);
	dataString += ","; dataString += String(i_distance);
	dataString += ","; dataString += String(u_distance);
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

	//Serial printing of string
	Serial.println(dataString);
	dataString = "";				//resets the datastring to 0
	
	return;
}

void LCD_data_log() {
	lcd1.clear();
	lcd1.setCursor(0,0);	lcd1.print("Lt/Ln: "); lcd1.print(flat,6); lcd1.print(" "); lcd1.print(flon,6);
	lcd1.setCursor(29,0);	lcd1.print("Dft: "); lcd1.print(waypoint_distance,1);

	lcd1.setCursor(0,1);	lcd1.print("NWP:   "); lcd1.print(lat_array[waypoint_num],6); lcd1.print(" "); lcd1.print(lon_array[waypoint_num],6);
	lcd1.setCursor(29,1);	lcd1.print("Spd: "); lcd1.print(gps_speed);
	lcd1.setCursor(36,1);	lcd1.print("M: "); lcd1.print(autonomous);

	lcd2.clear();
							lcd2.print("ID_in: "); lcd2.print(i_distance);
	lcd2.setCursor(10,0);	lcd2.print("UD_in: "); lcd2.print(u_distance);
	lcd2.setCursor(20,0);	lcd2.print("Tm: "); lcd2.print(millis()/1000);
	lcd2.setCursor(28,0);	lcd2.print("Nl: "); lcd2.print(gyro_null);

	lcd2.setCursor(0,1);	lcd2.print("GY: "); lcd2.print(angle*360.0/GYRO_CAL,1);
	lcd2.setCursor(10,1);	lcd2.print("Hdng: "); lcd2.print(waypoint_heading,1);
	lcd2.setCursor(22,1);	lcd2.print("Diff: "); lcd2.print(angle_diff*360.0/GYRO_CAL,1);

	lcd2.setCursor(33,1);	lcd2.print("WP: "); lcd2.print(waypoint_num);

return;
}

//Distance Sensors
void auto_control(void) {
	int pulse_length;
	
	pulse_length = pulseIn(RECEIVER_IN, HIGH, 50000);

	if (pulse_length < 1600) {
		autonomous = true;
		digitalWrite(STROBE, LOW);
		digitalWrite(SELECT, HIGH);
	}
	else {
		autonomous = false;
		digitalWrite(STROBE, LOW);
		digitalWrite(SELECT, LOW);
	}

	return;
}

void IR_sensor(void) {							 //function receives nothing and returns distance in inches
	double voltage;
	voltage = 5.0 * analogRead(IR_PIN) / 1023.0;	//reads the IR pin and returns a value between 0 & 1023
	i_distance = 10.347244 * pow(voltage,-1.254);
	if(i_distance > 60)
		i_distance = 60;							//no object detected
	return; 							//distance converted from float to int
}

void u_sonic(void) {

	double duration;
	//HIGH pulse of 10 or more microseconds is neede to TRIGGER sensor.  Give a short LOW pulse beforehand to ensure a clean HIGH pulse.  Pulse whose duration is the time (in microseconds) from the sending of the ping to the reception of its echo off of an object.

	digitalWrite(TRIGGER, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIGGER, LOW);

	//There are 73.746 microseconds per inch (i.e. sound travels at 1130 feet per second).  The distance travelled by the ping, outbound and return, so we divide by 2 to get the distance of the obstacle.  If the pulse is >36000 useconds, no object was detected

	duration = pulseIn(ECHO, HIGH);
	u_distance = duration / 73.746 / 2.0;

	if (u_distance > 60)
		u_distance = 60;
	return;
}