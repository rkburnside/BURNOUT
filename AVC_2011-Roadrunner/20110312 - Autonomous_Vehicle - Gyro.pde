/* TO DO LIST
22 - clean up lcd outputs
23 - set up the ultrasonic sensor on a interrupt
24 - hook up gyro and make sure it is working correctly
18 - mail in AVC video
7 - add a button that will tell the arduino to start going AFTER the switch is flipped
8 - verify gryo heading is actually working correctly
12 - write a routine to turn right or whatever direction i need
14 - delete all of the unneeded GPS items that I have listed (alt, speed, course, time, date, etc)
15 - remove LCD and serial writing to determine many clock cycles they each take
17 - use an array to determine distances and angles using arrays and get averages of them

x1 - add sd card initialization in setup() - column headings, new line, etc
x2 - add time stamp to sd card data output
x3 - clean up gps code so that i only get the information that i need
x4 - output the gps info i need to serial, lcd, and sd card
x6 - add a 3 second delay into the beginning of the program 'cause that is what sparkfun wants
x9 - have the location for all the waypoints i need
x10 - write routine that calculates the distance and direction to next waypoint
x11 - write routine to determine when i have reached a way point
x13 - write a routine to proceed to the next waypoint
x16 - write to SD card once a second and combine the others into one single string
x19 - change the variable name of SELECT to SELECT
x20 - ensure that strobe is working correctly
x21 - hook up steering servo
x5 - setup servos so the radio is in control unless a switch is flipped giving the arduino control
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

//Included Libraries
#include <Servo.h> 
#include <math.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h>

//Sensors & Servors
#define TRIGGER 35			//pin # for usonic sensor trigger, P*-9
#define ECHO 34				//pin # for usonic sensor echo, P1
#define IR_PIN 15			//pin # for IR range finder
#define SERVO 12				//pin # for steering servo
#define RECEIVER_IN 13		//pin # for receiver IN
#define STROBE 11			//pin # for Strobe
#define SELECT 10			//pin # for Select
Servo steering;								//steering servo setup and direction
int direction = 90;
bool autonomous = false;					//true is ON, false is OFF
int up_or_down = 0;							//  ***DELETE TESTING PURPOSES ONLY
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
LiquidCrystal lcd1(RS, ENABLE_1, D4, D5, D6, D7);
LiquidCrystal lcd2(RS, ENABLE_2, D4, D5, D6, D7);
String dataString = "";

// Gyro Setup
#define GYRO_CAL 8741543
#define GYRO_LIMIT 1000					// defines how many gyro samples are taken between angle calculations
volatile boolean new_cycle = false, cal_flag = false;
boolean aux=1;
volatile long gyro_sum = 0, gyro_count = 0, gyro_null=0, angle=0;

//GPS is connected to Serial1 RX pin of arduino mega -- YELLOW/GREEN IS TX FROM THE GPS TO RX ON THE ARDUINO
TinyGPS gps;
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
int gps_speed, course;
double waypoint_distance, waypoint_heading;

/* LAT/LON SET - HOUSE TO MAILBOX WAYPOINTS */
//front, driveway, corner, mailbox, corner, driveway, front
double lat_array[7] = {39.53862, 39.53869, 39.53842, 39.53820, 39.53842, 39.53869, 39.53862};
double lon_array[7] = {-105.01686, -105.01678, -105.01649, -105.01672, -105.01649, -105.01678, -105.01686};
int waypoint_num = 0;
const int total_wps = 6;

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
	set_gyro_adc();						// sets up free running ADC for gyro
	Serial.println("gyro null calculation begin");
	calculate_null();
	Serial.println("gyro null calibration complete");
	Serial.println("gyro calibration start");
	
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
	IR_sensor();
	u_sonic();
	waypoint();

	if((millis() - temp_time) > 250)
		LCD_data_log();
	if((millis() - temp_time) > 250)
		Serial_data_log();
	if((millis() - temp_time) > 250) {		//check for autonomous mode 1 time per second
		auto_control();
		temp_time = millis();
		set_turn();
	}

		
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
}

/*----------unfinished code----------*/
void set_turn(void) {

	steering.write(direction);
	if(up_or_down == 0)
	{
		if(direction < 120)
		direction = direction + 5;
		else
		up_or_down = 1;
	}
	else if(direction > 60)
	direction = direction - 5;
	else
	up_or_down = 0;
	
	return;
}



/*---------completed functions---------*/
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
	while (Serial1.available())
	{
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

void waypoint(void) {

	double x, y;

	x = 69.1*(lon_array[waypoint_num] - flon) * cos(flat/57.3);
	y = 69.1*(lat_array[waypoint_num] - flat);

	waypoint_distance = sqrt(pow(x,2) + pow(y,2))*5280.0;
	waypoint_heading = atan2(y,x)*180.0/3.1417;

	if (waypoint_num < total_wps)
		waypoint_num++;
	else
		waypoint_num = total_wps;

	
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

	//String creation
	dataString += String(millis());
	dataString += ","; dataString += String(time);
	dataString += ","; dataString += String(gps_speed);
	dataString += ","; dataString += String(i_distance);
	dataString += ","; dataString += String(u_distance);
	dataString += ","; dataString += String(course);
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
	lcd1.print("ID_in: "); lcd1.print(i_distance);

	lcd1.setCursor(10,0); lcd1.print("UD_in: "); lcd1.print(u_distance);
	lcd1.setCursor(29,0); lcd1.print("Tm: "); lcd1.print(millis());

	lcd1.setCursor(0,1); lcd1.print("Lt/Ln: "); lcd1.print(flat,6); lcd1.print(" "); lcd1.print(flon,6);
	lcd1.setCursor(29,1); lcd1.print("Spd: "); lcd1.print(gps_speed);

	lcd2.clear();
	lcd2.print("NWP:   "); lcd2.print(lat_array[waypoint_num],6); lcd2.print(" "); lcd2.print(lon_array[waypoint_num],6);
	lcd2.setCursor(29,0); lcd2.print("Dft: "); lcd2.print(waypoint_distance,1);

	lcd2.setCursor(0,1); lcd2.print("Crs: "); lcd2.print(course);
	lcd2.setCursor(10,1); lcd2.print("Hdng: "); lcd2.print(waypoint_heading);
	lcd2.setCursor(22,1); lcd2.print("wp: "); lcd2.print(waypoint_num);
	lcd2.setCursor(28,1); lcd2.print("Mode: ");
	if(autonomous)
		lcd2.print("Auto");
	else
		lcd2.print("Manual");

return;
}

//Distance Sensors
void auto_control(void) {
	int pulse_length;
	
	pulse_length = pulseIn(RECEIVER_IN, HIGH, 50000);
	
	if(pulse_length > 1000)
	{
		autonomous = true;
		digitalWrite(STROBE, LOW);
		digitalWrite(SELECT, HIGH);
	}
	else
	{
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
