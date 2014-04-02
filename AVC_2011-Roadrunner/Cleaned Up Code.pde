/* 2011 Sparkfun Autonomous Vehicle Competition Vehicle Code
Author: Richard Burnside
Verion: 1.0
Last Update, 2:00PM: 4/23/2011
*/

//Included Libraries
#include <Wire.h>
#include <Servo.h> 
#include <math.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h>

//Sensors & Servors
#define RECEIVER_IN 13			//pin # for receiver IN
#define STROBE 11				//pin # for Strobe
#define SELECT 10				//pin # for Select
#define SERVO 12				//pin # for steering servo
#define THROTTLE 7				// pin # for throttle
Servo steering;					// steering servo setup and direction
Servo speed;
bool autonomous = false;		// true is ON, false is OFF
bool race_start = true;		// creates a 3 second delay to start the vehicle
unsigned long temp_time = 0;	// CHANGE THIS SINCE THE SD CARD IS NOT USED ANYMORE

//Compass
#define XCLR 8					// pin # for compass XCLR
#define MCLCK 9					// pin # for compass master clock
#define C_ADDRESS B0110000		// compass address, [0110xx0] [xx] is determined by factory programming, total 4 different addresses are available
#define COMP_X 2041				// compass center X
#define COMP_Y 2040				// compass center Y
double compass_heading = 0;
double angle_diff;				// for the compass

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

//GPS Initialization
TinyGPS gps;
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
int gps_speed, max_speed = 0, course;
double waypoint_distance, waypoint_heading = 0, lat_dist[5] = {0,0,0,0,0}, lon_dist[5] = {0,0,0,0,0};

/* GPS Waypoints */
int waypoint_num = 0;
int special_waypoint_num = 4; // ALWAYS 1 LESS 'cause arrays start at 0, not 1!
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

void setup() {
	//Set the car to manual mode
	autonomous = false;
	pinMode(RECEIVER_IN, INPUT);
	pinMode(STROBE, OUTPUT);
	pinMode(SELECT, OUTPUT);
	digitalWrite(STROBE, LOW);
	digitalWrite(SELECT, HIGH);

	steering.attach(SERVO);		//Servo Initialization
	speed.attach(THROTTLE);		//Speed control initialization
	
	//Serial Output Initialization
	Serial.begin(115200);
	Serial.println("Time,IR Distance,Ultrasonic Distance, Gyro Heading, Lat, Lon, Date, Time, Course, Speed, Failed");
	Serial1.begin(4800);		//GPS serial port initialization

	//Compass Module Initialization
	pinMode(XCLR, OUTPUT);
	digitalWrite(XCLR, LOW);
	pinMode(MCLCK, OUTPUT);
	digitalWrite(MCLCK, LOW);

	Wire.begin();
	delay(20);								//give some time (who's in a hurry?)
	Wire.beginTransmission(C_ADDRESS);		//Send target (master)address
	Wire.send(0x00);						//Wake up call, send SET signal to set/reset coil
	Wire.send(0x02);
	Wire.endTransmission();					//wait for SET action to settle
	delay(10);
	
	//LCD Setup
	lcd1.begin(40, 2);
	lcd2.begin(40, 2);
	lcd1.clear();
	lcd2.clear();
	lcd1.setCursor(0, 0); // top left
	lcd1.print("THE ARDUINO AUTONOMOUS VEHICLE PROJECT");
	lcd2.setCursor(0, 0); // top left
	lcd2.print("DESIGNED BY RICHARD BURNSIDE, HIGHLANDS, CO");
	delay(500);

	lcd1.clear();
	lcd2.clear();
	delay(500);
}

void loop() {
	//GPS data update
	bool newdata = false;
	if(feedgps())
		newdata = true;
	if(newdata)
		gpsdump(gps);

	waypoint();
	compass();
	set_turn();
	set_speed();

		if((millis() - temp_time) > 250) {
		auto_control();					// check for autonomous mode 4x / second
		LCD_data_log();
//		Serial_data_log();
		temp_time = millis();
	}
}

/*----------Functions----------*/
void set_speed(void) {	// test this and delete the delays after it works
		if(waypoint_distance <= 20)		speed.write(112);
		if(waypoint_distance > 20)		speed.write(150);
	return;
}

void waypoint(void) {				// Distance and angle to next waypoint
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

	while(waypoint_num >= waypoint_total) {		// shuts off the vehicle by disabling autonomous mode & MUXER & sets speed to 0
		autonomous = false;
		digitalWrite(STROBE, LOW);
		digitalWrite(SELECT, LOW);
		speed.write(90);
	}

	return;
}

void set_turn(void) {				// Set servo to steer in the direction of the next waypoint
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

void compass(void) {				// Determine compass heading with 0-deg matching standard graph
	int xParam, yParam;
	double x, y, northFi;

	byte rcvByte[4];
	Wire.beginTransmission(C_ADDRESS);		//Send target (master)address
	Wire.send(0x00);						//Wake up call, request for data
	Wire.send(0x01);
	Wire.endTransmission();
	delay(7);								//wait 5ms min for compass to acquire data
	Wire.requestFrom(C_ADDRESS, 4);

	for(int i=0; i < 4; i++) {
		rcvByte[i] = 0;
		rcvByte[i] = Wire.receive();
	}

	xParam = rcvByte[0] << 8;
	xParam = xParam | rcvByte[1];
	yParam = rcvByte[2] << 8;
	yParam = yParam | rcvByte[3];

	x = -(xParam - COMP_X);
	y = (yParam - COMP_Y);
	northFi = atan2(y,x);
	compass_heading = (northFi * 180.0) / M_PI + 90.0 - 10.0; // 9 is to correct for magnetic north
	if(compass_heading < 0) compass_heading += 360.0;

	return;
}

void auto_control(void) {			// Autonomous Mode
	int pulse_length;

	pulse_length = pulseIn(RECEIVER_IN, HIGH, 50000);

	Serial.println(pulse_length);
	if(pulse_length < 500) {
		if(race_start) {
			delay(250);
			race_start = false;
		}
		
		autonomous = true;
		digitalWrite(STROBE, LOW);
		digitalWrite(SELECT, LOW);
	}
	else {
		autonomous = false;
		digitalWrite(STROBE, LOW);
		digitalWrite(SELECT, HIGH);
	}

	return;
}

bool feedgps() {					// GPS Data Input
	while(Serial1.available()) {
		if(gps.encode(Serial1.read()))
			return true;
	}
	return false;
}

void gpsdump(TinyGPS &gps) {		// GPS Calculation
	gps.f_get_position(&flat, &flon, &age);
	gps.get_datetime(&date, &time, &age);
	gps.stats(&chars, &sentences, &failed);	//should be removed later...just a waste of time
	gps_speed = gps.f_speed_mph();	//not needed also
	if(gps_speed > max_speed) max_speed = gps_speed;
	course = gps.course();	//could be useful once i figure out what it does

	return;
}

void Serial_data_log() {			// Serial Data Logging
	//String() will not handle float/doubles...this will convert these to arrays
	long latitude, longitude, wp_distance, wp_heading, latitude_array, longitude_array;
	latitude = flat*1000000;
	longitude = flon*1000000;
	latitude_array = gps_array[waypoint_num][0]*1000000;
	longitude_array = gps_array[waypoint_num][1]*1000000;
	wp_distance = waypoint_distance;
	wp_heading = waypoint_heading;
	long angle_string;

	//String creation
	dataString += String(millis());
	dataString += ","; dataString += String(time);
	dataString += ","; dataString += String(gps_speed);
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

void LCD_data_log() {				// LCD Data Logging
	lcd1.clear();
	lcd1.setCursor(0,0);	lcd1.print("Lt/Ln: "); lcd1.print(flat,6); lcd1.print(" "); lcd1.print(flon,6);
	lcd1.setCursor(29,0);	lcd1.print("Dft: "); lcd1.print(waypoint_distance,1);
	lcd1.setCursor(0,1);	lcd1.print("NWP:   "); lcd1.print(gps_array[waypoint_num][0],6); lcd1.print(" "); lcd1.print(gps_array[waypoint_num][1],6);
	lcd1.setCursor(29,1);	lcd1.print("Spd: "); lcd1.print(max_speed);
	lcd1.setCursor(36,1);	lcd1.print("M: "); lcd1.print(autonomous);

	lcd2.clear();
	lcd2.setCursor(20,0);	lcd2.print("Tm: "); lcd2.print(millis()/1000);
	lcd2.setCursor(0,1);	lcd2.print("CM: "); lcd2.print(compass_heading,1);
	lcd2.setCursor(10,1);	lcd2.print("Hdng: "); lcd2.print(waypoint_heading,1);
	lcd2.setCursor(22,1);	lcd2.print("Diff: "); lcd2.print(angle_diff,1);
	lcd2.setCursor(33,1);	lcd2.print("WP: "); lcd2.print(waypoint_num);

return;
}