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

/* How servos work
pulse every 20ms, pulse lenght is between 1.25ms (0-deg) and 1.75ms (180-deg) with 1.50ms (90-deg) being neutral
arduion reads pulses from Hitec radio as follows:

		low		neutral	high
ch1 -	1145	1515	1884
ch2 -	1145	1515	1884
ch3 -	1145	1515	1884 
ch4 -	1145	1515	1884
ch5 -	1130	1515	1884 <-- push switch toward batteries is 1130, pull switch to front is 1884	
ch6 -	1395	1510	1615
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
#include <Servo.h> 
#include <math.h>
#include <TinyGPS.h>

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

//GPS is connected to Serial1 RX pin of arduino mega -- YELLOW/GREEN IS TX FROM THE GPS TO RX ON THE ARDUINO
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
	steering.attach(SERVO);		//Servo Initialization
	speed.attach(THROTTLE);		//Speed control initialization
	
	//Serial Output Initialization
	Serial.begin(115200);
	Serial.println("Time,IR Distance,Ultrasonic Distance, Gyro Heading, Lat, Lon, Date, Time, Course, Speed, Failed");

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
	
	Serial.println("gyro null calculation begin");

	delay(500);
}

void loop() {
	//GPS data update
	bool newdata = false;
	if(feedgps()) newdata = true;
	if(newdata) gpsdump(gps);

	waypoint();
	compass();
	set_turn();
	set_speed();

	if((millis() - temp_time) > 250) {
		auto_control();					// check for autonomous mode 4x / second
		//		Serial_data_log();
		temp_time = millis();
	}
}

/*----------unfinished code----------*/
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

/*---------completed functions---------*/
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
Sparkfun Race Course ***USE THIS ONE!!!***
{{40.065199508645286, -105.20982731314206}, // 1st corner
{40.065136900410714, -105.2097642812302}, // 1st line up point
{40.065066081190785, -105.2097629401257}, // 2nd line up point or start of straight line
{40.06152208395571, -105.20980853767887}, // long distance point for direction
{40.06453058970293, -105.2097763511701}, // lat point for start of turn
{40.06448686920148, -105.20985212357533}, // 2nd corner, just inside of south parking lot
{40.06445080617122, -105.21016393037269}, // bottom middle of south parking lot
{40.06449104008855, -105.21041203470733}, // inbetween sidewalks for 3rd corner
{40.06450670940305, -105.21046836109664}, // 3rd corner
{40.06465758628262, -105.21046433778312}, // line up for inbetween next bottleneck
{40.06475817068338, -105.21046970220115}, // past the bottleneck
{40.06493060073908, -105.21048043103721}, // middle of the 3rd side
{40.06514305860027, -105.21048311324623}, // end of 3rd side
{40.06519745591719, -105.21041069360282}, // 4th corner
{40.06518985782108, -105.21001238556421}}; // way past the finish line

Work Course
{{39.64625554553962, -105.08055895510579},
{39.64624728435262, -105.08022636118794},
{39.646387724397506, -105.08025050106907},
{39.64651164184729, -105.08025050106907},
{39.64655914347748, -105.08040875140095},
{39.646493054243976, -105.08057773056889},
{39.646312823084216, -105.08063137474922}};

SPARKFUN ACTUAL ROUTE - old
{{40.065170,-105.209709}, //1st corner
{40.065040,-105.209709}, //mid 1-2 stretch
{40.064849,-105.209777}, //mid 1-2 stretch
{40.064678,-105.209716}, //mid 1-2 stretch
{40.064521,-105.209777}, //2nd corner
{40.064460,-105.210121}, //mid 2-3 stretch
{40.064510,-105.210418}, //3rd corner
{40.064689,-105.210441}, //mid 3-4 stretch
{40.064918,-105.210479}, //mid 3-4 stretch
{40.065170,-105.210441}, //4th corner
{40.065170,-105.209793}}; //final point

{{39.53869644189598, -105.011342275864},
{39.538756428534754, -105.0113261826099},
{39.53879159309199, -105.01129667831074},
{39.53990661599784, -105.01013125849373}, // used for large compass heading
{39.539091634185084, -105.01098420096034}, // should be used for lat/lon point
{39.53912048770903, -105.01094966751911},
{39.53914427537494, -105.01088529450276},
{39.53913910414391, -105.01080751044132},
{39.53906108399808, -105.010736096627},
{39.538867610530026, -105.01043300700806},
{39.53876735711323, -105.01024257016824},
{39.538710404308084, -105.01010041309021},
{39.53861939001685, -105.01014064622544},
{39.53846063184789, -105.0103203542293},
{39.53831583580741, -105.01044373584398},
{39.53831952464619, -105.01056175304089},
{39.53835779219851, -105.01067842913304},
{39.538375822775116, -105.01085411382326},
{39.53864472923475, -105.01122694087636}};

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

Sparkfun Race Course - new bing map
{{40.06519806873442,-105.20985681744139},
{40.065144697782415,-105.20975489349878},
{40.06504821941771,-105.20975489349878},
{40.06492505534932,-105.20976025791681},
{40.06472799237675,-105.20976562233484},
{40.06455761455558,-105.20977366896189},
{40.06448166285943,-105.20979244442499},
{40.064454977108255,-105.21014113159693},
{40.064502190353224,-105.21048981876886},
{40.06471978140714,-105.21046299667871},
{40.06490452513427,-105.21047707827601},
{40.065128275942754,-105.2104790899328},
{40.06519601600586,-105.21043081017054},
{40.06518985782108,-105.21001238556421}};


*/