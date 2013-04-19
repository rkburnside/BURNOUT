/* 2013 Sparkfun Autonomous Vehicle Competition Code
Vehicle: Wile E. Coyote
Author: Richard Burnside
Verion: 1.0
Date: 4/12/2013

Microcontroller: Arduino mega.
***USE THE MEGA. SERVO AND SOFTWARE SERIAL DO NOT!!! PLAY WELL TOGETHER***
Components: steering servo, speed control servo, GPS unit, bluetooth, magnetometer
Vehicle: Turnigy Beetle RC car with 7.2V battery and 2.4-GHz TX/RX
*/

/* TO DO LIST
- verify cross track error handling works
- build a roll cage
- calibrate the CAR speed to servo angle (little changes seem to make HUGE differences)
- compare GPS points at stake center between bing maps and my own gps. drive the car, with the data logger attached, along hard, physical routes to compare its accuracy.
- create serial driven menu system using serial UI
- add manual over ride switch (i.e. competition mode)
- add ~1.0 second delay after flipping switch
- use bt-serial to get gps wayponits. write them using EEPROM
- incorporate breaking into the throttle routine
- write routine that calculates the distance and direction to next waypoint
- write routine to determine when i have reached a way point
- recalibrate compass
- create on the fly calibration routine for the compass
- verify copmass heading is actually working correctly
- incorporate steering limits so i don't roll the car again
- set gain for steering (i.e. more vs. less agressive steering)
- set gain for throttle (i.e. more vs. less acceleration)
- mail in AVC video
- create shut down function for when the car passes the finish line
- create shut down or reverse function if the car hasn't moved for ~15 seconds from its current spot
- hot glue everything
- use an array to determine GPS points, distances and angles using arrays and get averages of them
- develop a correct method/procedure for programming waypoints and driving them consistently
x - clean up gps code so that i only get the information that i need
x - delete all of the unneeded GPS items that I have listed (alt, speed, course, time, date, etc)
x - (no longer needed now that i am using the mega.) determine GPS output rate. print millis(), get and print gps reading, print millis(), do the math
x - zipties everything
x - create header file with all tweaks i need
x - output the gps info i need to bluetooth
x - use GPS speed and get top speed report
x - use GPS heading to calibrate compass heading
x - (i'm not going to use this now, it shouldn't be an issue.) use latitude/longitude for waypoint acceptance
*/

//Included Libraries
#include <Wire.h>
#include <HMC5883L.h>
#include <Servo.h> 
#include <TinyGPS.h>
#include <math.h>
#include "AVC_2013_Beetle.h"
//#include "EEPROMAnything.h"

//Function and Object Declarations
HMC5883L compass;
TinyGPS gps;
Servo steering;
Servo speed;

void setup(){
	Serial1.begin(115200);		//bluetooth serial
	Serial2.begin(9600);		//gps serial
	steering.attach(SERVO);		//Servo Initialization
	speed.attach(THROTTLE);		//Speed control initialization

	//compass initialization
	Wire.begin(); //Start the I2C interface.
	compass = HMC5883L(); //Construct a new HMC5883 compass.
	compass.SetMeasurementMode(Measurement_Continuous); //Set the measurement mode to Continuous
	
	Serial1.setTimeout(100000);
	Serial1.println("perform calibration? (y=1 / n=0): ");
	while(Serial1.parseInt() == 1) compass_calibration_routine();
	Serial1.setTimeout(1000);
}

void loop(){
	compass_measurement();
	gps_data();
	waypoint();
//	cross_track_calculation();
	set_turn();
	set_speed();

	if((millis() - print_delay) > 250){
		serial_data_log();
		print_delay = millis();
	}	
}

/* these functions need work. consider incorporating XXX code ideas.*/

void set_speed(void){		//test this and delete the delays after it works
	if(waypoint_distance <= 20)									speed.write(SPEED_SLOW);
	if((waypoint_distance > 20) && (waypoint_distance <=40))	speed.write(SPEED_MED);
	if(waypoint_distance > 20)									speed.write(SPEED_MED);
	while(waypoint_num >= waypoint_total) speed.write(SPEED_STOP);		//shuts off the vehicle by setting speed to 0

	return;
}

void set_turn(void){				//Set servo to steer in the direction of the next waypoint
	double servo_angle;
//	right is 0, left is 180.
	compass_heading = -(compass_heading - 90.0);	//switches heading to normal x,y coordinates

	if(compass_heading < 0.0)	compass_heading += 360.0;
	if(compass_heading > 360.0)	compass_heading -= 360.0;

	angle_diff = compass_heading - waypoint_heading - cross_track_error * ERROR_GAIN;
	
	if(angle_diff < -180.0)	angle_diff += 360.0;
	if(angle_diff >= 180.0)	angle_diff -= 360.0;

	servo_angle = angle_diff/2.0 + 90.0;		//changes domain from -180...180 to 0...180
	servo_angle = 180.0 - servo_angle;
	
//	this section sets the servo/turning limits
	if(servo_angle > SERVO_STEERING_LIMIT_LEFT)			steering.write(SERVO_STEERING_LIMIT_LEFT);
	else if(servo_angle < SERVO_STEERING_LIMIT_RIGHT)	steering.write(SERVO_STEERING_LIMIT_RIGHT);
	else												steering.write(servo_angle);

	return;
}

void waypoint(void){				//Distance and angle to next waypoint
	double x, y;

	x = 69.1*(gps_array[waypoint_num][1] - flon) * cos(flat/57.3);
	y = 69.1*(gps_array[waypoint_num][0] - flat);
	
	waypoint_distance = sqrt(pow(x,2) + pow(y,2))*5280.0;	//converts distance to feet
	waypoint_heading = atan2(y,x)*180.0/M_PI;				//180/pi converts from rads to degrees

	if(waypoint_heading < 0) waypoint_heading += 360.0;	//ensures heading is ALWAYS positive
	
	if(waypoint_distance < WAYPOINT_ACCEPT_RANGE) waypoint_num++;	//waypoint acceptance

	return;
}

/*
void cross_track_calculation(void){
	double x1, x2, y1, y2, cross_product;

	if(waypoint_num == 0) cross_track_error = 0.0;
	else {
		x1 = gps_array[waypoint_num-1][1];
		x2 = gps_array[waypoint_num][1];
		y1 = gps_array[waypoint_num-1][0];
		y2 = gps_array[waypoint_num][0];
 
		//create unit vectors
		unit_factor = sqrt(x1*x1 + y1*y1);
		x1 = x1/unit_factor;
		y1 = y1/unit_factor;
		 
		unit_factor = sqrt(x2*x2 + y2*y2);
		x2 = x2/unit_factor;
		y2 = y2/unit_factor;

		cross_product = (x1*y2 - x2*y1);              //normalization of the dot_product
		cross_track_error = asin(cross_product)*180.0/PI;
	}
return;
}
*/

//compass calibration routine
 
void compass_calibration_routine(void){
	int max_x[10], max_y[10], min_x[10], min_y[10];

	Serial1.println("compass calibration will start in 1 seconds");
	delay(1000);
	Serial1.println("begin compass calibration. drive in circles");
	delay(1000);
	
	for(int i=0; i<1000; i++){
		MagnetometerRaw raw = compass.ReadRawAxis();	//get raw 

		if(raw.XAxis < 1000){		//test to see if max axis reading is acceptable
			if(raw.XAxis > -1000)	//test to see if min axis reading is acceptable
				XAxis = raw.XAxis;	//adjust axis with calibration factor
		}
		
		if(raw.YAxis < 1000){		//test to see if max axis reading is acceptable
			if(raw.YAxis > -1000)	//test to see if min axis reading is acceptable
				YAxis = raw.YAxis + compass_y_cal;	//adjust axis with calibration factor
		}
		
		for(int j=0; j<10 && i==0; j++)	max_x[j] = XAxis; //initialization of the array with the current compass values
		for(int j=0; j<10 && i==0; j++) max_y[j] = YAxis; //initialization of the array with the current compass values
		for(int j=0; j<10 && i==0; j++) min_x[j] = XAxis; //initialization of the array with the current compass values
		for(int j=0; j<10 && i==0; j++) min_y[j] = YAxis; //initialization of the array with the current compass values

		if(XAxis > max_x[9]){
			max_x[0] = max_x[1];
			max_x[1] = max_x[2];
			max_x[2] = max_x[3];
			max_x[3] = max_x[4];
			max_x[4] = max_x[5];
			max_x[5] = max_x[6];
			max_x[6] = max_x[7];
			max_x[7] = max_x[8];
			max_x[8] = max_x[9];
			max_x[9] = XAxis;
			}

		if(YAxis > max_y[9]){
			max_y[0] = max_y[1];
			max_y[1] = max_y[2];
			max_y[2] = max_y[3];
			max_y[3] = max_y[4];
			max_y[4] = max_y[5];
			max_y[5] = max_y[6];
			max_y[6] = max_y[7];
			max_y[7] = max_y[8];
			max_y[8] = max_y[9];
			max_y[9] = YAxis;
			}

		if(XAxis < min_x[9]){
			min_x[0] = min_x[1];
			min_x[1] = min_x[2];
			min_x[2] = min_x[3];
			min_x[3] = min_x[4];
			min_x[4] = min_x[5];
			min_x[5] = min_x[6];
			min_x[6] = min_x[7];
			min_x[7] = min_x[8];
			min_x[8] = min_x[9];
			min_x[9] = XAxis;
			}

		if(YAxis < min_y[9]){
			min_y[0] = min_y[1];
			min_y[1] = min_y[2];
			min_y[2] = min_y[3];
			min_y[3] = min_y[4];
			min_y[4] = min_y[5];
			min_y[5] = min_y[6];
			min_y[6] = min_y[7];
			min_y[7] = min_y[8];
			min_y[8] = min_y[9];
			min_y[9] = YAxis;
			}

		Serial1.print(XAxis);		Serial1.print("\t");
		Serial1.print(YAxis);		Serial1.print("\t");
		Serial1.print(min_x[9]);	Serial1.print("\t");
		Serial1.print(max_x[9]);	Serial1.print("\t");
		Serial1.print(min_y[9]);	Serial1.print("\t");
		Serial1.println(max_y[9]);

		delay(8);
	}
	
	compass_x_cal = (max_x[0] + min_x[0])/2;
	compass_y_cal = (max_y[0] + min_y[0])/2;
	
	Serial1.println("compass calibration complete\t");
	for(int j=0; j<10; j++){
		Serial1.print(min_x[j]);	Serial1.print("\t");
		Serial1.print(max_x[j]);	Serial1.print("\t");
		Serial1.print(min_y[j]);	Serial1.print("\t");
		Serial1.println(max_y[j]);
	}

	Serial1.print(compass_x_cal);		Serial1.print("\t");
	Serial1.println(compass_y_cal);
	Serial1.println("rerun calibration routine? y=1 / n=0): ");

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

void gpsdump(TinyGPS &gps){		//GPS Calculation
	gps.f_get_position(&flat, &flon, &age);
	gps.stats(&chars, &sentences, &failed);	//should be removed later...just a waste of time

	if(max_speed < gps.f_speed_mph()) max_speed = gps.f_speed_mph();	//stores maximum speed

	return;
}

void compass_measurement(){
	
	MagnetometerRaw raw = compass.ReadRawAxis();	//get raw 

	//Compass Reading Filtering (sometimes bogus values are read)
	if(raw.XAxis < 1000){		//test to see if max axis reading is acceptable
		if(raw.XAxis > -1000)	//test to see if min axis reading is acceptable
			XAxis = raw.XAxis + compass_x_cal;	//adjust axis with calibration factor
	}
	
	if(raw.YAxis < 1000){		//test to see if max axis reading is acceptable
		if(raw.YAxis > -1000)	//test to see if min axis reading is acceptable
			YAxis = raw.YAxis + compass_y_cal;	//adjust axis with calibration factor
	}
	
	compass_heading = atan2(YAxis, XAxis);	//calculate compass_heading

	compass_heading = PI - compass_heading;
	compass_heading = compass_heading + DECLINATION;
	if(compass_heading < 0.0) compass_heading += 2.0*PI;
	if(compass_heading > (2.0*PI)) compass_heading -= 2.0*PI;

	compass_heading = compass_heading * 180.0/PI;

	return;
}

void serial_data_log(){			//Serial Data Logging
	// Serial1.print(XAxis);				Serial1.print("\t\t");   
	// Serial1.print(YAxis);				Serial1.print("\t\t");
	Serial1.print(waypoint_heading,1);		Serial1.print("\t");   
	Serial1.print(cross_track_error,1);		Serial1.print("\t");
	Serial1.print(compass_heading,1);		Serial1.print("\t");
	Serial1.print(waypoint_distance,1);		Serial1.print("\t");
	Serial1.print(flat,8);					Serial1.print("\t");
	Serial1.print(flon,8); 					Serial1.print("\t");
	Serial1.print(max_speed,1); 			Serial1.print("\t");
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