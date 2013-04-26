/* 2013 Sparkfun Autonomous Vehicle Competition Code
Vehicle: Wile E. Coyote
Author: Richard Burnside
Verion: 1.0
Date: 4/21/2013

Microcontroller: Arduino mega.
***USE THE MEGA. SERVO AND SOFTWARE SERIAL DO NOT!!! PLAY WELL TOGETHER***
Components: steering servo, speed control servo, GPS unit, bluetooth, magnetometer
Vehicle: Turnigy Beetle RC car with 7.2V battery and 2.4-GHz TX/RX
*/

/* TO DO LIST
- (H) create reset switch for arduino
- (H) hot glue everything
- (H) build a roll cage
- (H) add manual over ride switch (i.e. competition mode)
- (H) use bt-serial to get gps wayponits. write them using EEPROM
- (H) create and implement cross track error handling
- (H) create function to calculate unit vectors and use that in cross_track_error
- (M) calibrate the CAR speed to servo angle (little changes seem to make HUGE differences)
- (M) create serial driven menu system
- (M) add ~1.0 second delay after flipping switch
- (M) incorporate breaking into the throttle routine
- (M) set gain for steering (i.e. more vs. less agressive steering)
- (M) set gain for throttle (i.e. more vs. less acceleration)
- (M) mail in AVC video
- (L) reincorporate the compass. read bot thoughts tutorial on compass calibration.
- (L) develop a correct method/procedure for programming waypoints and driving them consistently
- (L) compare GPS points at stake center between bing maps and my own gps. drive the car, with the data logger attached, along hard, physical routes to compare its accuracy.
- (L) verify copmass heading is actually working correctly
- (L) recalibrate compass
x - write routine to determine when i have reached a way point
x - write routine that calculates the distance and direction to next waypoint
x - (not going to implement) create shut down or reverse function if the car hasn't moved for ~15 seconds from its current spot
x - create shut down function for when the car passes the finish line
x - incorporate steering limits so i don't roll the car again
x - create on the fly calibration routine for the compass
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
#include "AVC_2013_Beetle.h"
#include <Wire.h>
#include <L3G.h>
#include <Servo.h> 
#include <TinyGPS.h>
#include <math.h>
//#include "EEPROMAnything.h"

//Function and Object Declarations
TinyGPS gps;
Servo steering;
Servo speed;
L3G gyro;

void setup(){
	Serial1.begin(115200);		//bluetooth serial
	Serial2.begin(115200);		//gps serial
	steering.attach(SERVO);		//Servo Initialization
	speed.attach(THROTTLE);		//Speed control initialization

	//gyro setup
	Wire.begin();
	if (!gyro.init()) {
		Serial1.println("Failed to autodetect gyro type!");
		while (1);
	}

	Serial1.setTimeout(100000);
	Serial1.println("is car aligned with true north for calibration? (y=1 / n=0): ");
	while(Serial1.parseInt() != 1) Serial1.println("align car with true north");

	gyro.enableDefault();
	Serial1.println("gyro null being calculated");
	gyro_null_cal();
	delay(1000);
	
	// Serial1.println("perform gyro calibration? (y=1 / n=0): ");
	// while(Serial1.parseInt() == 1) gyro_calibration();
	// Serial1.setTimeout(1000);

//	heading_calibration();
	gyro_sum = (90.0/360.0)*GYRO_CALIBRATION_NUMBER;
}

void loop(){
	//sample gyro
	if((millis() - sample_time) > GYRO_SAMPLING_RATE){
		sample_time = millis();
		angle_update();
	}

	gps_data();
	waypoint();
//	cross_track_calculation();
	set_turn();
	set_speed();

	if((millis() - print_delay) > 100){
		serial_data_log();
		print_delay = millis();
	}	
}


/*
void heading_calibration(void){
/* i need this function to do the following:
1. have the car drive at heading 0
2. adjust the steering servo to make it drive at heading 0
3. record the starting gps location
4. drive for 5 seconds and record the next gps location
5. make sure that the gps location has changed
6. find the heading using those 2 gps locations
7. change the gyro heading to be a the angle calculated

	while(1){
		

		//sample gyro
		if((millis() - sample_time) > GYRO_SAMPLING_RATE){
			sample_time = millis();
			angle_update();
		}

		gps_data();
	}
}
*/




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

	angle_diff = gyro_angle - waypoint_heading;
	//incorporate eventually - cross_track_error * ERROR_GAIN;

	if(angle_diff < -180.0)	angle_diff += 360.0;
	if(angle_diff >= 180.0)	angle_diff -= 360.0;

	servo_angle = 180.0 - (angle_diff/2.0 + 90.0);	//changes domain from -180...180 to 0...180
	
//	this section sets the servo/turning limits
	if(servo_angle > SERVO_STEERING_LIMIT_LEFT)			steering.write(SERVO_STEERING_LIMIT_LEFT);
	else if(servo_angle < SERVO_STEERING_LIMIT_RIGHT)	steering.write(SERVO_STEERING_LIMIT_RIGHT);
	else												steering.write(servo_angle);

	return;
}

/* Cross Track Error
void unit_vector_calculation(void){
	double x1, x2, y1, y2;

	for(int j=0; j<waypoint_total; j++){
		if(j == 0) unit_vectors[0] = {0,0];
		else {
		x1 = gps_array[j][1];
		x2 = gps_array[j+1][1];
		y1 = gps_array[j][0];
		y2 = gps_array[j+1][0];
 
		//create unit vectors
		unit_factor = sqrt(x1*x1 + y1*y1);
		x1 = x1/unit_factor;
		y1 = y1/unit_factor;
		 
		// cross_product = (x1*y2 - x2*y1);              //normalization of the dot_product
		// cross_track_error = asin(cross_product)*180.0/PI;
	}
return;
}
*/

/*Working Functions That Are Working Well*/

//Gyro routines
void angle_update(void){
	gyro.read();

	gyro_sum = gyro_sum - (gyro.g.z - gyro_null);// - (3890318.0/4.0*360); //changes orientation from true north to (X,Y)

	if(gyro_sum > GYRO_CALIBRATION_NUMBER) gyro_sum = gyro_sum - GYRO_CALIBRATION_NUMBER;
	if(gyro_sum < 0.0) gyro_sum = gyro_sum + GYRO_CALIBRATION_NUMBER;

	gyro_angle = gyro_sum / GYRO_CALIBRATION_NUMBER * 360.0;
}

void gyro_calibration(void){
	while(1){
		if((millis() - sample_time) > GYRO_SAMPLING_RATE){
			sample_time = millis();
			gyro.read();
			gyro_sum = gyro_sum - (gyro.g.z - gyro_null);
			gyro_angle = gyro_sum / GYRO_CALIBRATION_NUMBER * 360.0;
		}
		
		if((millis() - print_delay) > 100){
			print_delay = millis();
			Serial1.print((int)gyro.g.z);
			Serial1.print("\t\t");
			Serial1.print(gyro_sum);
			Serial1.print("\t\t");
			Serial1.println(gyro_angle);
		}
	}
}

//GPS data update
void gyro_null_cal() {
	for(int i = 0; i<30; i++) {
		gyro.read();
		delay(GYRO_SAMPLING_RATE);
	}
		
	for(int i = 0; i<500; i++) {
		gyro.read();
		gyro_null = gyro_null + gyro.g.z;
		Serial1.print((int)gyro.g.z);
		Serial1.print("\t\t");
		Serial1.println(gyro_null);
		delay(GYRO_SAMPLING_RATE);
	}
	
	gyro_null = gyro_null / 500.0;
	Serial1.print("\n\ngyro_null is: ");
	Serial1.println(gyro_null);
	Serial1.println();
	delay(1000);
}

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

void serial_data_log(){			//Serial Data Logging
	Serial1.print(gyro_angle,1);			Serial1.print("\t");
	Serial1.print(waypoint_heading,1);		Serial1.print("\t");   
	// Serial1.print(cross_track_error,1);	Serial1.print("\t");
	Serial1.print(waypoint_distance,1);		Serial1.print("\t");
	Serial1.print(flat,8);					Serial1.print("\t");
	Serial1.print(flon,8); 					Serial1.print("\t");
	Serial1.print(max_speed,1); 			Serial1.print("\t");
	Serial1.println();

	return;
}
