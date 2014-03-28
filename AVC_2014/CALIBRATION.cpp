//CALIBRATION FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
unsigned long click_calibration_counter = 0;
const byte InterruptPin = 2 ;		//interrupt on digital pin 2
long time = 0;


//EXTERNAL VARIABLES
extern double angle_target, x, y;
extern int steer_us;
extern double angle;
extern int mode;
extern long accum;

//OBJECT DECLARATIONS
extern Servo steering;


//PROGRAM FUNCTIONS
void click_calibration(){
	Serial2.println();
	click_calibration_counter = 0;
	pinMode(HALL_EFFECT_SENSOR, INPUT);
	attachInterrupt(HALL_EFFECT_SENSOR, click_calibration_increment, CHANGE);	//according to the teensy documentation, all pins can be interrupts

	get_mode();
	while(mode == MANUAL){
		if((millis() - time) > 1000){
			Serial2.println(click_calibration_counter);
			time = millis();
		}
		get_mode();
	}
	
	detachInterrupt(HALL_EFFECT_SENSOR);
	Serial2.println();
	Serial2.print("Total clicks: ");
	Serial2.println(click_calibration_counter);
	Serial2.println();
	return ;
}

void click_calibration_increment(){
	click_calibration_counter++;
	return ;
}

void steering_calibration(){
	Serial2.println();
	angle_target = 0.0;
		
	steering.attach(STEERING);
	steering.writeMicroseconds(STEER_ADJUST);
	delay(500);
	setup_mpu6050();
	calculate_null();

	Serial2.println("set controller to automatic");
	get_mode();
	while(mode != AUTOMATIC) get_mode();
	accum = 0;	//this is ONLY used to reset the 0 the gyro angle for real (setting angle to 0 does nothing!!! (never forget last year's debacle))
	
	while(mode == AUTOMATIC){
		read_FIFO();
		
		update_steering();
		steering.writeMicroseconds(steer_us);
		
		if((millis() - time) > 200){
			Serial2.print("angle: ");
			Serial2.print(angle,5);
			Serial2.print("\tsteering ms: ");
			Serial2.println(steer_us);
			time = millis();
		}
		get_mode();
	}

	steering.detach();

	return ;
}

void servo_test(){
	steering.attach(STEERING);
	steering.writeMicroseconds(STEER_ADJUST);

	Serial2.println();
	Serial2.println();
	Serial2.println("set CH3 to automatic");

	while(mode != AUTOMATIC) get_mode();
	while(mode == AUTOMATIC){
		for(int pos = 30; pos < 150; pos += 1){	//goes from 0 degrees to 180 degrees in steps of 1 degree
			steering.write(pos);					//tell servo to go to position in variable 'pos'
			Serial2.println(pos);
			delay(15);							//waits 15ms for the servo to reach the position
		}

		for(int pos = 150; pos >= 30; pos -= 1){		//goes from 180 degrees to 0 degrees
			steering.write(pos);					//tell servo to go to position in variable 'pos'
			Serial2.println(pos);
			delay(15);							//waits 15ms for the servo to reach the position
		}
		get_mode();
	}

	steering.detach();
	
	return;
}