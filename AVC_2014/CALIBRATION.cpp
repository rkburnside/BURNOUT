//AVC_2014 multi-file branch, calibaration functions

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
unsigned int click_calibration_counter = 0;
const byte InterruptPin = 2 ;		//interrupt on digital pin 2


//EXTERNAL VARIABLES
extern double angle_target, x, y;
extern bool manual, automatic, aux, running, first;
extern long time;
extern int steer_us;
extern double angle;


//OBJECT DECLARATIONS
extern Servo steering;


//PROGRAM FUNCTIONS
void click_calibration(){
	Serial.println();
	click_calibration_counter = 0;
	pinMode(InterruptPin, INPUT);
	attachInterrupt(0, click_calibration_increment, CHANGE);	//interrupt 0 is on digital pin 2
	get_mode();
	while(manual){
		if((millis()-time)>1000){
			Serial.println(click_calibration_counter);
			time = millis();
		}
		get_mode();
	}
	
	detachInterrupt(0);
	Serial.println();
	Serial.print("Total clicks: ");
	Serial.println(click_calibration_counter);
	Serial.println();
	return ;
}

void click_calibration_increment(){
    click_calibration_counter++;
	return ;
}

void steering_calibration(){
	Serial.println();
	angle_target = 0.0;
		
	steering.attach(10);
	steering.writeMicroseconds(STEER_ADJUST);
	delay(500);
	setup_mpu6050();
	calculate_null();

	Serial.println("set controller to automatic");
	get_mode();
	while(!automatic) get_mode();
	
	while(automatic){
		read_FIFO();
		
		update_steering();
		steering.writeMicroseconds(steer_us);
		
		if((millis()-time)>200){
			Serial.print("angle: ");
			Serial.print(angle,5);
			Serial.print("\tsteering ms: ");
			Serial.println(steer_us);
			time = millis();
		}
		get_mode();
	}

	steering.detach();

	return ;
}







