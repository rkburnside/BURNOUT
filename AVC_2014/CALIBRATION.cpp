//CALIBRATION FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
unsigned long click_calibration_counter = 0;
long time = 0;


//EXTERNAL VARIABLES
extern double angle_target, x, y;
extern int steer_us;
extern double angle;
extern int mode;
extern long accum;

//OBJECT DECLARATIONS
extern Servo steering, esc;


//PROGRAM FUNCTIONS
void click_calibration(){
	SERIAL_OUT.println();
	click_calibration_counter = 0;
	pinMode(HALL_EFFECT_SENSOR, INPUT);
	attachInterrupt(HALL_EFFECT_SENSOR, click_calibration_increment, CHANGE);	//according to the teensy documentation, all pins can be interrupts

	get_mode();
	while(mode == MANUAL){
		get_mode();
	}
	
	detachInterrupt(HALL_EFFECT_SENSOR);
	SERIAL_OUT.println();
	SERIAL_OUT.print("Total clicks: ");
	SERIAL_OUT.println(click_calibration_counter);
	SERIAL_OUT.println();
	return ;
}

void click_calibration_increment(){
	click_calibration_counter++;
	SERIAL_OUT.println(click_calibration_counter);
	return ;
}

void steering_calibration(){
	esc.detach();

	SERIAL_OUT.println();
	angle_target = 0.0;
		
	steering.attach(STEERING);
	steering.writeMicroseconds(STEER_ADJUST);

	delay(500);
	setup_mpu6050();
	calculate_null();

	SERIAL_OUT.println("set controller to automatic");
	get_mode();
	while(mode != AUTOMATIC) get_mode();
	accum = 0;	//this is ONLY used to reset the 0 the gyro angle for real (setting angle to 0 does nothing!!! (never forget last year's debacle))
	
	delay(250);
	esc.attach(THROTTLE);
	delay(1000);
	esc.writeMicroseconds(S1);
	delay(1000);
	esc.writeMicroseconds(S2);
	
	while(mode != AUX){
		read_FIFO();
		
		update_steering();
		steering.writeMicroseconds(steer_us);
		
		if((millis() - time) > 200){
			SERIAL_OUT.print("angle: ");
			SERIAL_OUT.print((angle*180.0/3.1415),5);
			SERIAL_OUT.print("\tsteering ms: ");
			SERIAL_OUT.println(steer_us);
			time = millis();
		}
		get_mode();
	}

	steering.detach();
	esc.detach();
	
	return ;
}

void servo_test(){
	steering.attach(STEERING);
	steering.writeMicroseconds(STEER_ADJUST);

	esc.attach(THROTTLE);
	esc.write(90);

	SERIAL_OUT.println();
	SERIAL_OUT.println();
	SERIAL_OUT.println("set CH3 to AUTOMATIC");

	while(mode != AUTOMATIC) get_mode();
	while(mode == AUTOMATIC){
		SERIAL_OUT.println("normal angle output");
		for(int pos = 30; pos < 150; pos += 1){	//goes from 0 degrees to 180 degrees in steps of 1 degree
			steering.write(pos);					//tell servo to go to position in variable 'pos'
			SERIAL_OUT.println(pos);
			delay(15);							//waits 15ms for the servo to reach the position
		}

		for(int pos = 150; pos >= 30; pos -= 1){		//goes from 180 degrees to 0 degrees
			steering.write(pos);					//tell servo to go to position in variable 'pos'
			SERIAL_OUT.println(pos);
			delay(15);							//waits 15ms for the servo to reach the position
		}

		SERIAL_OUT.println("microsecond angle output");
		for(int pos = 1250; pos < 1750; pos += 1){	//goes from 0 degrees to 180 degrees in steps of 1 degree
			steering.writeMicroseconds(pos);					//tell servo to go to position in variable 'pos'
			SERIAL_OUT.println(pos);
			delay(5);							//waits 15ms for the servo to reach the position
		}

		for(int pos = 1750; pos >= 1250; pos -= 1){		//goes from 180 degrees to 0 degrees
			steering.writeMicroseconds(pos);					//tell servo to go to position in variable 'pos'
			SERIAL_OUT.println(pos);
			delay(5);							//waits 15ms for the servo to reach the position
		}

		get_mode();
	}

	esc.detach();
	steering.detach();
	
	return;
}

void mode_test(){
	SERIAL_OUT.println();
	SERIAL_OUT.println();
	SERIAL_OUT.println("toggle mode (i.e. TX CH3) and this function will print its current state");
	SERIAL_OUT.println("perform hard reset to exit function");
	esc.detach();
	pinMode(THROTTLE, OUTPUT);
	digitalWrite(THROTTLE, LOW);
	bool attached = true;
	long time_temp = 0;

	while(1){
		get_mode();
		if(mode == MANUAL){
			if(attached == true){
				esc.detach();
				// pinMode(THROTTLE, OUTPUT);
				// digitalWrite(THROTTLE, LOW);
				attached = false;
			}
		}

		else if(mode == AUTOMATIC){
			if(attached == false){
				delay(250);
				esc.attach(THROTTLE);
				delay(250);
				esc.writeMicroseconds(S1);
				attached = true;
			}
		}

		else if(mode == AUX){
			if(attached == true){
				esc.detach();
				// pinMode(THROTTLE, OUTPUT);
				// digitalWrite(THROTTLE, LOW);
				attached = false;
			}
		}
		
		else if(mode == WP_MODE){
			if(attached == true){
				esc.detach();
				// pinMode(THROTTLE, OUTPUT);
				// digitalWrite(THROTTLE, LOW);
				attached = false;
			}
		}
		
		else SERIAL_OUT.println("nothing valid detected");
		
		if((millis() - time_temp) > 250){
			SERIAL_OUT.println(mode);
			time_temp = millis();
		}
	}
	
	return ;	
}

void toggle_test(){
	SERIAL_OUT.println();
	SERIAL_OUT.println();
	SERIAL_OUT.println("toggle the switch and this function will print its current state");
	SERIAL_OUT.println("toggle CH3 to exit routine");

	SERIAL_OUT.println();
	get_mode();
	if(mode != MANUAL) SERIAL_OUT.println("set CH3 to MANUAL");
	while(mode != MANUAL) get_mode();
	while(mode == MANUAL){
		if(digitalRead(TOGGLE) == HIGH) SERIAL_OUT.println("state: HIGH");
		else if(digitalRead(TOGGLE) == LOW) SERIAL_OUT.println("state: LOW");
		else SERIAL_OUT.println("bad reading");
		
		delay(250);
		get_mode();
	}
	
	SERIAL_OUT.println();SERIAL_OUT.println();SERIAL_OUT.println();
	return;
}

void mode_and_toggle_test(){
	SERIAL_OUT.println();
	SERIAL_OUT.println();
	SERIAL_OUT.println("this function will determine the switch and toggle positions");
	SERIAL_OUT.println("perform hard reset to exit function");

	//determines the current state and waits for it to change to start the race
	int toggle_state = digitalRead(TOGGLE);
	int mode_state = mode;

	SERIAL_OUT.println("switch state = ");
	SERIAL_OUT.println(toggle_state);
	SERIAL_OUT.println("mode state = ");
	SERIAL_OUT.println(mode_state);
	
	while(1){
		if((toggle_state == digitalRead(TOGGLE)) && (mode == mode_state)) SERIAL_OUT.println("switch or mode has not changed");
		else SERIAL_OUT.println("something changed");
		get_mode();		//waits until the switch is flipped to start the race
		delay(250);
	}

	return;
}

void activate_the_frickin_laser(){
	SERIAL_OUT.println();
	SERIAL_OUT.println();
	SERIAL_OUT.println("set CH3 to AUX to activate the FRICKIN LASER");
	SERIAL_OUT.println("toggle CH3 to exit the routine");
	get_mode();
	while(mode != AUX) get_mode();
	while(mode == AUX){
		digitalWrite(FRICKIN_LASER, HIGH);
		get_mode();
	}

	digitalWrite(FRICKIN_LASER, LOW);
	
	return;
}