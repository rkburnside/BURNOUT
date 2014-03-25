//MINUTEMAN / ROADRUNNER COMPETITION CODE

//#INCLUDE FILES
/* the arduino ide is honky...
see: http://jamesreubenknowles.com/including-libraries-in-the-arduino-ide-1625
Any Arduino libraries that are needed by the files outside of the sketch (.ino) file must also be listed in the sketch file itself.
The sketch is parsed for include files. The sketch, all included header files, and the corresponding source files, are copied to another directory for compiling. From that directory, library-based include files are NOT available unless they are included in the sketch and copied to the build directory.
*/

#include "DECLARATIONS.h"
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

//the following are for reseting the teensy. see this link for more details: http://goo.gl/nV8kMs
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

//INTERNAL VARIABLES
bool running = false, first = true;
volatile int clicks = 0;
int mode = MANUAL;

//EXTERNAL VARIABLES
extern byte wpr_count;
extern int steer_us;
extern long accum; //this is ONLY used to reset the 0 the gyro angle for real (setting angle to 0 does nothing!!! (never forget last year's debacle))
extern double x_wp, y_wp;
extern double target_x, target_y;
extern double angle_target, x, y;
extern position_structure waypoint;


//OBJECT DECLARATIONS
Servo steering, esc;

//PROGRAM FUNCTIONS
void encoder_interrupt(){
	clicks++;
	return ;
}

void navigate(){
	calculate_speed();
	cal_steer_lim();
	update_position();
	update_cross_product();
	update_steering();
	update_waypoint();
	get_mode();
	if(mode == AUTOMATIC){
		steering.writeMicroseconds(steer_us);
		speed();
	}
	return ;
}

void get_mode(){
	int mode_1 = 0, mode_2 = 0;

	mode_1 = digitalRead(MODE_LINE_1);
	mode_2 = digitalRead(MODE_LINE_2);
	
	if((mode_1 == HIGH) && (mode_2 == HIGH)) mode = MANUAL;
	else if((mode_1 == LOW) && (mode_2 == HIGH)) mode = AUTOMATIC;
	else if((mode_1 == LOW) && (mode_2 == LOW)) mode = WP_MODE;
	else if((mode_1 == HIGH) && (mode_2 == LOW)) mode = RESET;

	if(mode == RESET){	//performs a software reset on the M4. see this post: http://goo.gl/nV8kMs
		// 0000101111110100000000000000100
		// Assert [2]SYSRESETREQ
		delay(1000);
		WRITE_RESTART(0x5FA0004);
	}
	
	return ;	
}

void setup(){
	Wire.begin();

	Serial.begin(115200);
	Serial.setTimeout(100000);
	Serial.println(CAR_NAME);
	Serial.println();

	Serial1.begin(115200);
	Serial1.setTimeout(100000);
	Serial1.println(CAR_NAME);
	Serial1.println();

   	//Pin assignments:
	pinMode(MODE_LINE_1, INPUT);
	pinMode(MODE_LINE_2, INPUT);
	pinMode(TOGGLE, INPUT);
	digitalWrite(TOGGLE, HIGH);

	main_menu();
	delay(500);
	
	setup_mpu6050();
	calculate_null();

	pinMode(HALL_EFFECT_SENSOR, INPUT);	 
	attachInterrupt(HALL_EFFECT_SENSOR, encoder_interrupt, CHANGE);	//according to the teensy documentation, all pins can be interrupts

	steering.attach(STEERING);
	steering.writeMicroseconds(STEER_ADJUST);
	esc.attach(THROTTLE);
	esc.writeMicroseconds(S1);

	race_startup_routine();
	
}

void loop(){
	//watch_angle();
	read_FIFO();
	//watch_gyro();
	/* in the main loop here, we should wait for thing to happen, then act on them. Watch clicks and wait for it to reach CLICK_MAX, then calculate position and such.*/
	get_mode();

	if(clicks >= CLICK_MAX){
		clicks = 0;
		navigate();
	}

	if(mode == AUTOMATIC){	//this function get the car started moving and then clicks will take over
		if(!running){
			esc.write(S2);	//i don't understand this function...help...i changed this to S1 so the car is stationary?
			running = true;
		}
		if(first){
			accum = 0;		//zeros out the accumulator which zeros out the angle
			first = false;
		}
	}
	
	if(mode == MANUAL){	//this function makes the car be stationary when in manual waypoint setting mode
		if(running){
			esc.write(S1);	//i changed this to S1 so the car is stationary?
			running = false;
		}
	}

	if(wpr_count >= WAYPOINT_COUNT){	//this locks the car into this loop and makes it stationary WHEN the course is completed
		esc.writeMicroseconds(S1);
		while(true);
	}
	
	if(mode == WP_MODE){
		long temp = millis();
		while(mode == WP_MODE){
			get_mode();
			read_FIFO();
		}

		if((millis() - temp) > 500) set_waypoint();
	}
	
	static long time = 0;
	if((millis() - time) > 500){
		print_coordinates();
		time = millis();
	}
}

void race_startup_routine(){
	//verify that car is in automatic mode
	get_mode();
	if(mode == MANUAL){
		Serial.println();
		Serial.println("1. SET CAR TO MODE 1 / AUTOMATIC / PRESS CH3 TO CONTINUE");
		Serial.println();
	}
	while(mode == MANUAL){
		get_mode();		//waits until radio is set to automatic
		read_FIFO();
	}

	//by turning off the radio, the automatic mode is locked in
	Serial.println("2. TURN OFF THE RADIO!");
	Serial.println();
	delay(2500);

	Serial.println("***READY TO RUN***");
	Serial.println("3. FLIP THE SWITCH TO START THE RACE!");
	Serial.println();

	//determines the current state and waits for it to change to start the race
	int toggle_state = digitalRead(TOGGLE);
	int mode_state = mode;
	while((toggle_state == digitalRead(TOGGLE)) && (mode == mode_state)){	//waits for the switch to be flipped OR for the mode to change
		get_mode();		//waits until the switch is flipped to start the race
		read_FIFO();
	}

	for(int i=0; i<100; i++){	//clears the FIFO buffer and waits 1 sec to start
		delay(1);
		read_FIFO();
	}

	//the following zeros out everything and sets the waypoint and counter
	wpr_count = 1;		//set waypoint read counter to first waypoint
	EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
	x_wp = waypoint.x;
	y_wp = waypoint.y;

	x=0;
	y=0;
	accum=0;			//***ZEROS out the accumulator which zeros out the gyro angle
	clicks = 0;
	first = true;
	target_x = x_wp;
	target_y = y_wp;

	return;
}