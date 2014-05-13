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
//#include <i2c_t3.h>
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
extern double x, y;
extern position_structure waypoint;


//OBJECT DECLARATIONS
Servo steering, esc;


//PROGRAM FUNCTIONS
void encoder_interrupt(){
	clicks++;
	return ;
}

void reset_requested_interrupt(){
	//performs a software reset on the M4. see this post: http://goo.gl/nV8kMs
	// 0000101111110100000000000000100
	// Assert [2]SYSRESETREQ
	delay(1000);
	WRITE_RESTART(0x5FA0004);
	
	return;
}

void navigate(){
	calculate_speed();
	//map_rates();
	//cal_steer_lim();
	update_position();
	update_waypoint();
	calculate_look_ahead();
	update_steering();
	get_mode();
	if(mode == AUTOMATIC){
		steering.writeMicroseconds(steer_us);
		//speed();
		esc.writeMicroseconds(S4);
	}
	print_data();
	return ;
}

void get_mode(){
	int mode_1 = digitalRead(MODE_LINE_1);
	int mode_2 = digitalRead(MODE_LINE_2);
	
	if((mode_1 == HIGH) && (mode_2 == HIGH)) mode = MANUAL;
	else if((mode_1 == LOW) && (mode_2 == HIGH)) mode = AUTOMATIC;
	else if((mode_1 == LOW) && (mode_2 == LOW)) mode = WP_MODE;
	else if((mode_1 == HIGH) && (mode_2 == LOW)) mode = AUX;

	return ;	
}

void race_startup_routine(){
//	activate_the_frickin_laser();
	SERIAL_OUT.println("-----RACE SETUP ROUTINE-----");
	
	esc.detach();

	setup_mpu6050();
	calculate_null();
	
	SERIAL_OUT.println();

	//verify that car is in automatic mode
	get_mode();
	if(mode == MANUAL){
		SERIAL_OUT.println();
		SERIAL_OUT.println("1. SET CAR TO AUTOMATIC");
		SERIAL_OUT.println();
	}
	while(mode != AUTOMATIC){
		static long time = millis();
		if((millis() - time) > 250){
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			time = millis();
		}
		get_mode();		//waits until radio is set to automatic
		read_FIFO();
	}

	delay(250);
	esc.attach(THROTTLE);
	delay(250);
	esc.writeMicroseconds(S1);


	//by turning off the radio, the automatic mode is locked in
	SERIAL_OUT.println("2. TURN OFF THE RADIO AND FLIP THE SWITCH TO START THE RACE!");
	SERIAL_OUT.println();
	delay(1250);

	SERIAL_OUT.println("3. ***READY TO RUN***");
	SERIAL_OUT.println();
	SERIAL_OUT.println();

	digitalWrite(LED_BUILTIN, HIGH);	//this is used to indicate that the car is ready to run

	//determines the current state and waits for it to change to start the race
	get_mode();
	int toggle_state = digitalRead(TOGGLE);
	int mode_state = mode;
	while((toggle_state == digitalRead(TOGGLE)) && (mode == mode_state)){	//waits for the switch to be flipped OR for the mode to change
		get_mode();		//waits until the switch is flipped to start the race
		read_FIFO();
	}

	digitalWrite(LED_BUILTIN, LOW);		//turn off LED since car is going to run

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
	reset_FIFO();
	accum=0;			//***ZEROS out the accumulator which zeros out the gyro angle
	clicks = 0;
	first = true;
	target_x = x_wp;
	target_y = y_wp;

	return;
}

void wp_setup_routine(){
	SERIAL_OUT.println("-----WP SETUP ROUTINE-----");

	esc.detach();

	setup_mpu6050();
	calculate_null();
	
	SERIAL_OUT.println();
	//verify that car is in MANUAL mode
	get_mode();
	if(mode != MANUAL){
		SERIAL_OUT.println();
		SERIAL_OUT.println("SET CAR TO MANUAL MODE");
		SERIAL_OUT.println();
	}
	while(mode != MANUAL){
		static long time = millis();
		if((millis() - time) > 250){
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			time = millis();
		}
		get_mode();		//waits until radio is set to automatic
		read_FIFO();
	}

	delay(250);
	esc.attach(THROTTLE);
	delay(250);
	esc.writeMicroseconds(S1);

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
	reset_FIFO();
	accum=0;			//***ZEROS out the accumulator which zeros out the gyro angle
	clicks = 0;
	first = true;
	target_x = x_wp;
	target_y = y_wp;

	digitalWrite(LED_BUILTIN, HIGH);	//this is used to indicate that the car is ready to run

	SERIAL_OUT.println("***READY TO SET WAYPOINTS***");
	SERIAL_OUT.println();
	SERIAL_OUT.println();

	return;
}

void setup(){
	SERIAL_OUT.begin(115200);
	SERIAL_OUT.setTimeout(100000);
	SERIAL_OUT.println(CAR_NAME);
	SERIAL_OUT.println();

	//Pin assignments:
	pinMode(MODE_LINE_1, INPUT);
	pinMode(MODE_LINE_2, INPUT);
	pinMode(TOGGLE, INPUT_PULLUP);			//this is the switch that needs to be toggled to start the race
	pinMode(FRICKIN_LASER, OUTPUT);
	digitalWrite(FRICKIN_LASER, LOW);

	pinMode(RESET_PIN, INPUT);
	attachInterrupt(RESET_PIN, reset_requested_interrupt, RISING);	//according to the teensy documentation, all pins can be interrupts

	pinMode(HALL_EFFECT_SENSOR, INPUT);	 
	attachInterrupt(HALL_EFFECT_SENSOR, encoder_interrupt, CHANGE);	//according to the teensy documentation, all pins can be interrupts

	steering.attach(STEERING);
	steering.writeMicroseconds(STEER_ADJUST);

	esc.attach(THROTTLE);
	delay(250);
	esc.writeMicroseconds(S1);
	
	bool bypass_menu = false;
	pinMode(LED_BUILTIN, OUTPUT);
	long temp_time = millis();
	for(int i = 0; i<8; i++){
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		delay(500);
		get_mode();
		if(mode == WP_MODE) bypass_menu = true;
	}

	if(bypass_menu) wp_setup_routine();
	else{
		main_menu();
		race_startup_routine();
	}
}

void loop(){
	/* in the main loop here, we should wait for thing to happen, then act on them. Watch clicks and wait for it to reach CLICK_MAX, then calculate position and such.*/
	read_FIFO();
	get_mode();

	if(clicks >= CLICK_MAX){
		clicks = 0;
		navigate();
	}

	if(mode == AUTOMATIC){	//this function get the car started moving and then clicks will take over
	SERIAL_OUT.println("auto mode");
	if(first){
			accum = 0;		//zeros out the accumulator which zeros out the angle
			reset_FIFO();
			first = false;
		}

		if(!running){	//this will kick start the car/get it moving when it first starts the race
			esc.writeMicroseconds(S2);
			running = true;
		}
	}
	
	if(mode == WP_MODE){
	SERIAL_OUT.println("auto mode");
		long temp = millis();
		while(mode == WP_MODE){
			get_mode();
			read_FIFO();
		}

		if((millis() - temp) > 500) set_waypoint();
	}

	if((wpr_count >= WAYPOINT_COUNT) || (((int)x_wp == 0) && ((int)y_wp == 0))){	//this locks the car into this loop and makes it go slow when we've reached the max waypoints OR the waypoints are 0,0
		esc.writeMicroseconds(S2);
		while(true);
	}
	
	static long time = 0;
	if((millis() - time) > 500){
		//print_coordinates();
		time = millis();
	}
}