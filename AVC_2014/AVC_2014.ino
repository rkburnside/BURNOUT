/* MINUTEMAN / ROADRUNNER COMPETITION CODE

Pin Assignments:

A0 - Analog input from gyro
A1 - analog input for temp, from gyro
A2 - NC
A3 - NC
A4 - SDA
A5 - SCL
D0 - RX
D1 - TX
D2 - NC (normally Ch1 in)
D3 - Steering input (connected to Ch2 in on board)
D4 - MUX enble input (input only. manual if low, auto if high)
D5 - Mode input (switched by Ch3)
D6 - Toggle
D7 - NC
D8 - NC
D9 - NC (internally connected to MUX 1)  **consider connecting to MUX 3
D10 - Steering contorl out (internally connected to MUX 2
D11 - ESC control out (connect to MUX 3)
D12 - LED status
D13 - LED status

FULLY AUTONOMOUS MODE - MOVE THROTTLE INPUT TO 3 AND THROTTLE OUTPUT TO 3
STEERING AUTONOMOUS MODE - MOVE THROLLER INPUT TO 4 AND THROTTLE OUTPUT TO 4

*/


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


//INTERNAL VARIABLES
//these are used for setting and clearing bits in special control registers on ATmega
bool manual, automatic, aux=false, running=false, first=true;
volatile byte clicks = 0;
long time=0;
const byte InterruptPin = 2 ;		//interrupt on digital pin 2


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
	//cal_steer_lim();
	update_position();
	//update_cross_product();
	update_steering();
	update_waypoint();
	get_mode();
	if(automatic) steering.writeMicroseconds(steer_us);
	if(automatic) speed();
	
	return ;
}

void get_mode(){
	if(!digitalRead(TMISO)){
		manual = true;
		automatic = false;
		aux = false;
    }
    else if(!digitalRead(MODE)){
		manual = false;
		automatic = false;
		aux = true;
    }
    else {
		manual = false;
		automatic = true;
		aux = false;
    }

	return ;	
}

void setup(){
	//Pin assignments:
	pinMode(TMISO, INPUT);
	pinMode(MODE, INPUT);
	pinMode(TOGGLE, INPUT);
	digitalWrite(TOGGLE, HIGH);
	pinMode(12, OUTPUT);
	digitalWrite(12, LOW); 

	Wire.begin();

	Serial.begin(115200);
	Serial.setTimeout(100000);
	Serial.println(CAR_NAME);
	Serial.println();
	
	get_mode();
	main_menu();
	delay(500);	

	if(digitalRead(TOGGLE)){
		Serial.println("FLIP SWITCH TO CONTINUE");
		Serial.println();
	}
	while(digitalRead(TOGGLE)) get_mode();		//waits until the switch is flipped to start the race
	delay(2000);

	setup_mpu6050();
	calculate_null();

	pinMode(InterruptPin, INPUT);	 
	attachInterrupt(0, encoder_interrupt, CHANGE);	//interrupt 0 is on digital pin 2

	steering.attach(10);
	steering.writeMicroseconds(STEER_ADJUST);
	esc.attach(11);
	esc.writeMicroseconds(S1);

	Serial.println();
	Serial.println();

	//verify that car is in automatic mode
	get_mode();
	if(!automatic){
		Serial.println("1. SET CAR TO AUTOMATIC MODE! or press AUX to exit");
		Serial.println();
	}
	while(!automatic && !aux){
		get_mode();		//waits until the switch is flipped to start the race
		read_FIFO();
	}
	for(int i=0; i<100; i++){
		delay(1);
		read_FIFO();
	}

	//by turning off the radio, the automatic mode is locked in
	Serial.println("2. TURN OFF THE RADIO!");
	Serial.println();
	for(int i=0; i<500; i++){
		delay(1);
		read_FIFO();
	}

	Serial.println("***READY TO RUN***");
	Serial.println("3. FLIP THE SWITCH TO START THE RACE!");
	Serial.println();
	digitalWrite(12, HIGH);

	while(!digitalRead(TOGGLE) && (automatic)){	//waits for the switch to be flipped
		get_mode();		//waits until the switch is flipped to start the race
		read_FIFO();
	}
	for(int i=0; i<100; i++){	//waits 1 second before starting
		delay(1);
		read_FIFO();
	}

	wpr_count = 1;		//set waypoint read counter to first waypoint
	EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
	x_wp = waypoint.x;
	y_wp = waypoint.y;

	x=0;
	y=0;
	accum=0;
	clicks = 0;
	first = true;
	target_x = x_wp;
	target_y = y_wp;
}

void loop(){
	long temp;
	//watch_angle();
	read_FIFO();
	//watch_gyro();
	/* in the main loop here, we should wait for thing to happen, then act on them. Watch clicks and wait for it to reach CLICK_MAX, then calculate position and such.*/
	get_mode();

	if(clicks >= CLICK_MAX){
		clicks = 0;
		navigate();
	}

	if(automatic){	//this function get the car started moving and then clicks will take over
		if(!running){
			esc.write(S2);	//i don't understand this function...help...i changed this to S1 so the car is stationary?
			running = true;
		}
		if(first){
			accum = 0;
			first = false;
		}
	}
	
	if(manual){	//this function makes the car be stationary when in manual waypoint setting mode
		if(running){
			esc.write(S1);	//i changed this to S1 so the car is stationary?
			running = false;
		}
	}

	if(wpr_count >= WAYPOINT_COUNT){	//this locks the car into this loop and makes it stationary WHEN the course is completed
		esc.writeMicroseconds(S1);
		while(true);
	}
	
	if(aux){
		temp = millis();
		while(aux){
			get_mode();
			read_FIFO();
		}

		temp = millis() - temp;
		if(temp > 500) set_waypoint();
		//if(temp > 5000) read_waypoint();
	}
	
	if((millis()-time)>500){
		print_coordinates();
		time = millis();
	}
}

