#include <EEPROM.h>
#include <Arduino.h>  // for type definitions
#include "EEPROMAnything.h"
#include <Servo.h>
#include <IRremote.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#define ESC_NULL_R		1500
#define ESC_NULL_L		1500

//speed definitions
#define FAST 500    //500
#define MEDIUM 250     //250
#define SLOW 80		//80

//Pin definitions
#define FL_PIN A1
#define FR_PIN A2
#define RIGHT_PIN A3
#define LEFT_PIN A0
//#define FR_LINE_PIN 3
//#define FL_LINE_PIN 4
//#define RC_LINE_PIN 5
#define R_ESC_PIN 3
#define L_ESC_PIN 2
#define IR_PIN 4


//Sensor bit defines
#define NUM_SENSORS 0
#define FL_BIT 1
#define FR_BIT 2
#define RIGHT_BIT 4
#define LEFT_BIT 8

//Line sensor defines
#define FR_LINE 1
#define FL_LINE 2
#define RC_LINE 3

//sensor limits
#define SHARP_LIM_FAR	210
#define SHARP_LIM_CLOSE	350
#define WHITE_THRESHOLD	200

//misc definitions
#define RIGHT_TURN 1
#define LEFT_TURN 0
#define DELAY_TIME 500

//Initialize variables
unsigned int sensorValues[NUM_SENSORS];
boolean last_turn;
long timeout = 0, ir_counter = 0;
byte state, state_cur, state_pre;

//Initialize objects
Servo escR;
Servo escL;
IRrecv irrecv(IR_PIN);

decode_results results;  // declaration of structure (?) to hold ir results

struct gyro_settings_structure {

/* Using structures to contain location information. Will record old position 
and new position. The actual structures will be position[0] and position[1], 
but will use pointers old_pos and new_pos to access them. This way we can simply
swap the pointers instead of copying entire structure from one to the other. Access
data in the structures as follows: old_pos->x or new_pos->time, etc. this is equivalent
to (*old_pos).x.*/

    long gyro_cal;
	int x_gyro_offset;
	int y_gyro_offset;
	int z_gyro_offset;
    int r_ESC_mid_high;
    int r_ESC_high;
    int r_ESC_mid_low;
    int r_ESC_low;
    int r_ESC_mid;
    int l_ESC_mid_high;
    int l_ESC_high;
    int l_ESC_mid_low;
    int l_ESC_low;
    int l_ESC_mid;
 } settings;

byte read_sensors(){
	//Serial.println("rfront");
	byte flags = 0;
	if (digitalRead(FL_PIN)) flags += FL_BIT;
	if (digitalRead(FR_PIN)) flags += FR_BIT;
	return flags;
}

byte read_side(){
	//Serial.println("rside");
	byte flags = 0;
	return flags;
}

void decide_front(){
}

void decide_side(){
}

void decide(){
	//if (true) {//if current state =! 0, save state as previous,
	if (state_cur > 0) {//if current state =! 0, save state as previous,
		state = state_cur;
		state_pre = state_cur;
		timeout = 0;
		set_motors();
	}
	
	else {
		timeout++;
		if (timeout < DELAY_TIME) {
		//if (true) {
			state = state_pre;
			set_motors();
		}
		// else {
			// escR.writeMicroseconds(ESC_NULL_R);
			// escL.writeMicroseconds(ESC_NULL_L);
		// }
		
		else {
			if (last_turn == RIGHT_TURN) {
				escL.writeMicroseconds(FAST + ESC_NULL_L);
				escR.writeMicroseconds(-SLOW + ESC_NULL_R);
			}
			else {
				escR.writeMicroseconds(FAST + ESC_NULL_R);
				escL.writeMicroseconds(-SLOW + ESC_NULL_L);
			}
		}
	}
}
		
void set_motors(){	
	switch (state) {
		case FR_BIT:  // right sensor
			//reset counter
			//fast forward, right
			escL.writeMicroseconds(FAST + ESC_NULL_L);
			escR.writeMicroseconds(FAST - 40 + ESC_NULL_R);
			//escR.writeMicroseconds(MEDIUM + ESC_NULL_R);
			last_turn = RIGHT_TURN;
			break;
		case FL_BIT:  //left sensor
			escR.writeMicroseconds(FAST + ESC_NULL_R);
			escL.writeMicroseconds(FAST - 40 + ESC_NULL_L);
			//escL.writeMicroseconds(MEDIUM + ESC_NULL_L);
			last_turn = LEFT_TURN;
			break;
		case FR_BIT+FL_BIT:  //both sensors
			escR.writeMicroseconds(FAST + ESC_NULL_R);
			escL.writeMicroseconds(FAST + ESC_NULL_L);
			break;
		default:
			escR.writeMicroseconds(ESC_NULL_R);
			escL.writeMicroseconds(ESC_NULL_L);
		}
}

void watch_sensors(){
	Serial.print(analogRead(0));
	Serial.print("\t");
	Serial.print(analogRead(1));
	Serial.print("\t");
	Serial.print(analogRead(2));
	Serial.print("\t");
	Serial.println(analogRead(3));
	delay(300);
}

void start_IR(){
	digitalWrite(13, HIGH);
	ir_counter = 0;
	while (ir_counter < 1000){
		if (digitalRead(IR_PIN) < 1) ir_counter++;
		else ir_counter = 0;
	}
	digitalWrite(13, LOW);
	delay(5000);
}

void setup(){
	// set pins
	pinMode(R_ESC_PIN, OUTPUT);   // sets the pin as output
	pinMode(L_ESC_PIN, OUTPUT);   // sets the pin as output
	pinMode(FL_PIN, INPUT);   // sets the pin as input
	pinMode(FR_PIN, INPUT);   // sets the pin as input
	pinMode(RIGHT_PIN, INPUT);   // sets the pin as input
	pinMode(LEFT_PIN, INPUT);   // sets the pin as input
	pinMode(IR_PIN, INPUT);   // sets the pin as input
	
	//Initialize modules
	escR.attach(R_ESC_PIN);
	escL.attach(L_ESC_PIN);
	escR.writeMicroseconds(ESC_NULL_R);
	escL.writeMicroseconds(ESC_NULL_L);
	
	irrecv.enableIRIn(); // Start the receiver

	//while (true);
	pinMode(9, INPUT_PULLUP);
	//while(digitalRead(9));
	//calculate_null();
	delay(1000);
	start_IR();
	ir_counter = 0;
//	randomSeed(analogRead(0));
//	if(random(1000) < 500) last_turn = RIGHT_TURN;
//	else last_turn = LEFT_TURN;
	last_turn = LEFT_TURN;
	
}

void loop(){
	state_cur = read_sensors();
	decide();
	if (digitalRead(IR_PIN) < 1) ir_counter++;
	else ir_counter = 0;
	if (ir_counter > 200) {
		escR.writeMicroseconds(ESC_NULL_R);
		escL.writeMicroseconds(ESC_NULL_L);
		digitalWrite(13, HIGH);
		while (true);
	}
	//state = read_sensors();
	//set_motors();
	//Serial.println(accum);
	//watch_line();
}