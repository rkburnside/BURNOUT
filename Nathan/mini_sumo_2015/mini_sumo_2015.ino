#include <EEPROM.h>
#include <Arduino.h>  // for type definitions
#include "EEPROMAnything.h"
#include <Servo.h>
#include <IRremote.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

//#define GYRO_CAL 235434200	//this has to be measured by rotating the gyro 180 deg. and reading the output
#define GYRO_CAL 58408577	// for 1000 deg/sec this has to be measured by rotating the gyro 180 deg. and reading the output
#define Kp 3.424e-5
#define Ki 3.424e-9

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
#define DELAY_TIME 5

//strategy definitions
#define GOTO_ANGLE 1
#define LOOP_RIGHT 2
#define LOOP_LEFT 3
#define CHARGE_AHEAD 4 
#define SEARCH_RIGHT 5
#define SEARCH_LEFT 6

//IR key definitions
#define SELECT_BOT_1 0xE2
#define SELECT_BOT_2 0xD2
#define SELECT_BOT_3 0xF2
#define NUMBER_1 0xE0
#define NUMBER_2 0xD0
#define NUMBER_3 0xF0
#define NUMBER_4 0xC8
#define NUMBER_5 0xE8
#define NUMBER_6 0xD8
#define NUMBER_7 0xF8
#define NUMBER_8 0xC4
#define NUMBER_9 0xE4
#define NUMBER_0 0xD4
#define TEN_PLUS 0xF4
#define KARAOKE 0xCA
#define PLAY_KEY 0xC0
#define FUNCTION_KEY 0xDE
#define VOL_DOWN 0xDC
#define VOL_UP 0xEC
#define POWER_KEY 0xC0

//Initialize variables
unsigned int sensorValues[NUM_SENSORS];
boolean last_turn;
long timeout = 0, ir_counter = 0;
byte state, state_cur, state_pre;

//gyro variables
boolean gyro_flag = false, cal_flag = false, long_flag = false;
long gyro_count = 0, gyro_null=0, accum=0, time=0, angle_target = 0;
int angle_diff, angle_last, angle_camera, angle=0;
double angle_err, angle_errSum; 
byte result;

//Initialize objects
Servo escR;
Servo escL;
IRrecv irrecv(IR_PIN);
//IRrecv irrecv(9);
MPU6050 accelgyro;

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
				ESCL_percent(99);
				ESCR_percent(0);
			}
			else {
				ESCR_percent(99);
				ESCL_percent(0);
			}
		}
	}
}
		
void set_motors(){	
	switch (state) {
		case FR_BIT:  // right sensor
			//reset counter
			ESCL_percent(99);
			ESCR_percent(85);
			last_turn = RIGHT_TURN;
			break;
		case FL_BIT:  //left sensor
			ESCR_percent(99);
			ESCL_percent(85);
			last_turn = LEFT_TURN;
			break;
		case FR_BIT+FL_BIT:  //both sensors
			ESCL_percent(99);
			ESCR_percent(99);
			break;
		default:
			ESCL_percent(0);
			ESCR_percent(0);
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

void IR_menu(){
	while (true) {
		if (irrecv.decode(&results)) {
			if (results.value/256 == 0x76044f) {
				byte ir_command = results.value;
				Serial.println(ir_command, HEX);
				switch (ir_command) {
					case TEN_PLUS:
						IR_set_angle();
						break;
					case KARAOKE:
						Serial.println("karaoke");
						break;
					case PLAY_KEY:
						return;
						break;
				}
			}
			irrecv.resume(); // Receive the next value
		}
		delay(100);
	}
}

void IR_set_angle(){
	double angle_temp = 0.0;
	angle_temp = (float)IR_get_number() * 10.0;
	//delay(300);
	angle_temp += (float)IR_get_number();
	Serial.println(angle_temp);
	//delay(300);
	angle_temp -= 50.0;
	angle_temp = angle_temp/50.0;
	Serial.println(angle_temp);
	angle_temp *= (float)GYRO_CAL;
	angle_target = angle_temp;
	Serial.println(angle_target);
	accum = (long)angle_target;
	return;
}

byte IR_get_number(){
	int value = -1;
	while (true) {
		if (irrecv.decode(&results)) {
			if (results.value/256 == 0x76044f) {
				byte ir_command = results.value;
				//Serial.println(ir_command, HEX);
				switch (ir_command) {
					case NUMBER_1:
						irrecv.resume();
						value = 1;
						break;
					case NUMBER_2:
						value = 2;
						break;
					case NUMBER_3:
						value = 3;
						break;
					case NUMBER_4:
						value = 4;
						break;
					case NUMBER_5:
						value = 5;
						break;
					case NUMBER_6:
						value = 6;
						break;
					case NUMBER_7:
						value = 7;
						break;
					case NUMBER_8:
						value = 8;
						break;
					case NUMBER_9:
						value = 9;
						break;
					case NUMBER_0:
						value = 0;
						break;
				}
				if (value > -1) {
					irrecv.resume();
					delay(100);
					return value;
				}
			}
			irrecv.resume(); // Receive the next value
		}
		delay(100);
	}

}

void ready_to_start(){
	while (true) {
		if (irrecv.decode(&results)) {
			if (results.value/256 == 0x76044f) {
				byte ir_command = results.value;
				Serial.println(ir_command, HEX);
				switch (ir_command) {
					case SELECT_BOT_1:
						IR_menu();
						return;
						break;
					case SELECT_BOT_2:
						break;
					case POWER_KEY:
						return;
						break;
				}
			}
			irrecv.resume(); // Receive the next value
		}	
	delay(100);
	}
}

void PID_angle(){
	angle_err = (float)accum;
	angle_errSum += accum;
	float temp = angle_err * Kp + (float)angle_errSum * Ki;
}

void goto_angle(){
	if (accum > 0) {
		ESCL_percent(100);
		float temp;
		temp = 100.0 - (float)accum/20e4;
		if (temp > 100) temp = 100;
		if (temp < 0) temp = 0;
		ESCR_percent(temp);
		//Serial.println(temp);
		//Serial.println(accum);
	}
	else {
		ESCR_percent(100);
		float temp;
		temp = 100.0 + (float)accum/20e4;
		if (temp > 100) temp = 100;
		if (temp < 0) temp = 0;
		ESCL_percent(temp);		
		//Serial.println(temp);
		//Serial.println(accum);
	}
}

void ESCR_percent(int value){
	if (value > 0) value = map(value, 0, 100, 1680, 1970);		
	else if (value < 0) value = map(value, -100, 0, 1230, 1370);
	else value = 1480;
	escR.writeMicroseconds(value);
}

void ESCL_percent(int value){
	if (value > 0) value = map(value, 0, 100, 1650, 1950);		
	else if (value < 0) value = map(value, -100, 0, 1220, 1360);
	else value = 1470;
	escL.writeMicroseconds(value);
}

void PID_turn_rate(){

}

//*********GYRO STUFF**************************
void watch_angle(){
	Serial.println("watch angle");
	while(true) {
		read_FIFO();
		Serial.println(angle);  //watching in radians
		delay(30);
	}
}

void watch_gyro(){
	Serial.println();
	setup_mpu6050();
	calculate_null();

	Serial.println("watch gyro");
	do {
		read_FIFO();

		if((millis()-time)> 250){
			Serial.println(accum);
			time = millis();
		}
	} while(true);		//keep summing until we turn the mode switch off.

	return ;
}

void calculate_null(){
	Serial.println("CALCULATING NULL");
	cal_flag = true;		//calibrating,
	accum = 0;				//reset the angle. angle will act as accumulator for null calculation
	gyro_null = 0;			//make sure to not subtract any nulls here
	gyro_count = 0;

	while(gyro_count < 500)	read_FIFO();

	gyro_null = accum/gyro_count - 1;	//calculate the null. the -30 is a fudge factor for 5000 pts.
	cal_flag = false;		//stop calibration
	accum = 0;
	
	//should print null here
	Serial.println("Null: ");
	Serial.println(gyro_null);
	
	return ;
}

void read_FIFO(){
	//Serial.println("FIFO");
	uint8_t buffer[2];
	long temp = 0;
	int samplz = 0;
	samplz = accelgyro.getFIFOCount() >> 1;
	//Serial.println(samplz,DEC);
	for(int i=0; i < samplz; i++){
		accelgyro.getFIFOBytes(buffer, 2);
		temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
		accum += temp*10 - gyro_null;    
		//accum += temp;  // - gyro_null;    
		gyro_count++;
		
		if((accum > GYRO_CAL) && (!cal_flag)) accum -= GYRO_CAL*2; //if we are calculating null, don't roll-over
		if((accum < -GYRO_CAL) && (!cal_flag)) accum += GYRO_CAL*2;
	}
	//angle = (float)accum/(float)GYRO_CAL * -3.14159;   //change sign of PI for flipped gyro
	//accum = 0 - accum;
	angle = (float)accum/(float)GYRO_CAL * -180;   //using degrees *10, negative for flipped gyro.

	return ;
}

void setup_mpu6050(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    accelgyro.setXGyroOffset(7);  //85
    accelgyro.setYGyroOffset(60);  //-70
    accelgyro.setZGyroOffset(44);  //-22
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // 
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // 
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 
    Serial.print("\n");
    
	Serial.println(F("Setting clock source to Z Gyro..."));
	accelgyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
	//Serial.println(accelgyro.getClockSource(MPU6050_CLOCK_PLL_ZGYRO);

	Serial.println(F("Setting sample rate to 200Hz..."));
	accelgyro.setRate(0); // 1khz / (1 + 4) = 200 Hz

 // *          |   ACCELEROMETER    |           GYROSCOPE
 // * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 // * ---------+-----------+--------+-----------+--------+-------------
 // * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 // * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 // * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 // * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 // * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 // * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 // * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 // * 7        |   -- Reserved --   |   -- Reserved --   | Reserved

	Serial.println(F("Setting DLPF bandwidth"));
	accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);

	Serial.println(F("Setting gyro sensitivity to +/- 250 deg/sec..."));
	accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	//accelgyro.setFullScaleGyroRange(0);  // 0=250, 1=500, 2=1000, 3=2000 deg/sec

	Serial.println(F("Resetting FIFO..."));
	accelgyro.resetFIFO();

	Serial.println(F("Enabling FIFO..."));
	accelgyro.setFIFOEnabled(true);
	accelgyro.setZGyroFIFOEnabled(true);
	accelgyro.setXGyroFIFOEnabled(false);
	accelgyro.setYGyroFIFOEnabled(false);
	accelgyro.setAccelFIFOEnabled(false);
	Serial.print("Z axis enabled?\t"); Serial.println(accelgyro.getZGyroFIFOEnabled());
	Serial.print("x axis enabled?\t"); Serial.println(accelgyro.getXGyroFIFOEnabled());
	Serial.print("y axis enabled?\t"); Serial.println(accelgyro.getYGyroFIFOEnabled());
	Serial.print("accel enabled?\t"); Serial.println(accelgyro.getAccelFIFOEnabled());
	accelgyro.resetFIFO();
	return ;
}

//************** SETUP ***********************

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
	ESCL_percent(0);
	ESCR_percent(0);

	Wire.begin();
	Serial.begin(115200);
	setup_mpu6050();
	pinMode(9, INPUT_PULLUP);
	pinMode(10, OUTPUT);
	//while(digitalRead(9));
	calculate_null();
	//delay(3200);
	accelgyro.resetFIFO();

	irrecv.enableIRIn(); // Start the receiver
	ready_to_start();
//	pinMode(9, INPUT_PULLUP);
	//delay(1000);
	//start_IR();
	ir_counter = 0;
	TIMSK2 = 0;  //should disable ir stuff.
//	randomSeed(analogRead(0));
//	if(random(1000) < 500) last_turn = RIGHT_TURN;
//	else last_turn = LEFT_TURN;
	last_turn = LEFT_TURN;
	//accum = -14600000;
}

void loop(){
	state_cur = read_sensors();
	if ((millis() - time) > 0) {
		time = millis();
		read_FIFO();
		decide();
		//goto_angle();
	}
	if (digitalRead(IR_PIN) < 1) ir_counter++;
	else ir_counter = 0;
	if (ir_counter > 20) {
		ESCL_percent(0);
		ESCR_percent(0);
		digitalWrite(13, HIGH);
		while (true);
	}
	//state = read_sensors();
	//set_motors();
	//Serial.println(accum);
	//watch_line();
}