/* Minuteman / Roadrunner competition code

Pin Assignments:

A0 - Analog input from gyro
A1 - analog input for temp, from gyro
A2 - nc
A3 - LCD  (RS?)
A4 - LCD  (enable?)
A5 - LCD
D0 - RX
D1 - TX
D2 - nc (normally Ch1 in)
D3 - Steering input (connected to Ch2 in on board)
D4 - MUX enble input (input only. manual if low, auto if high)
D5 - Mode input (switched by Ch3)
D6 - LCD
D7 - LCD
D8 - LCD
D9 - nc (internally connected to MUX 1)  **consider connecting to MUX 3
D10 - Steering contorl out (internally connected to MUX 2
D11 - ESC control out (connect to MUX 3)
D12 - LED status
D13 - LED status
*/

#include <LiquidCrystal.h>
#include <Servo.h>
#include <EEPROM.h>
#include "AVC_2013.h"
#include "EEPROMAnything.h"
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
//#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050.h>
//#include "new_gyro.h"


//these are used for setting and clearing bits in special control registers on ATmega

//LiquidCrystal lcd(A3, A4, A5, 8, 7, 6);
volatile boolean gyro_flag = false, cal_flag = false;
boolean manual, automatic, aux=false, running=false, first=true;
volatile long gyro_sum = 0, gyro_count = 0, gyro_null=0, angle=0, clicks = 0;
long count, angle_last, angle_target, proximity, steer_us, angle_diff, previous_proximity=10000;
double x_wp[WAYPOINT_COUNT], y_wp[WAYPOINT_COUNT];
double x=0, y=0;
int wpr_count=1, wpw_count=1, speed_cur=0, speed_new=0, speed_old=0, steer_limm = 300;
const int InterruptPin = 2 ;		//intterupt on digital pin 2
Servo steering, esc;
long time=0;

MPU6050 accelgyro;

struct position_structure {

/* Using structures to contain location information. Will record old position 
and new position. The actual structures will be position[0] and position[1], 
but will use pointers old_pos and new_pos to access them. This way we can simply
swap the pointers instead of copying entire structure from one to the other. Access
data in the structures as follows: old_pos->x or new_pos->time, etc. this is equivalent
to (*old_pos).x.*/

    double x;
    double y;
    //boolean last;
} waypoint;

void encoder_interrupt() {
    clicks++;
}

void navigate() {
	calculate_speed();
	cal_steer_lim();
	update_position();
//	print_coordinates();
	update_steering();
	update_waypoint();
	get_mode();
	if (automatic) steering.writeMicroseconds(steer_us);
	if (automatic) speed();
}

void calculate_speed() {
    speed_new = micros();
    speed_cur = speed_new - speed_old;
    speed_old = speed_new;
}

void cal_steer_lim() {
	steer_limm = (int)map(speed_cur, L1, L2, L3, L4);
	if (steer_limm > L4) steer_limm = L4;
	// Serial.println(gyro_limm);
	}

void update_position() {
	//calculate position
	x += sin((angle + angle_last) * 3.14159/GYRO_CAL);
	y += cos((angle + angle_last) * 3.14159/GYRO_CAL);
	angle_last = angle;
	angle_target = atan2((x_wp[wpr_count] - x),(y_wp[wpr_count] - y)) * GYRO_CAL/2.0/3.14159;
	proximity = abs(x_wp[wpr_count]-x) + abs(y_wp[wpr_count]-y);
}

void update_steering() {
	//calculate and write angles for steering
	angle_diff = angle_target - angle;
	if (angle_diff < -GYRO_CAL/2) angle_diff += GYRO_CAL;   //if angle is less than 180 deg, then add 360 deg
	if (angle_diff > GYRO_CAL/2) angle_diff -= GYRO_CAL;	//if angle is greater than 180 deg, then subtract 360
	//now, we have an angle as -180 < angle_diff < 180. 
	steer_us = (float)angle_diff/GYRO_CAL*STEER_GAIN;
	if (steer_us < 0-steer_limm) steer_us = 0-steer_limm;
	if (steer_us > steer_limm) steer_us = steer_limm;
	steer_us += STEER_ADJUST;  //adjusts steering so that it will go in a straight line
}

void update_waypoint() {
	//waypoint acceptance and move to next waypoint
	if (proximity < WAYPOINT_ACCEPT) {
		wpr_count++;
		Serial.println("read WP # ");
		Serial.println(wpr_count);
		Serial.println(x_wp[wpr_count]);
		Serial.println(" , ");
		Serial.println(y_wp[wpr_count]);
		proximity = abs(x_wp[wpr_count]-x) + abs(y_wp[wpr_count]-y);
		previous_proximity = proximity;
	}
}

void print_coordinates() {
	//print stuff to LCD
	if((millis()-time)>1000) {
		Serial.println(x_wp[wpr_count]);
		Serial.println(" , ");
		Serial.println(y_wp[wpr_count]);
		Serial.println(x);
		Serial.println(" , ");
		Serial.println(y);
		time = millis();
	}
}

void speed() {
	running = true;			// make sure running is updated.

	if((previous_proximity - proximity) <= P1) esc.writeMicroseconds(S2); //allow car to line up with the next point
	else if(proximity < P2) esc.writeMicroseconds(S2); //ensure that a waypoint can be accepted
	else if(proximity >= P2 && proximity < P3) { //slow way down  50-200 works well, 50-300 is more conservative for higher speeds
		if (speed_cur < BREAKING_SPEED)  esc.writeMicroseconds(SB);  // less than 8000 means high speed, apply brakes
		else esc.writeMicroseconds(S3);  //once speed is low enough, resume normal slow-down
	}
	else if(proximity >= P3) esc.writeMicroseconds(S4); //go wide open 200 works well for me. 
}


void calculate_null() {
	Serial.println("CALCULATING NULL");

	cal_flag = true;		//tell ADC ISR that we are calibrating,
	angle = 0;				//reset the angle. angle will act as accumulator for null calculation
	gyro_null = 0;			//make sure to not subract any nulls here
	gyro_count = 0;

	while (gyro_count < 20000){
		read_FIFO();
		//delay(10);
		//Serial.println(gyro_count);
	}
	
	gyro_null = angle/gyro_count;	//calculate the null
	cal_flag = false;		//stop calibration
	angle = 0;
	

	//should print null here
	Serial.print("Null: ");
	Serial.println(gyro_null);
}

long get_temp() {
}

void stab_temp() {
}

void watch_angle() {
	Serial.println("angle watch");
	do {
		get_mode();
		read_FIFO();
		Serial.println(angle*360.0/GYRO_CAL);
		delay(30);
	} while (manual);		//keep summing unitil we turn the mode switch off.
}

void get_mode() {
    if (!digitalRead(TMISO)) {
		manual = true;
		automatic = false;
		aux = false;
    }
    else if (!digitalRead(MODE)) {
		manual = false;
		automatic = false;
		aux = true;
    }
    else {
		manual = false;
		automatic = true;
		aux = false;
    }
}

void set_waypoint() {
	waypoint.x = x;
	waypoint.y = y;
	//waypoint.last = false
	EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
	Serial.println("set WP # ");
	Serial.println(wpw_count);
	Serial.println(waypoint.x);
	Serial.println(" , ");
	Serial.println(waypoint.y);
	wpw_count++;
	while(aux) get_mode();
}    

void load_waypoints() {
	int temp = 1;
	Serial.println("LOADING POINTS");
	delay(1500);

	while (temp <= WAYPOINT_COUNT) {
		EEPROM_readAnything(temp*WP_SIZE, waypoint);
		x_wp[temp] = waypoint.x;
		y_wp[temp] = waypoint.y;
		temp++;
	}

	Serial.println("ALL POINTS");
	Serial.println("LOADED");
	delay(1500);
}

void read_waypoint() {
	long temp = micros();
	EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
	//x_wp = waypoint.x;
	//y_wp = waypoint.y;
	//waypoint.last = false
	//EEPROM_writeAnything(wp_count*WP_SIZE, waypoint);
	Serial.println("read WP # ");
	Serial.println(wpr_count);
	Serial.println(waypoint.x);
	Serial.println(" , ");
	Serial.println(waypoint.y);
	wpr_count++;
	Serial.println(micros() - temp);
}    

void eeprom_clear() {  //EEPROM Clear
	// write a 0 to all 512 bytes of the EEPROM
	for (int i = 0; i < 512; i++) EEPROM.write(i, 0);

	// turn the LED on when we're done
	Serial.println("EEPROM clear");
	delay(1500);
}

void import_waypoints() {
	eeprom_clear();

	int i=0, j=WAYPOINT_COUNT;
	WAYPOINTS_STRING    //edit this in header file to change waypoints
	
	while(i<j) {
		waypoint.x = excel_waypoints[i][0];
		waypoint.y = excel_waypoints[i][1];
		EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
		i++;
		wpw_count++;
	}

	Serial.println("ALL POINTS");
	Serial.println("IMPORTED");
	delay(1500);
}

void export_waypoints() {
	Serial.begin(115200);
	load_waypoints();
	
	for(int i=0; i<19; i++) {
		EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
		Serial.print("waypoint #");
		Serial.print(wpr_count);
		Serial.print(":\t");
		Serial.print(waypoint.x);
		Serial.print("\t");
		Serial.println(waypoint.y);
		wpr_count++;
	}

	Serial.println("ALL POINTS");
	Serial.println("EXPORTED");
	delay(1500);
	Serial.end();
}

void root_menu(){
    char* value_list[]={"ROOT MENU", "CONTINUE", "TEMPERATURE STAB", "IMPORT WP", "EXPORT WP", "WATCH ANGLE"};
    byte selection;
    selection = menu_list(5, value_list);   // try using sizeof(value_list) here
    switch (selection) {
	case 1:
		break;
    case 2:
		stab_temp();
       break;
    case 3:
		import_waypoints();
       break;
    case 4:
		export_waypoints();
       break;
    case 5:
		watch_angle();
       break;
    default:
       return;
    }
}

byte menu_list(byte items, char* value_list[]) {
    int pulse_length, adjust = 1, temp = 0;
    while (true) {
       pulse_length = pulseIn(3,HIGH);
       if (pulse_length - STEER_ADJUST > 50) adjust++;
       else if (pulse_length - STEER_ADJUST < -50) adjust--;
       if (adjust > items) adjust -= items;
       if (adjust < 1) adjust += items;
       get_mode();
       if (aux) {
           temp = millis();
           while (aux) get_mode();
           temp -= millis();
           if (temp < 500) return adjust;
           else return 0;
       }
       delay(500);
       //delay(map(abs(pulse_length-STEER_ADJUST), 0, 300, 700, 100));
       Serial.println(value_list[0]);
       Serial.println(value_list[adjust]);

       //Serial.println(adjust);
    }    
}

void setup_mpu6050() {
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	
    // reset device
    Serial.println(F("\n\nResetting MPU6050..."));
    accelgyro.reset();
    delay(30); // wait after reset


    // disable sleep mode
    Serial.println(F("Disabling sleep mode..."));
    accelgyro.setSleepEnabled(false);

    // get X/Y/Z gyro offsets
    Serial.println(F("Reading gyro offset values..."));
    int8_t xgOffset = accelgyro.getXGyroOffset();
    int8_t ygOffset = accelgyro.getYGyroOffset();
    int8_t zgOffset = accelgyro.getZGyroOffset();
    Serial.print(F("X gyro offset = "));
    Serial.println(xgOffset);
    Serial.print(F("Y gyro offset = "));
    Serial.println(ygOffset);
    Serial.print(F("Z gyro offset = "));
    Serial.println(zgOffset);

            Serial.println(F("Setting clock source to Z Gyro..."));
            accelgyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

            // Serial.println(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
            // accelgyro.setIntEnabled(0x12);

             Serial.println(F("Setting sample rate to 200Hz..."));
             accelgyro.setRate(0); // 1khz / (1 + 4) = 200 Hz

            // Serial.println(F("Setting external frame sync to TEMP_OUT_L[0]..."));
            // accelgyro.setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

            Serial.println(F("Setting DLPF bandwidth to 42Hz..."));
            accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);

            Serial.println(F("Setting gyro sensitivity to +/- 250 deg/sec..."));
            accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

            // Serial.println(F("Setting X/Y/Z gyro offsets to previous values..."));
            // accelgyro.setXGyroOffset(xgOffset);
            // accelgyro.setYGyroOffset(ygOffset);
            // accelgyro.setZGyroOffset(61);

            // Serial.println(F("Setting X/Y/Z gyro user offsets to zero..."));
            // accelgyro.setXGyroOffsetUser(0);
            // accelgyro.setYGyroOffsetUser(0);
            //accelgyro.setZGyroOffsetUser(0);
    //Serial.print(F("Z gyro offset = "));
    //Serial.println(accelgyro.getZGyroOffset());

            // Serial.println(F("Setting motion detection threshold to 2..."));
            // accelgyro.setMotionDetectionThreshold(2);

            // Serial.println(F("Setting zero-motion detection threshold to 156..."));
            // accelgyro.setZeroMotionDetectionThreshold(156);

            // Serial.println(F("Setting motion detection duration to 80..."));
            // accelgyro.setMotionDetectionDuration(80);

            // Serial.println(F("Setting zero-motion detection duration to 0..."));
            // accelgyro.setZeroMotionDetectionDuration(0);

            Serial.println(F("Resetting FIFO..."));
            accelgyro.resetFIFO();

            Serial.println(F("Enabling FIFO..."));
            accelgyro.setFIFOEnabled(true);
	        accelgyro.setZGyroFIFOEnabled(true);
	
}

void read_FIFO() {
  uint8_t buffer[2];
  int accumulator = 0;
  int samplz = 0;

  samplz = accelgyro.getFIFOCount() >> 1;
  //Serial.println("FIFO_COUNTH : ");
  //Serial.println(samplz,DEC);
  for (int i=0; i < samplz; i++) {
		accelgyro.getFIFOBytes(buffer, 2);
		angle -= ((((int16_t)buffer[0]) << 8) | buffer[1]) + gyro_null;
		gyro_count++;
		
		if ((angle > GYRO_CAL) && (!cal_flag)) angle -= GYRO_CAL; //if we are calculating null, don't roll-over
		if ((angle < 0) && (!cal_flag)) angle += GYRO_CAL;
	}
}

void watch_gyro() {
	while(true) {
		read_FIFO();
		//Serial.println(angle/130797);
//	Serial.println(angle);
	//Serial.println((float)angle/130797.0);
    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState);
		delay(4);
	}
}

void setup() {
	Serial.begin(115200);
	Wire.begin();
	//Pin assignments:
	pinMode(TMISO, INPUT);
	pinMode(MODE, INPUT);
	Serial.println(CAR_NAME);	//Print a message to the LCD.
	delay(10);

	pinMode(InterruptPin, INPUT);	 
	attachInterrupt(0, encoder_interrupt, CHANGE);	//interrupt 0 is on digital pin 2

	get_mode();
	//root_menu();
	load_waypoints();

	//verify that car is in manual mode prior to starting null calculation
	get_mode();
	if(manual == false) {
		Serial.println("SET CAR TO");
		Serial.println("MANUAL MODE!");
	}
	while(manual == false) get_mode();
	delay(500);

	setup_mpu6050();
	calculate_null();

	steering.attach(10);
	steering.writeMicroseconds(STEER_ADJUST);

	esc.attach(11);
	esc.writeMicroseconds(S1);

	Serial.println("**READY TO RUN**");
	wpr_count = 1;		//set waypoint read counter to first waypoint
	x=0;
	y=0;
	angle=0;
	clicks = 0;
	first = true;
}

void loop() {
	long temp;
	//watch_angle();
	read_FIFO();
	//watch_gyro();
	/* in the main loop here, we should wait for thing to happen, then act on them. Watch clicks and wait for it to reach CLICK_MAX, then calculate position and such.*/
	get_mode();
	if (clicks >= CLICK_MAX) {
		clicks = 0;
		navigate();
	}

	if (aux && DEBUG == 2) watch_angle();
	
	if (automatic) {	//this function makes the car be stationary when in manual waypoint setting mode
		if (!running) {
			esc.write(S2);	//i changed this to S1 so the car is stationary?
			running = true;
		}
		if (first) {
			angle = 0;
			first = false;
		}
	}
	
	if (manual) {	//this function makes the car be stationary when in manual waypoint setting mode
		if (running) {
			esc.write(S1);	//i changed this to S1 so the car is stationary?
			running = false;
		}
	}

	if (wpr_count >= WAYPOINT_COUNT) {
		esc.writeMicroseconds(S1);
		while (true);
	}
	
	if (aux && DEBUG == 0) {
		temp = millis();
		while (aux) get_mode();
		temp = millis() - temp;
		if (temp > 500) set_waypoint();
		//if (temp > 5000) read_waypoint();
	}
	
	if (aux && DEBUG == 3) {
		temp = millis();
		while (aux) get_mode();
		read_waypoint();
	}
}
