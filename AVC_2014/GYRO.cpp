//GYRO FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
long accum = 0; //required for main program to reset gyro accum (so that the angle can actually be reset)
double angle = 0;
static long gyro_count = 0, gyro_null = 0; //visible only to GYRO.cpp
static bool cal_flag = false, gyro_error = false;


//EXTERNAL VARIABLES
extern int mode;


//OBJECT DECLARATIONS
MPU6050 accelgyro;

//PROGRAM FUNCTIONS
void setup_mpu6050(){
	clear_i2c();
	Wire.begin();
	SERIAL_OUT.println("Initializing gyro...");
	accelgyro.initialize();
	//accelgyro.reset();
    accelgyro.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

	// verify connection
	SERIAL_OUT.println("Testing device connections...");
	SERIAL_OUT.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	SERIAL_OUT.println(F("Setting clock source to Z Gyro..."));
	accelgyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
	//SERIAL_OUT.println(accelgyro.getClockSource(MPU6050_CLOCK_PLL_ZGYRO);

	SERIAL_OUT.println(F("Setting sample rate to 200Hz..."));
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

	SERIAL_OUT.println(F("Setting DLPF bandwidth"));
	accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);

	SERIAL_OUT.println(F("Setting gyro sensitivity to +/- 250 deg/sec..."));
	accelgyro.setFullScaleGyroRange(0);
	//accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	//accelgyro.setFullScaleGyroRange(0);  // 0=250, 1=500, 2=1000, 3=2000 deg/sec

	//SERIAL_OUT.println(F("Resetting FIFO..."));
	//accelgyro.resetFIFO();

	// use the code below to change accel/gyro offset values
	accelgyro.setXGyroOffset(XGYROOFFSET);
	accelgyro.setYGyroOffset(YGYROOFFSET);
	accelgyro.setZGyroOffset(ZGYROOFFSET);
	SERIAL_OUT.print(accelgyro.getXAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getYAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getZAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getXGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getYGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getZGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print("\n");
		
	SERIAL_OUT.println(F("Enabling FIFO..."));
	accelgyro.setFIFOEnabled(true);
	accelgyro.setZGyroFIFOEnabled(true);
	accelgyro.setXGyroFIFOEnabled(false);
	accelgyro.setYGyroFIFOEnabled(false);
	accelgyro.setAccelFIFOEnabled(false);
	SERIAL_OUT.print("Z axis enabled?\t"); SERIAL_OUT.println(accelgyro.getZGyroFIFOEnabled());
	SERIAL_OUT.print("x axis enabled?\t"); SERIAL_OUT.println(accelgyro.getXGyroFIFOEnabled());
	SERIAL_OUT.print("y axis enabled?\t"); SERIAL_OUT.println(accelgyro.getYGyroFIFOEnabled());
	SERIAL_OUT.print("accel enabled?\t"); SERIAL_OUT.println(accelgyro.getAccelFIFOEnabled());
	accelgyro.resetFIFO();
	return ;
}

void read_FIFO(){
	uint8_t buffer[2];
	int16_t temp = 0;
	int samplz = 0;
	samplz = accelgyro.getFIFOCount() / 2;
	//SERIAL_OUT.println("FIFO_COUNTH : ");
	//SERIAL_OUT.println(samplz,DEC);
	for(int i=0; i < samplz; i++){
		accelgyro.getFIFOBytes(buffer, 2);
		temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
		if (abs(temp) > 32765) gyro_error = true;
		accum += temp*10 - gyro_null;    
		//accum = temp;    
		gyro_count++;
		
		if((accum > GYRO_CAL) && (!cal_flag)) accum -= GYRO_CAL*2; //if we are calculating null, don't roll-over
		if((accum < -GYRO_CAL) && (!cal_flag)) accum += GYRO_CAL*2;
	}
	angle = (float)accum/(float)GYRO_CAL * -3.14159;   //change sign of PI for flipped gyro
	//angle = (float)accum/GYRO_CAL * -180;   //using degrees *10, negative for flipped gyro.

	return ;
}

void clear_i2c(){
	pinMode(18, INPUT);
	delayMicroseconds(100);
	if(digitalRead(18)) return;
	SERIAL_OUT.println("i2c appears to be hung. attempting to clear...");
	int i = 0;
	while(!digitalRead(18)||(i<1)){		//if SDA is low, then the bus is hung
		pinMode(19, INPUT);	// making this and input pulls the line high (1)
		delayMicroseconds(100);
		pinMode(19, OUTPUT);   // pulls line low (0)
		digitalWrite(19, LOW);
		delayMicroseconds(100);
		pinMode(19, INPUT);   // make line high again
		delayMicroseconds(100);
		i++;
	}
	SERIAL_OUT.println("OK, generating stop");
	pinMode(18, OUTPUT);
	digitalWrite(18, LOW);
	delayMicroseconds(100);
	pinMode(18, INPUT);
	delayMicroseconds(100);
	return;
}
	
void calculate_null(){
	bool retest = false;
	do{
		SERIAL_OUT.println("CALCULATING NULL");
		cal_flag = true;		//calibrating,
		accum = 0;				//reset the angle. angle will act as accumulator for null calculation
		gyro_null = 0;			//make sure to not subtract any nulls here
		gyro_count = 0;

		while(gyro_count < 5000){
			read_FIFO();
			//delay(10);
			//SERIAL_OUT.println(gyro_count);
		}
		gyro_null = accum/gyro_count -1;	//calculate the null. the -30 is a fudge factor for 5000 pts.
		cal_flag = false;		//stop calibration
		accum = 0;
		
		if(gyro_null > 150){
			SERIAL_OUT.print("Null was rather high: ");
			SERIAL_OUT.println(gyro_null);
			SERIAL_OUT.print("Set to Auto to recalculate or WP mode to bypass");
			while(1){
				get_mode();
				
				if(mode == WP_MODE){
					retest = false;
					break;
				}
				if(mode == AUTOMATIC){
					retest = true;
					break;
				}
			}
		
		}
		else retest = false;

	} while(retest);
	
	//should print null here
	SERIAL_OUT.print("Null: ");
	SERIAL_OUT.println(gyro_null);
	
	return ;
}

void gyro_calibration(){
	static long time = millis();
	SERIAL_OUT.println();
	setup_mpu6050();
	calculate_null();
	cal_flag = true;

	SERIAL_OUT.println("calibrate gyro");
	do{
		read_FIFO();
		
		if((millis() - time) > 250){
			SERIAL_OUT.println(accum);
			time = millis();
		}
		get_mode();
	} while(mode == MANUAL);
	
	cal_flag = false;
	SERIAL_OUT.println();
	return ;
}

void watch_angle(){
	static long time = millis();
	SERIAL_OUT.println();
	setup_mpu6050();
	calculate_null();

	SERIAL_OUT.println("watch angle");

	do{
		read_FIFO();

		if((millis() - time) > 250){
			SERIAL_OUT.println(angle*180.0/3.14159);	//angle;
			time = millis();
		}
		get_mode();
	} while(mode == MANUAL);		//keep summing until we turn the mode switch off.

	return ;
}

void watch_gyro(){
	static long time = millis();
	SERIAL_OUT.println();
	setup_mpu6050();
	calculate_null();

	SERIAL_OUT.println("watch gyro");
	do{
		read_FIFO();

		if((millis() - time) > 250){
			SERIAL_OUT.println(accum);
			time = millis();
		}
		get_mode();
	} while(mode == MANUAL);		//keep summing until we turn the mode switch off.

	return ;
}

void reset_FIFO(){
	accelgyro.resetFIFO();
	return;
}

void gyro_rate(){
	int16_t temp = 0, max=0, min = 0;
	static long time = millis();
	SERIAL_OUT.println();
	setup_mpu6050();

	SERIAL_OUT.println("min/max gyro rates:");
	do{
//		temp = accelgyro.getRotationZ();
		temp = accelgyro.getAccelerationZ();
		if (temp > max) max = temp;
		if (temp < min) min = temp;
		if((millis() - time) > 250){
			SERIAL_OUT.print(min);
			SERIAL_OUT.print('\t');
			SERIAL_OUT.println(max);
			time = millis();
		}
		delay(2);
		get_mode();
	} while(mode == MANUAL);		//keep summing until we turn the mode switch off.
	return ;
}	

int get_gyro_rate() {
	return accelgyro.getRotationZ();
}

int get_accel_rate() {
	return accelgyro.getAccelerationY();
}

void lateral_accel(){
	int16_t temp = 0, max=0, min = 0;
	static long time = millis();
	SERIAL_OUT.println();
	clear_i2c();
	Wire.begin();
	SERIAL_OUT.println("Initializing gyro...");
	accelgyro.initialize();
	//accelgyro.reset();
    accelgyro.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

	// verify connection
	SERIAL_OUT.println("Testing device connections...");
	SERIAL_OUT.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	SERIAL_OUT.println(F("Setting clock source to Z Gyro..."));
	accelgyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
	//SERIAL_OUT.println(accelgyro.getClockSource(MPU6050_CLOCK_PLL_ZGYRO);

	SERIAL_OUT.println(F("Setting sample rate to 200Hz..."));
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

	SERIAL_OUT.println(F("Setting DLPF bandwidth"));
	accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);

	SERIAL_OUT.println(F("Setting accel sensitivity..."));
	accelgyro.setFullScaleAccelRange(3);   // 0=2g, 1=4g, 2=8g, 3=16g deg/sec

	// use the code below to change accel/gyro offset values
	accelgyro.setXAccelOffset(-5097);
	accelgyro.setYAccelOffset(-37);
	accelgyro.setZAccelOffset(1125);
	SERIAL_OUT.print(accelgyro.getXAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getYAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getZAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getXGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getYGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getZGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print("\n");
		
	SERIAL_OUT.println("min/max gyro rates:");
	do{
//		temp = accelgyro.getRotationZ();
		temp = accelgyro.getAccelerationY();
		if (temp > max) max = temp;
		if (temp < min) min = temp;
		if((millis() - time) > 250){
			SERIAL_OUT.print(min);
			SERIAL_OUT.print('\t');
			SERIAL_OUT.println(max);
			time = millis();
		}
		delay(2);
		get_mode();
	} while(mode == MANUAL);		//keep summing until we turn the mode switch off.
	return ;
}	
