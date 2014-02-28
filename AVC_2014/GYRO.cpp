//AVC_2014 multi-file branch, gyro code

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
long accum = 0; //required for main program to reset gyro accum (so that the angle can actually be reset)
double angle = 0;
static long gyro_count = 0, gyro_null = 0; //visible only to GYRO.cpp
static bool cal_flag = false;


//EXTERNAL VARIABLES
extern long time;
extern bool manual;


//OBJECT DECLARATIONS
MPU6050 accelgyro;


//PROGRAM FUNCTIONS
void setup_mpu6050(){
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	
    // reset device
    Serial.println(F("\nResetting MPU6050..."));
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
	
	return ;
}

void read_FIFO(){
	uint8_t buffer[2];
	long temp = 0;
	int samplz = 0;

	samplz = accelgyro.getFIFOCount() >> 1;
	//Serial.println("FIFO_COUNTH : ");
	//Serial.println(samplz,DEC);
	for(int i=0; i < samplz; i++){
		accelgyro.getFIFOBytes(buffer, 2);
		temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
		accum -= (temp * 10) + gyro_null;
		gyro_count++;
		
		if((accum > GYRO_CAL) && (!cal_flag)) accum -= GYRO_CAL*2; //if we are calculating null, don't roll-over
		if((accum < -GYRO_CAL) && (!cal_flag)) accum += GYRO_CAL*2;
	}

	angle = (float)accum/(float)GYRO_CAL * 3.14159;

	return ;
}

void calculate_null(){
	Serial.println("CALCULATING NULL");

	cal_flag = true;		//tell ADC ISR that we are calibrating,
	accum = 0;				//reset the angle. angle will act as accumulator for null calculation
	gyro_null = 0;			//make sure to not subract any nulls here
	gyro_count = 0;

	while(gyro_count < 5000){
		read_FIFO();
		//delay(10);
		//Serial.println(gyro_count);
	}

	gyro_null = accum/gyro_count + NULL_FF;	//calculate the null. the -30 is a fudge factor for 5000 pts.
	cal_flag = false;		//stop calibration
	accum = 0;
	

	//should print null here
	Serial.print("Null: ");
	Serial.println(gyro_null);
	
	return ;
}

void gyro_calibration(){
	Serial.println();
	setup_mpu6050();
	calculate_null();
	cal_flag = true;

	Serial.println("calibrate gyro");
	do{
		read_FIFO();
		
		if((millis()-time)> 250){
			Serial.println(accum);
			time = millis();
		}
		get_mode();
	} while(manual);
	
	cal_flag = false;
	Serial.println();
	return ;
}

void watch_angle(){
	Serial.println();
	setup_mpu6050();
	calculate_null();

	Serial.println("watch angle");
	do {
		read_FIFO();

		if((millis()-time)> 250){
			//Serial.println(angle);
			Serial.println(angle*180.0/3.14159,5);
			//Serial.println(accum);
			time = millis();
		}
		get_mode();
	} while(manual);		//keep summing unitil we turn the mode switch off.

	return ;
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
		get_mode();
	} while(manual);		//keep summing unitil we turn the mode switch off.

	return ;
}


