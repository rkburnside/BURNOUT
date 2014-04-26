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
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	// initialize device
	SERIAL_OUT.println("Initializing I2C devices...");
	accelgyro.initialize();

	// verify connection
	SERIAL_OUT.println("Testing device connections...");
	SERIAL_OUT.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	// use the code below to change accel/gyro offset values
	accelgyro.setXGyroOffset(XGYROOFFSET);  //85
	accelgyro.setYGyroOffset(YGYROOFFSET);  //-70
	accelgyro.setZGyroOffset(ZGYROOFFSET);  //-22
	SERIAL_OUT.print(accelgyro.getXAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getYAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getZAccelOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getXGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getYGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print(accelgyro.getZGyroOffset()); SERIAL_OUT.print("\t"); // 
	SERIAL_OUT.print("\n");
	
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

	SERIAL_OUT.println(F("Resetting FIFO..."));
	accelgyro.resetFIFO();

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

void calculate_null(){
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
		temp = accelgyro.getRotationZ();
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


	// This setup routine should/will ensure the i2c connection is working
	// Wire.begin();
	// test the connection to the I2C bus, sometimes it doesn't connect
	// keep trying to connect to I2C bus if we get an error
	// boolean error = true;
	// while (error){
		// SERIAL_OUT.println("begin transmission with gyro");
		// Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
		// SERIAL_OUT.println("ending transmission with gyro");
		// error = Wire.endTransmission();			// if error = 0, we are properly connected
		// SERIAL_OUT.println("transmission ended");
		// if(error){								// if we aren't properly connected, try connecting again and loop
			// SERIAL_OUT.println();
			// SERIAL_OUT.println("Not properly connected to I2C, trying again");
			// SERIAL_OUT.println();
			// Wire.begin();
			// TWBR = 24; // 400kHz I2C clock
		// }
	// }
	// SERIAL_OUT.println("Properly connected to I2C");
