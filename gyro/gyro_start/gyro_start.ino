#include <Wire.h>  // note, to make i2c run at 400kHz, change twi.h, line  #define TWI_FREQ 400000L
#include <I2Cdev.h>
#include <MPU6050.h>

#define NULL_FF -30
#define GYRO_CAL 235434205	//this has to be measured by rotating the gyro 360 deg. and reading the output
boolean gyro_flag = false, cal_flag = false;
long gyro_count = 0, gyro_null = 0, accum = 0, time=0;
byte result, state, flags, timeout=0;
float angle;
MPU6050 accelgyro;

void watch_angle(){
	Serial.println("watch angle");
	while(true) {
		read_FIFO();
		Serial.println(angle);  //watching in radians
		delay(30);
	}
}

void watch_fifo(){
	uint8_t buffer[2];
	int16_t temp = 0, temp2 = 0;
	accelgyro.resetFIFO();
	while(true){
		if (accelgyro.getFIFOCount() == 2){
		//if (true){
			//accelgyro.getFIFOBytes(buffer,2);
			buffer[0] = accelgyro.getFIFOByte();
			buffer[1] = accelgyro.getFIFOByte();
			temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
		}
		//accelgyro.getFIFOByte(buffer, 2);
		if((millis()-time)> 250){
			temp2 = micros();
			//Serial.println(accelgyro.getFIFOCount());
			accelgyro.getFIFOBytes(buffer,10);			
			temp2 = micros() - temp2;
			// Serial.print(buffer[0]);
			// Serial.print("  ");
			// Serial.println(buffer[1]);
			Serial.println(temp2);
			time = millis();
		}
	}
}

void watch_gyro(){
	Serial.println();
	//setup_mpu6050();
	//calculate_null();
	accum = 0;
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
	gyro_null = 0;			//make sure to not subract any nulls here
	gyro_count = 0;

	while(gyro_count < 5000){
		read_FIFO();
		//delay(10);
		//Serial.println(gyro_count);
	}
	gyro_null = accum/gyro_count -1;	//calculate the null. the -30 is a fudge factor for 5000 pts.
	cal_flag = false;		//stop calibration
	accum = 0;
	

	//should print null here
	Serial.print("Null: ");
	Serial.println(gyro_null);
	
	return ;
}

void read_FIFO(){
	uint8_t buffer[2];
	int16_t temp = 0;
	int samplz = 0;
	samplz = accelgyro.getFIFOCount() / 2;
	//Serial.println("FIFO_COUNTH : ");
	//Serial.println(samplz,DEC);
	for(int i=0; i < samplz; i++){
		accelgyro.getFIFOBytes(buffer, 2);
		temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
		accum += temp*10 - gyro_null;    
		//accum = temp;    
		gyro_count++;
		
		if((accum > GYRO_CAL) && (!cal_flag)) accum -= GYRO_CAL*2; //if we are calculating null, don't roll-over
		if((accum < -GYRO_CAL) && (!cal_flag)) accum += GYRO_CAL*2;
	}
	//angle = (float)accum/(float)GYRO_CAL * -3.14159;   //change sign of PI for flipped gyro
	angle = (float)accum/GYRO_CAL * -180;   //using degrees *10, negative for flipped gyro.

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
    accelgyro.setXGyroOffset(85);  //85
    accelgyro.setYGyroOffset(-70);  //-70
    accelgyro.setZGyroOffset(46);  //-22
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
	accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
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

void setup() {
	delay(3000);
	Wire.begin(); 
	Serial.begin(115200);
	setup_mpu6050();
	calculate_null();
}

void loop(){
	//watch_fifo();
	//watch_gyro();
	watch_angle();
	}
