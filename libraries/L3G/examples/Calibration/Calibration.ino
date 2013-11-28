//L3G4200D Calibration Routine

#include <Wire.h>
#include <L3G.h>
#define GYRO_SAMPLING_RATE 10 //sampling at 10ms
#define GYRO_CALIBRATION_NUMBER 3890318.0 //THIS NUMBER IS BASED ON SAMPLING TIME!!!
L3G gyro;
double gyro_sum = 0, gyro_null = 0, gyro_angle = 0, sample_time = 0, print_delay = 0;

void setup() {
	Serial.begin(115200);
	Wire.begin();

	if (!gyro.init()) {
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}

	gyro.enableDefault();

	Serial.println();
	Serial.println("gyro null being calculated");
	gyro_null_cal();
	Serial.println();
	Serial.println("Z, Sum, Angle");
}

void loop() {
	if((millis() - sample_time) > GYRO_SAMPLING_RATE){
		sample_time = millis();
		gyro.read();

		gyro_sum = gyro_sum - (gyro.g.z - gyro_null);
		gyro_angle = gyro_sum / GYRO_CALIBRATION_NUMBER * 360.0;
	}
	
	if((millis() - print_delay) > 100){
		print_delay = millis();
		Serial.print((int)gyro.g.z);
		Serial.print("\t\t");
		Serial.print(gyro_sum);
		Serial.print("\t\t");
		Serial.println(gyro_angle);
	}
}

void gyro_null_cal() {
	for(int i = 0; i<30; i++) {
		gyro.read();
		delay(GYRO_SAMPLING_RATE);
	}
		
	for(int i = 0; i<500; i++) {
		gyro.read();
		gyro_null = gyro_null + gyro.g.z;
		Serial.print((int)gyro.g.z);
		Serial.print("\t\t");
		Serial.println(gyro_null);
		delay(GYRO_SAMPLING_RATE);
	}
	
	gyro_null = gyro_null / 500.0;
	Serial.print("\n\ngyro_null is: ");
	Serial.print(gyro_null);
	delay(1000);
}
