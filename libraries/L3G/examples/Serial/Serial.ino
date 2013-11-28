#include <Wire.h>
#include <L3G.h>

L3G gyro;

void setup() {
	Serial.begin(115200);
	Wire.begin();

	if (!gyro.init()) {
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}

	gyro.enableDefault();
	//  Serial.print("G ");
	Serial.println("X,Y,Z");
}

void loop() {
	gyro.read();

	Serial.print((int)gyro.g.x);
	Serial.print("\t");
	Serial.print((int)gyro.g.y);
	Serial.print("\t");
	Serial.println((int)gyro.g.z);

	delay(12);
}
