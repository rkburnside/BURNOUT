/*
An Arduino code example for interfacing with the HMC5883

by: Jordan McConnell
 SparkFun Electronics
 created on: 6/30/11
 license: OSHW 1.0, http://freedomdefined.org/OSHW

Analog input 4 I2C SDA
Analog input 5 I2C SCL
*/

#include <Wire.h> //I2C Arduino Library

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

void setup(){
  //Initialize Serial and I2C communications
  Serial.begin(115200);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.write(0x00); //select config A register
  Wire.write(0b00011000); //75 hz measurements
  Wire.endTransmission();
}

void loop(){
  
  int x,y,z; //triple axis data
  long xl=0, yl=0;
  double angle = 0;
  double temp = 0;

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 for(int i=0; i < 200; i++) {
	//Read data from each axis, 2 registers per axis
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
	Wire.requestFrom(address, 6);
	if(6<=Wire.available()){
	x = Wire.read()<<8; //X msb
		x |= Wire.read(); //X lsb
		z = Wire.read()<<8; //Z msb
		z |= Wire.read(); //Z lsb
		y = Wire.read()<<8; //Y msb
		y |= Wire.read(); //Y lsb
	}
	xl += x;
	yl += y;
	// Serial.print("x: ");
	// Serial.print(x);
	// Serial.print("  y: ");
	// Serial.print(y);
	 //temp += atan2(x,y)/3.14159*180.0;
	 //Serial.println(temp);
	delay(50);
  }
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(xl);
  Serial.print("  y: ");
  Serial.print(yl);
  Serial.print("angle: ");
  Serial.println(atan2(xl/200.0, yl/200.0)/3.14159*180.0);
    
}

