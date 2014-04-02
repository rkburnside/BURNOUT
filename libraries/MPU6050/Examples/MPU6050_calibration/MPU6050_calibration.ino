// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define OUTPUT_READABLE_ACCELGYRO


int count = 0;

// NOTE:
// initial guesses for offsets; if you guess too small, it will not converge because we are adjusting the new guess in the wrong direction
long xAccelOff = -2460; 
long yAccelOff = -500; 
long zAccelOff = 1100; 
long xGyroOff = 40; 
long yGyroOff = 40; 
long zGyroOff = 60; 

boolean rough = true;
boolean medium = false;
boolean fine = false;
int adjustType = 1; // 1 = x accel, 2 = y accel, 3 = z accel, 4 = x gyro, 5 = y gyro, 6 = z gyro
const int countMax = 200; // number of iterations to check for settling of axes
int errorCheck = 5; // this value gets checked against the average of countMax number of samples, should be as close to 0 as possib

long readings[countMax];
long sum = 0;
long avg = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    accelgyro.setXGyroOffset(xGyroOff);
    accelgyro.setYGyroOffset(yGyroOff);
    accelgyro.setZGyroOffset(zGyroOff);
    accelgyro.setXAccelOffset(xAccelOff);
    accelgyro.setYAccelOffset(yAccelOff);
    accelgyro.setZAccelOffset(zAccelOff);

}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);


    // display tab-separated accel/gyro x/y/z values
    /*if (adjustType == 6) {
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
    }*/
    
    count++;
    
    if (adjustType == 3) { // z accel
      
      long val = az - 16384 ;
      
      // add the reading to the total:
      sum = sum + val;                           
      
      // calculate the average:
      avg = sum / count;
      
      if (count == countMax) {
        Serial.print("Average reading of "); Serial.print(avg); Serial.print(" with z accel offset of "); Serial.println(zAccelOff);
        if (avg > errorCheck && rough) {
          accelgyro.setZAccelOffset(zAccelOff -= 100);
          count = 0;
          sum = 0;
        } else if (avg < -errorCheck && rough || avg < -errorCheck && medium) {
          rough = false;
          medium = true;
          accelgyro.setZAccelOffset(zAccelOff += 10);
          count = 0;
          sum = 0;
        } else if (avg > errorCheck && medium || avg > errorCheck && fine) {
          medium = false;
          fine = true;
          accelgyro.setZAccelOffset(zAccelOff -= 1);
          count = 0;  
          sum = 0;
        }
      } else if (count > countMax) { 
        
        Serial.print("---> Converged z accel at: "); Serial.print(zAccelOff); Serial.print(" with average reading of: "); Serial.println(avg);
        adjustType++;
        count = 0;
        sum = 0;
        rough = false;
        medium = true;
        fine = false;
        
       }
        
    } else if (adjustType == 1) { // x accel

      long val = ax ;
      
      // add the reading to the total:
      sum = sum + val;                           
      
      // calculate the average:
      avg = sum / count;
      
      
      if (count == countMax) {
        Serial.print("Average reading of "); Serial.print(avg); Serial.print(" with x accel offset of "); Serial.println(xAccelOff);
        if (avg > errorCheck && rough) {
          accelgyro.setXAccelOffset(xAccelOff -= 100);
          count = 0;
          sum = 0;
        } else if (avg < -errorCheck && rough || avg < -errorCheck && medium) {
          rough = false;
          medium = true;
          accelgyro.setXAccelOffset(xAccelOff += 10);
          count = 0;
          sum = 0;
        } else if (avg > errorCheck && medium || avg > errorCheck && fine) {
          medium = false;
          fine = true;
          accelgyro.setXAccelOffset(xAccelOff -= 1);
          count = 0;  
          sum = 0;
        }
        
      } else if (count > countMax) { 
        
        Serial.print("---> Converged x accel at: "); Serial.print(xAccelOff); Serial.print(" with average reading of: "); Serial.println(avg);
        adjustType++;
        count = 0;
        rough = true;
        medium = false;
        fine = false;
        sum = 0;
        
       }
    
    } else if (adjustType == 2) { // y accel

      long val = ay ;
      
      // add the reading to the total:
      sum = sum + val;                           
      
      // calculate the average:
      avg = sum / count;

      if (count == countMax) {
        Serial.print("Average reading of "); Serial.print(avg); Serial.print(" with y accel offset of "); Serial.println(yAccelOff);
        if (avg > errorCheck && rough) {
          accelgyro.setYAccelOffset(yAccelOff -= 100);
          count = 0;
          sum = 0;
        } else if (avg < -errorCheck && rough || avg < -errorCheck && medium) {
          rough = false;
          medium = true;
          accelgyro.setYAccelOffset(yAccelOff += 10);
          count = 0;
          sum = 0;
        } else if (avg > errorCheck && medium || avg > errorCheck && fine) {
          medium = false;
          fine = true;
          accelgyro.setYAccelOffset(yAccelOff -= 1);
          count = 0;  
          sum = 0;
        }
      } else if (count > countMax) { 
        
        Serial.print("---> Converged y accel at: "); Serial.print(yAccelOff); Serial.print(" with average reading of: "); Serial.println(avg);
        adjustType++;
        count = 0;
        sum = 0;
        rough = true;
        medium = false;
        fine = false;
        
       }
        
    } else if (adjustType == 6) { // z gyro
      
      long val = gz;
      
      // add the reading to the total:
      sum = sum + val;                           
      
      // calculate the average:
      avg = sum / count;
      

      if (count == countMax) {
        Serial.print("Average reading of "); Serial.print(avg); Serial.print(" with z gyro offset of "); Serial.println(zGyroOff);
        if (avg > errorCheck && rough) {
          accelgyro.setZGyroOffset(zGyroOff -= 100);
          count = 0;
          sum = 0;
        } else if (avg < -errorCheck && rough || avg < -errorCheck && medium) {
          rough = false;
          medium = true;
          accelgyro.setZGyroOffset(zGyroOff += 10);
          count = 0;
          sum = 0;
        } else if (avg > errorCheck && medium || avg > errorCheck && fine) {
          medium = false;
          fine = true;
          accelgyro.setZGyroOffset(zGyroOff -= 1);
          count = 0;  
          sum = 0;
        }
      } else if (count > countMax) { 
        
        Serial.print("---> Converged z gyro at: "); Serial.print(zGyroOff); Serial.print(" with average reading of: "); Serial.println(avg);
        adjustType++;
        count = 0;
        sum = 0;
        
       }
        
    } else if (adjustType == 4) { // x gyro

      long val = gx ;
      
      // add the reading to the total:
      sum = sum + val;                           
      
      // calculate the average:
      avg = sum / count;

      if (count == countMax) {
        Serial.print("Average reading of "); Serial.print(avg); Serial.print(" with x gyro offset of "); Serial.println(xGyroOff);
        if (avg > errorCheck && rough) {
          accelgyro.setXGyroOffset(xGyroOff -= 100);
          count = 0;
          sum = 0;
        } else if (avg < -errorCheck && rough || avg < -errorCheck && medium) {
          rough = false;
          medium = true;
          accelgyro.setXGyroOffset(xGyroOff += 10);
          count = 0;
          sum = 0;
        } else if (avg > errorCheck && medium || avg > errorCheck && fine) {
          medium = false;
          fine = true;
          accelgyro.setXGyroOffset(xGyroOff -= 1);
          count = 0;  
          sum = 0;
        }
      } else if (count > countMax) { 
        
        Serial.print("---> Converged x gyro at: "); Serial.print(xGyroOff); Serial.print(" with average reading of: "); Serial.println(avg);
        adjustType++;
        count = 0;
        sum = 0;
        rough = false;
        medium = true;
        fine = false;
        
       }
    
    } else if (adjustType == 5) { // y gyro

      long val = gy ;
      
      // add the reading to the total:
      sum = sum + val;                           
      
      // calculate the average:
      avg = sum / count;

      if (count == countMax) {
        Serial.print("Average reading of "); Serial.print(avg); Serial.print(" with y gyro offset of "); Serial.println(yGyroOff);
        if (avg > errorCheck && rough) {
          accelgyro.setYGyroOffset(yGyroOff -= 100);
          count = 0;
          sum = 0;
        } else if (avg < -errorCheck && rough || avg < -errorCheck && medium) {
          rough = false;
          medium = true;
          accelgyro.setYGyroOffset(yGyroOff += 10);
          count = 0;
          sum = 0;
        } else if (avg > errorCheck && medium || avg > errorCheck && fine) {
          medium = false;
          fine = true;
          accelgyro.setYGyroOffset(yGyroOff -= 1);
          count = 0;  
          sum = 0;
        }
      } else if (count > countMax) { 
        
        Serial.print("---> Converged y gyro at: "); Serial.print(yGyroOff); Serial.print(" with average reading of: "); Serial.println(avg);
        adjustType++;
        count = 0;
        rough = false;
        medium = true;
        fine = false;
        
       }
        
    } else if (adjustType == 7) {
      Serial.println(" ");
      Serial.println("Calculated offsets:");
      Serial.print(" x accel: "); Serial.println(xAccelOff);
      Serial.print(" y accel: "); Serial.println(yAccelOff);
      Serial.print(" z accel: "); Serial.println(zAccelOff);
      Serial.print(" x gyro: "); Serial.println(xGyroOff);
      Serial.print(" y gyro: "); Serial.println(yGyroOff);
      Serial.print(" z gyro: "); Serial.println(zGyroOff);
      while(1) {} // stay here
    }
}
