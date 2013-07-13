#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

const int far1 = 0;    //10-80 cm sensor
const int far2 = 1;    //10-80 cm sensor

#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  500 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     300 // 0 is stopped, 400 is full speed
#define TURN_SPEED        300
#define FORWARD_SPEED     300
#define LINE_DETECTED		1
#define SEARCH_ENEMY		2
#define ENEMY_LONG	        3
#define STOP_ROBOT	        4
#define SENSOR_FR		1
#define SENSOR_FL		2
#define SENSOR_RL		4
#define SENSOR_RR		8
//#define REVERSE_DURATION  200 // ms
//#define TURN_DURATION     200 // ms
 
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

// Numbers of sensors and pins
#define NUM_SENSORS 3 
unsigned int sensor_values[NUM_SENSORS];
byte state, sensors_detected = 0;
int val[4];
byte pins[] = {4, 11, 5, 6};
ZumoReflectanceSensorArray sensors(pins, 4, 800, QTR_NO_EMITTER_PIN);
 

void waitForButtonAndCountDown()
{
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 4; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
 // delay(1000);
}

// byte stopRobot()
// {
	    // motors.setSpeeds(0, 0);
		// button.waitForRelease();
		// waitForButtonAndCountDown();
		// state = SEARCH_ENEMY;
	
// }
byte lineDetected()
{
	if(sensors_detected & SENSOR_RL) {
		readReflectorValues();
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		buzzer.playNote(NOTE_G(4), 500, 15);
		while(readLineSensors());
		delay(100);
	}
	
	else {
	    readReflectorValues();
		motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
		buzzer.playNote(NOTE_G(3), 200, 15);
		while(readLineSensors());
		delay(100);
	}
    return(SEARCH_ENEMY);
}

byte readLineSensors()
{
	sensors_detected=0;
	sensors.read(sensor_values);
    if (sensor_values[0] < QTR_THRESHOLD) sensors_detected += SENSOR_FL;
    if (sensor_values[1] < QTR_THRESHOLD) sensors_detected += SENSOR_RL;
    if (sensor_values[2] < QTR_THRESHOLD) sensors_detected += SENSOR_FR;
    if (sensor_values[3] < QTR_THRESHOLD) sensors_detected += SENSOR_RR;
	return (sensors_detected);
}


byte enemyLong ()
{
	
	while(true) {
	readReflectorValues();
		if (val[0] > 200 && val[1]<200)    // long range turn right
			{
				motors.setSpeeds(TURN_SPEED, TURN_SPEED*.1); //turn right slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
			//delay(50);
			}
		else if (val[0] < 200 && val[1]>200)    // long range turn right
			{
				motors.setSpeeds(-TURN_SPEED*.3, TURN_SPEED); //turn left slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
				//delay(50);
			}	
		
		// if (val[0] > 150 && val[1]<160)
			// {
				// motors.setSpeeds(TURN_SPEED, -TURN_SPEED*.3); //short range turn right medium
				// sensors_detected = readLineSensors();
				// if(sensors_detected > 0) return(LINE_DETECTED);//    delay(50);
			// }
	
		// else 
		
		// if (val[0] <150 && val[1]>160)
			// {
				// motors.setSpeeds(-TURN_SPEED*.3, TURN_SPEED); //short range turn right medium
			    // sensors_detected = readLineSensors();
				// if(sensors_detected > 0) return(LINE_DETECTED);// delay(50);
			// }
	
		// else

 
		//else

		else if (val[0] >200 && val[1]>200)
		{
			motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED); //attack
		    sensors_detected = readLineSensors();
		    if(sensors_detected > 0) return(LINE_DETECTED);
		}
		
		// if (button.isPressed())
		// {
		// // if button is pressed, stop and wait for another press to go again
			// return(STOP_ROBOT);
		// }   
		
		else return(SEARCH_ENEMY);

	}
}

byte searchEnemy()
{
	while(true) {
		readReflectorValues();
		if (val[0] < 90 && val[1]<90) //search spin turn right
			{
				motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
			}
		else {
		//motors.setSpeeds(0, 0);
		return(ENEMY_LONG);
		}
		
		
	}
}

void readReflectorValues() {
  val[0] = analogRead(far1);    // the far sensors are analog
  val[1] = analogRead(far2);
 //Serial.print(val[0]);
  //Serial.print("\t");      // this prints a tab
  Serial.print(state);
  Serial.print("\t");
  Serial.print(val[0]);
  Serial.print("\t");
  Serial.println(val[1]);  //serial.println make a new line

  
}
 
void setup()
{
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
   Serial.begin(115200);
  state = SEARCH_ENEMY; 
  pinMode(LED, HIGH);
   
  waitForButtonAndCountDown();
}

void loop()
{
//  readReflectorValues();
//  sensors.read(sensor_values);


switch (state) {
    case LINE_DETECTED:
		state = lineDetected();
 //do something when var equals 1
      break;
    case SEARCH_ENEMY:
	state = searchEnemy();
      //do something when var equals 2
      break;
	case ENEMY_LONG:
	state = enemyLong();
      //do something when var equals 2
      break;
	
//	case STOP_ROBOT:
//	state = stopRobot();
      //do something when var equals 2
//      break;
    //default: 
      // if nothing else matches, do the default
      // default is optional
  }

  
// if (sensor_values[0] < QTR_THRESHOLD)
  // {
    // // if leftmost sensor detects line, reverse and turn to the right
    // motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    // delay(500);
   
  // }
  // else 
  // if (sensor_values[2] < QTR_THRESHOLD)
  // {
    // // if rightmost sensor detects line, reverse and turn to the left
    
   // motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
  // delay(500);  }
    

  // else 
  // if (sensor_values[1]  <1500)
  // {
    // // if rightmost sensor detects line, reverse and turn to the left
    
    // motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    // delay(500);
  // }



 // else
  
  
  
 // if (val[2] > 80 && val[3]<80)    // long range turn right
 // {
 // motors.setSpeeds(TURN_SPEED, TURN_SPEED*.8); //turn right slow
    // //delay(50);
	// }


// else 
// if (val[2] < 80 && val[3]>80)    // long range turn right
 // {
 // motors.setSpeeds(TURN_SPEED*.8, TURN_SPEED); //turn left slow
    // //delay(50);
	// }
// else 
 
 
    // if (val[2] > 150 && val[3]<160)
	// {
    // motors.setSpeeds(TURN_SPEED, -TURN_SPEED*0.5); //short range turn right medium
// //    delay(50);
	// }
	
// else 
  // if (val[2] <150 && val[3]>160)
	// {
    // motors.setSpeeds(-TURN_SPEED*.5, TURN_SPEED); //short range turn right medium
// //    delay(50);
	// }
	
// else

 	
  // if (val[2] >300 && val[3]>300)
   // {
 // motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED); //attack
   // }
   
  
 
 
 
   
 
  
  
   

//  sensors.read(sensor_values);
  
  //delay(50);   // wait 100 ms between loops
  
}

