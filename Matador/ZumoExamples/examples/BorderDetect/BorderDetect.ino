#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

const int far1 = 0;    //top left 10-80 cm sensor
const int far2 = 1;    //top right 10-80 cm sensor
const int far3 = 2;    //bottom left 10-80 cm sensor
const int far4 = 3;    //bottom right 10-80 cm sensor

#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  200 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     400 // 0 is stopped, 400 is full speed
#define TURN_SPEED        400
#define FORWARD_SPEED     400
#define FAST_SPEED     	  400 // 0 is stopped, 400 is full speed
#define MEDIUM_SPEED      400
#define SLOW_SPEED     	  300
#define LINE_DETECTED		1
#define SEARCH_ENEMY		2
#define ENEMY_LONG	        3
#define STOP_ROBOT	        4
#define ENEMY_LONG_REAR     5
#define SWITCH_ATTACK		6
#define SWITCH_ATTACK_REAR	7
#define ATTACK_1				8
#define SENSOR_FR		    1
#define SENSOR_FL		    2
#define SENSOR_RL		    4
#define SENSOR_RR			8
#define MAX_SEARCH			140

//#define REVERSE_DURATION  200 // ms
//#define TURN_DURATION     200 // ms

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

 
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

// Numbers of sensors and pins
#define NUM_SENSORS 4 
unsigned int sensor_values[NUM_SENSORS];
byte state, sensors_detected = 0;
int val[4];
byte pins[] = {4, 5, 11, 6};
ZumoReflectanceSensorArray sensors(pins, 4, 700, QTR_NO_EMITTER_PIN);
 
void raise_flag(){
  for (int i = 0; i<40; i++) {
    digitalWrite(2, LOW);
    delay(20);
    digitalWrite(2, HIGH);
    delayMicroseconds(1700);
  }
}  

void lower_flag(){
  for (int i = 0; i<40; i++) {
    digitalWrite(2, LOW);
    delay(20);
    digitalWrite(2, HIGH);
    delayMicroseconds(800);
  }
}  

byte waitForButtonAndCountDown()
{
  //digitalWrite(LED, HIGH);
  //button.waitForButton();
  //digitalWrite(LED, LOW);

  buttonState = digitalRead(12);
 // Serial.println(buttonState);
 // Serial.println(button.isPressed());

// if (buttonState == 1){  buzzer.playNote(NOTE_G(3), 200, 15);}
// if (buttonState == 0){  buzzer.playNote(NOTE_G(8), 200, 15);}
// delay(2000);
long start_time = millis();
  Serial.println(buttonState);
while  ((millis() - start_time) < 6000)  {  

		// if the state has changed, increment the counter
		//if (buttonState == 1) 
		//	{
				// if the current state is HIGH then the button
				// wend from off to on:
				buttonPushCounter++;
				// Serial.println(buttonState);
				// Serial.println("on");
				// Serial.print("number of button pushes:  ");
				// Serial.println(buttonPushCounter);
				 buzzer.playNote(NOTE_G(6), 200, 15);
				   Serial.println(buttonPushCounter);
		//	} 
			

button.waitForButton();
		
  }
Serial.println(buttonPushCounter);
 buzzer.playNote(NOTE_G(8), 500, 15); 
    button.waitForButton();
  // play audible countdown
  for (int i = 0; i < 4; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
 
 // lower_flag();
   switch (buttonPushCounter) {
    case 1:
     state = SEARCH_ENEMY;
	 return(SEARCH_ENEMY);
	 //do something when var equals 1
      break;
    case 2:
	state = ATTACK_1;
	 return(ATTACK_1);
      //do something when var equals 2
      break;

  }

 // delay(1000);
}

// byte stopRobot()
// {		//raise_flag();//raise the flag when button is pressed to stop
	    // motors.setSpeeds(0, 0);
		// button.waitForRelease();
		// waitForButtonAndCountDown();
		// state = SEARCH_ENEMY;
	
// }

byte lineDetected2()
{
	Serial.println(sensors_detected, BIN);
	switch (sensors_detected) {
		case (SENSOR_FL):
			motors.setSpeeds(-MEDIUM_SPEED, -SLOW_SPEED);
			Serial.println("Front Left");
		break;
		
		case (SENSOR_FR):
			motors.setSpeeds(-SLOW_SPEED, -MEDIUM_SPEED);
			Serial.println("Front Right");
		break;
		  
		case (SENSOR_FL+SENSOR_FR):
			motors.setSpeeds(-MEDIUM_SPEED, -MEDIUM_SPEED);
			Serial.println("Front Both");
		break;

		case (SENSOR_RL):
			motors.setSpeeds(MEDIUM_SPEED, SLOW_SPEED);
			buzzer.playNote(NOTE_G(3), 200, 15);
			Serial.println("Rear Left");
		break;

		case (SENSOR_RR):
			motors.setSpeeds(SLOW_SPEED, MEDIUM_SPEED);
			buzzer.playNote(NOTE_G(3), 200, 15);
			Serial.println("Rear Right");
		break;

		case (SENSOR_RL+SENSOR_RR):
			motors.setSpeeds(MEDIUM_SPEED, MEDIUM_SPEED);
			buzzer.playNote(NOTE_G(3), 200, 15);
			Serial.println("Rear Both");
		break;
		
		// default: 
			// Serial.println("None");
		// break;
		}
	delay(300);
	return(SEARCH_ENEMY);
}

byte readLineSensors()
{
	byte edge_sensors=0;
	sensors.read(sensor_values);
    if (sensor_values[0] < QTR_THRESHOLD) edge_sensors += SENSOR_FL;
    if (sensor_values[1] < QTR_THRESHOLD) edge_sensors += SENSOR_FR;
    if (sensor_values[2] < QTR_THRESHOLD) edge_sensors += SENSOR_RL;
    if (sensor_values[3] < QTR_THRESHOLD) edge_sensors += SENSOR_RR;
	//Serial.println(edge_sensors, BIN);
	return (edge_sensors);
}

byte attack1 ()		
{
motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
						// sensors_detected = readLineSensors();
						// if(sensors_detected > 0) return(LINE_DETECTED);
delay(350);
 motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
						// // sensors_detected = readLineSensors();
						// // if(sensors_detected > 0) return(LINE_DETECTED);
delay(130);
motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
						// sensors_detected = readLineSensors();
						// if(sensors_detected > 0) return(LINE_DETECTED);
delay(330);
 motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
						// // sensors_detected = readLineSensors();
						// // if(sensors_detected > 0) return(LINE_DETECTED);
delay(170);
 motors.setSpeeds(-0, 0);
						// // sensors_detected = readLineSensors();
						// // if(sensors_detected > 0) return(LINE_DETECTED);
// delay(180);
//motors.setSpeeds(TURN_SPEED*5, TURN_SPEED);
return(SEARCH_ENEMY);
}

// byte switchAttack ()		
// {
// motors.setSpeeds(-TURN_SPEED, TURN_SPEED*0.5);
						// // sensors_detected = readLineSensors();
						// // if(sensors_detected > 0) return(LINE_DETECTED);
// delay(170);
 // motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
						// // // sensors_detected = readLineSensors();
						// // // if(sensors_detected > 0) return(LINE_DETECTED);
// delay(110);
// // motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
						// // // sensors_detected = readLineSensors();
						// // // if(sensors_detected > 0) return(LINE_DETECTED);
// // delay(180);
// //motors.setSpeeds(TURN_SPEED*5, TURN_SPEED);
// return(SEARCH_ENEMY);
// }

byte switchAttack ()		
{
motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
						// sensors_detected = readLineSensors();
						// if(sensors_detected > 0) return(LINE_DETECTED);
delay(175);
 motors.setSpeeds(-TURN_SPEED, -TURN_SPEED);
						// // sensors_detected = readLineSensors();
						// // if(sensors_detected > 0) return(LINE_DETECTED);
delay(175);
 motors.setSpeeds(-0, -0);
						// // sensors_detected = readLineSensors();
						// // if(sensors_detected > 0) return(LINE_DETECTED);
delay(2000);
// motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
						// // sensors_detected = readLineSensors();
						// // if(sensors_detected > 0) return(LINE_DETECTED);
// delay(180);
//motors.setSpeeds(TURN_SPEED*5, TURN_SPEED);
return(SEARCH_ENEMY);
}


byte switchAttackRear ()
		

{
motors.setSpeeds(TURN_SPEED*0.6, TURN_SPEED);
					    // sensors_detected = readLineSensors();
						// if(sensors_detected > 0) return(LINE_DETECTED);
delay(90);
motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
						// sensors_detected = readLineSensors();
						// if(sensors_detected > 0) return(LINE_DETECTED);
delay(150);
motors.setSpeeds(TURN_SPEED, TURN_SPEED);
						// sensors_detected = readLineSensors();
						// if(sensors_detected > 0) return(LINE_DETECTED);
delay(130);
motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
						// sensors_detected = readLineSensors();
						// if(sensors_detected > 0) return(LINE_DETECTED);
delay(150);
//motors.setSpeeds(TURN_SPEED*5, TURN_SPEED);
return(SEARCH_ENEMY);
}




byte enemyLong ()
{
long start_time = millis();
	while(true) {
	readReflectorValues();
		if (val[0] > MAX_SEARCH && val[1]<MAX_SEARCH)    // long range turn left
			{
				motors.setSpeeds(TURN_SPEED*5, TURN_SPEED); //turn left slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
			//delay(50);
			}
		else if (val[0] < MAX_SEARCH && val[1]>MAX_SEARCH)    // long range turn right
			{
				motors.setSpeeds(TURN_SPEED*.2, TURN_SPEED); //turn right slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
				//delay(50);
			}	
		if (val[0] > MAX_SEARCH+50 && val[1]<MAX_SEARCH)    // long range turn left
			{
				motors.setSpeeds(-TURN_SPEED*5, TURN_SPEED); //turn left slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
			//delay(50);
			}
		else if (val[0] < MAX_SEARCH && val[1]>MAX_SEARCH+50)    // long range turn right
			{
				motors.setSpeeds(TURN_SPEED, -TURN_SPEED*.5); //turn right slow
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

		else if (val[0] >MAX_SEARCH && val[1]>MAX_SEARCH)
		{
			motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED); //attack
		    sensors_detected = readLineSensors();
		    if(sensors_detected > 0) return(LINE_DETECTED);
//			if ((millis() - start_time) > 1000)  raise_flag();
			if ((millis() - start_time) > 2000)  return(SWITCH_ATTACK);

		}
		
		// if (button.isPressed())
		// {
		// // if button is pressed, stop and wait for another press to go again
			// return(STOP_ROBOT);
		// }   
		
		else return(SEARCH_ENEMY);

	}
}
byte enemyLongRear()
{
	int var2=0;
	long start_time = millis();
	while(true) {
	readReflectorValues();
		if (val[2] > MAX_SEARCH && val[3]<MAX_SEARCH)    // long range turn left
			{
				motors.setSpeeds(-TURN_SPEED*5, -TURN_SPEED); //turn left slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
			//delay(50);
			}
		else if (val[2] < MAX_SEARCH && val[3]>MAX_SEARCH)    // long range turn right
			{
				motors.setSpeeds(-TURN_SPEED*.2, -TURN_SPEED); //turn right slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
				//delay(50);
			}	
		if (val[2] > MAX_SEARCH+50 && val[3]<MAX_SEARCH)    // long range turn left
			{
				motors.setSpeeds(TURN_SPEED*5, -TURN_SPEED); //turn left slow
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
			//delay(50);
			}
		else if (val[2] < MAX_SEARCH && val[3]>MAX_SEARCH+50)    // long range turn right
			{
				motors.setSpeeds(-TURN_SPEED, TURN_SPEED*.5); //turn right slow
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

		else if (val[2] >MAX_SEARCH && val[3]>MAX_SEARCH)
		{
		
					//do
					//{					
					//while(var2=0 ) 		
					//{
					// do something repetitive 200 times
					//for (int i = 0; i < 5000; i // working for
					//{ // working for
						motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED); //attack
						sensors_detected = readLineSensors();
						if(sensors_detected > 0) return(LINE_DETECTED);
						
							// {
								// buzzer.playNote(NOTE_G(4), 500, 15);
								// Serial.println(sensors_detected, BIN);
								// return(LINE_DETECTED);
								
							//	 i=5000;						// working for
						// }
						// var2++;
							//Serial.println(sensors_detected, BIN); // working for
						//delay(1); // working for
						//};// working for
						//
					//} 
					//Serial.println("Senor"); 
						//Serial.println(sensors_detected, BIN);
						//buzzer.playNote(NOTE_G(4), 500, 15);
						//delay(5000);
					
					// if (var2=6) {  
					 // Serial.print("\t"); 					
					// Serial.print(LINE_DETECTED);
					// buzzer.playNote(NOTE_G(4), 500, 15);  
                    // return(LINE_DETECTED);					
					//}
					//motors.setSpeeds(0, 0); //attack
					//delay(2000);
	//				if ((millis() - start_time) > 1000)  raise_flag();
					if ((millis() - start_time) > 2000)  return(SWITCH_ATTACK_REAR);
					
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
		if (val[0] < MAX_SEARCH && val[1]<MAX_SEARCH && val[2]<MAX_SEARCH && val[3]<MAX_SEARCH ) //search spin turn right
			{
				motors.setSpeeds(TURN_SPEED*.7, -TURN_SPEED*.7);
				sensors_detected = readLineSensors();
				if(sensors_detected > 0) return(LINE_DETECTED);
			}
		else if (val[0] > MAX_SEARCH || val[1]>MAX_SEARCH ) //search spin turn right
			{
			//motors.setSpeeds(0, 0);
			return(ENEMY_LONG);
			}
		
		else if (val[2] > MAX_SEARCH || val[3]>MAX_SEARCH ) //search spin turn right
			{
			//motors.setSpeeds(0, 0);
			return(ENEMY_LONG_REAR);
			}
	}
}

void readReflectorValues() {	// Read the sharp distance sensors  the far sensors are analog
  val[0] = analogRead(far1);    // Top Left 
  val[1] = analogRead(far2);    // Top Right
  val[2] = analogRead(far3);    // Bottom Left 
  val[3] = analogRead(far4);    // Bottom Right
 // //Serial.print(val[0]);
  // //Serial.print("\t");      // this prints a tab
  // Serial.print(state);
  // Serial.print("\t");
  // Serial.print(val[0]);
  // Serial.print("\t");
  // Serial.println(val[1]);  //serial.println make a new line
}
 
void setup()
{
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
   Serial.begin(115200);
  //state = SEARCH_ENEMY; 
  pinMode(LED, HIGH);
  pinMode(2, OUTPUT);  //flag
  waitForButtonAndCountDown();
}

void loop()
{
//  readReflectorValues();
//  sensors.read(sensor_values);


switch (state) {
    case LINE_DETECTED:
		state = lineDetected2();
 //do something when var equals 1
      break;
    case SEARCH_ENEMY:
	state = searchEnemy();
      break;
	case ENEMY_LONG:
	state = enemyLong();
      break;
	case ENEMY_LONG_REAR:
	state = enemyLongRear();
      break;
	case SWITCH_ATTACK:
	state = switchAttack();
	  break;
	case SWITCH_ATTACK_REAR:
	state = switchAttackRear();
	  break;
	case ATTACK_1:
	state = attack1();
	  break;
	
	// case STOP_ROBOT:
	// state = stopRobot();
     // // do something when var equals 2
     // break;
    //default: 
      // if nothing else matches, do the default
      // default is optional
  }

}
