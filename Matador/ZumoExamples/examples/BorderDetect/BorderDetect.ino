#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

const int close1 = 4;   //10 cm sensor
const int close2 = 5;   //10 cm sensor
const int far1 = 0;    //10-80 cm sensor
const int far2 = 1;    //10-80 cm sensor
 
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     400
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     400 // ms
 
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
 
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

void waitForButtonAndCountDown()
{
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
  delay(1000);
}
 
void setup()
{
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
   Serial.begin(115200);
   pinMode(close1, INPUT);   // the close sensors are digital
  pinMode(close2, INPUT);
  pinMode(LED, HIGH);
   
  waitForButtonAndCountDown();
}

void loop()
{
int val[4];
  val[0] = digitalRead(close1);
  val[1] = digitalRead(close2);
  val[2] = analogRead(far1);    // the far sensors are analog
  val[3] = analogRead(far2);

 if (val[2] < 80 && val[3]<80) //search spin
 {
  motors.setSpeeds(TURN_SPEED*0.5, -TURN_SPEED*0.5);
 }
 else
  
  
 if (val[2] > 80 && val[3]<80)    // long range turn right
 {
 motors.setSpeeds(TURN_SPEED, TURN_SPEED*.8); //turn right slow
    //delay(50);
	}


else 
if (val[2] < 80 && val[3]>80)    // long range turn right
 {
 motors.setSpeeds(TURN_SPEED*.8, TURN_SPEED); //turn left slow
    //delay(50);
	}
else 
 
 
    if (val[2] > 150 && val[3]<160)
	{
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED*0.5); //short range turn right medium
//    delay(50);
	}
	
else 
  if (val[2] <150 && val[3]>160)
	{
    motors.setSpeeds(-TURN_SPEED*.5, TURN_SPEED); //short range turn right medium
//    delay(50);
	}
	
else

 	
  if (val[2] >190 && val[3]>190)
   {
 motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED); //attack
   }
   
  
 
 
 
   
 
  
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
   

  sensors.read(sensor_values);
  
 //Serial.print(val[0]);
  //Serial.print("\t");      // this prints a tab
  //Serial.print(val[1]);
  //Serial.print("\t");
  Serial.print(val[2]);
  Serial.print("\t");
  Serial.println(val[3]);  //serial.println make a new line

  delay(50);   // wait 100 ms between loops
  
}
