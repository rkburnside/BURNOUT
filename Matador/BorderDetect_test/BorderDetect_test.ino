#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
 
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     0 // 0 is stopped, 400 is full speed
#define TURN_SPEED        0
#define FORWARD_SPEED     0
#define REVERSE_DURATION  0 // ms
#define TURN_DURATION     0 // ms
 
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 4
unsigned int sensor_values[NUM_SENSORS];

byte pins[] = {4, 11, 6, 5};

ZumoReflectanceSensorArray sensors(pins, 4);
 

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
   
  pinMode(LED, HIGH);
   
  //waitForButtonAndCountDown();
}

void loop()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
   

  sensors.read(sensor_values);
  
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(200);
   
  }
  else if (sensor_values[2] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    
   motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);  }
 
  else if (sensor_values[1]  <1500)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  
  {
    // otherwise, go straight
   // motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  
  
  Serial.begin(115200);
Serial.print(sensor_values[5]);
  Serial.print("\t"); 
  Serial.print(sensor_values[4]);
  Serial.print("\t");
  Serial.print(sensor_values[3]);
  Serial.print("\t");
  Serial.print(sensor_values[2]);
  Serial.print("\t"); 
  Serial.print(sensor_values[1]);
  Serial.print("\t"); 
  Serial.print(sensor_values[0]);
  Serial.print("\t"); 
  Serial.println();
   delay(200);
}
