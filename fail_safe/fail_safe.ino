/*fail_safe.ino
This file is intended for the arduino pro-mini to monitor channel 3 from the radio and will toggle the muxer and communicate the channel 3 position to the main MCU board (i.e. teensy 3.1).
This code is based on the arduipilot fail safe developed by Chris Anderon, Jordi Munoz, Nathan (SparkFun).
*/

/* required functions

1. function that times how quickly channel three is toggled from low to high back to low. When it has reached three times, it will hard reset the MCU

2. function that will flash the LED in 1 of 4 ways:
	manual = 0
	state 1 = quick flashing
	state 2 = slow flashing
	automatic = constanst LED

x3. function that will determine the state of channel 3. servo pulse range = 1mSec ~ 2mSec. 1mSec would cause the shaft to revolve fully left. A 2mSec positive pulse width would cause the drive shaft to revolve fully right. 1.5mSec pulse width would cause the shaft to turn to the middle of the revolution area.
	
	manual -> manual <1250ms
	state 1 -> 1250ms <= state 1 < 1500ms
	state 2 -> 1500ms <= state 2 < 1750ms
	automatic -> 1750ms <= automatic

*/


#define SWITCH_POSITION_MANUAL 0		//full manual control of the car
#define SWITCH_POSITION_1 1			//switch state 1
#define SWITCH_POSITION_2 2			//switch state 2
#define SWITCH_POSITION_AUTOMATIC 3	//full autonomous mode

int rest_pin = 1;		//used to reset the external MCU
int ch_3_in = 2;		//pin to monitor radio channel 3
int ch_3_state_1 = 3;		//output line 1 to external MCU
int ch_3_state_2 = 4;		//output line 2 to external MCU
int multiplexor = 5;	//multiplexor toggle
int state_led = 13;		//MCU state led
int pulse_length = 0;
int switch_position = 0;

void setup(){
	pinMode(rest_pin, OUTPUT);
	pinMode(ch_3_in, INPUT);
	pinMode(ch_3_state_1, OUTPUT);
	pinMode(ch_3_state_2, OUTPUT);
	pinMode(state_led, OUTPUT);
	pinMode(multiplexor, OUTPUT);

	digitalWrite(rest_pin, LOW);
    digitalWrite(multiplexor, LOW);

	set_vehile_state(SWITCH_POSITION_MANUAL);
	visual_status(SWITCH_POSITION_MANUAL);
}

void loop(){
	pulse_length = pulseIn(ch_3_in, HIGH, 50000);	//function will time out after 50000 uS if no pulse is received
	determine_switch_position(void);
	set_vehile_state(void);
}

void determine_switch_position(void){
	if(pulse_length < 1250) switch_position = SWITCH_POSITION_MANUAL;
	else if((pulse_length >= 1250) && (pulse_length < 1500)) switch_position = SWITCH_POSITION_1;
	else if((pulse_length >= 1500) && (pulse_length < 1750)) switch_position = SWITCH_POSITION_2;
	else switch_position = SWITCH_POSITION_AUTOMATIC;
	
	return;
}

void set_vehile_state(void){
	switch (switch_position){
		case SWITCH_POSITION_1:
			digitalWrite(multiplexor, LOW);
			digitalWrite(ch_3_state_1, LOW);
			digitalWrite(ch_3_state_2, LOW);
			digitalWrite(multiplexor, HIGH);
			break;

		case SWITCH_POSITION_2:
			digitalWrite(multiplexor, LOW);
			digitalWrite(ch_3_state_1, LOW);
			digitalWrite(ch_3_state_2, HIGH);
			digitalWrite(multiplexor, HIGH);
			break;
		
		case SWITCH_POSITION_AUTOMATIC:
			digitalWrite(multiplexor, LOW);
			digitalWrite(ch_3_state_1, HIGH);
			digitalWrite(ch_3_state_2, LOW);
			digitalWrite(multiplexor, HIGH);
			break;
		
		default:	//default state is SWITCH_POSITION_MANUAL
			digitalWrite(multiplexor, LOW);
	}
	
	return;
}

void visual_status(int ){
	digitalWrite(ledPin, HIGH);   // sets the LED on
	delay(1000);                  // waits for a second
	digitalWrite(ledPin, LOW);    // sets the LED off
	delay(1000);                  // waits for a second
}