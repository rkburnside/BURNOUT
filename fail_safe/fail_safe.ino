/*fail_safe.ino
This file is intended for the arduino pro-mini to monitor channel 3 from the radio and will toggle the muxer and communicate the channel 3 position to the main MCU board (i.e. teensy 3.1).
This code is based on the arduipilot fail safe developed by Chris Anderon, Jordi Munoz, Nathan (SparkFun).
*/

#define SWITCH_STATE_MANUAL 0		//full manual control of the car
#define SWITCH_STATE_1 1			//switch state 1
#define SWITCH_STATE_2 2			//switch state 2
#define SWITCH_STATE_AUTOMATIC 3	//full autonomous mode

int rest_pin = 1;		//used to reset the external MCU
int ch_3_in = 2;		//pin to monitor radio channel 3
int ch_3_state_1 = 3;		//output line 1 to external MCU
int ch_3_state_2 = 4;		//output line 2 to external MCU
int multiplexor = 5;	//multiplexor toggle
int state_led = 13;		//MCU state led
int pulse_length = 0;

void setup(){
	pinMode(rest_pin, OUTPUT);
	pinMode(ch_3_in, INPUT);
	pinMode(ch_3_state_1, OUTPUT);
	pinMode(ch_3_state_2, OUTPUT);
	pinMode(state_led, OUTPUT);
	pinMode(multiplexor, OUTPUT);

	digitalWrite(rest_pin, LOW);
    digitalWrite(multiplexor, LOW);

	state(SWITCH_STATE_MANUAL);
	visual_status(SWITCH_STATE_MANUAL);

}

void loop(){

	pulse_length = pulseIn(ch_3_in, HIGH, 50000);
	switch_position

}

void state(int switch_status){
	switch (switch_status){
		case SWITCH_STATE_1:
			digitalWrite(multiplexor, LOW);
			digitalWrite(ch_3_state_1, LOW);
			digitalWrite(ch_3_state_2, LOW);
			digitalWrite(multiplexor, HIGH);
			break;

		case SWITCH_STATE_2:
			digitalWrite(multiplexor, LOW);
			digitalWrite(ch_3_state_1, LOW);
			digitalWrite(ch_3_state_2, HIGH);
			digitalWrite(multiplexor, HIGH);
			break;
		
		case SWITCH_STATE_AUTOMATIC:
			digitalWrite(multiplexor, LOW);
			digitalWrite(ch_3_state_1, HIGH);
			digitalWrite(ch_3_state_2, LOW);
			digitalWrite(multiplexor, HIGH);
			break;
		
		default:	//default state is SWITCH_STATE_MANUAL
			digitalWrite(multiplexor, LOW);
			digitalWrite(ch_3_state_1, HIGH);
			digitalWrite(ch_3_state_2, HIGH);
			digitalWrite(multiplexor, HIGH);
	}
	
	return;
}

void visual_status(int ){
	digitalWrite(ledPin, HIGH);   // sets the LED on
	delay(1000);                  // waits for a second
	digitalWrite(ledPin, LOW);    // sets the LED off
	delay(1000);                  // waits for a second
}

int channel_3_state(int ch_3_time){
	if(pulse_length < 1000) 
}


