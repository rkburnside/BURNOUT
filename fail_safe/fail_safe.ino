/*fail_safe.ino
This file is intended for the arduino pro-mini to monitor channel 3 from the radio and will toggle the muxer and communicate the channel 3 position to the main MCU board (i.e. teensy 3.1).

This code concept is based on the arduipilot fail safe developed by Chris Anderon, Jordi Munoz, Nathan (SparkFun).

Description and Functionality
1. if the switch is toggled from high to low and back 3 times in less than 1 second, a hard reset will be performed on the main MCU board.

2. the LED on the arduino will flash as follows, depending on the controlling state:
	manual = 0
	automatic = constant LED
	set waypoint = fast flashing
	reset = fast slow flashing

3. a servo pulse range is 1000us ~ 2000us. the channel 3 switch positions should be configured as follows to ensure the proper state is detected and enabled:
	manual -> manual (set to ~1000us) <1250us
	automatic -> 1250us <= automatic (set to ~1330) < 1750us
	set waypoint -> 1750us <= set waypoint (set to ~2000us)
	
4. multiplexor selection:
	channel A = MCU = A/B select LOW
	channel B = RX = A/B select HIGH
*/

#define TIME_TO_FLIP_SWITCH 1000	//time in ms to flip the switch 3 times
#define RESET_SWITCH_COUNTER 3		//number to reset the main MCU
#define SWITCH_POSITION_MANUAL 0	//manual control of the car
#define SWITCH_POSITION_AUTOMATIC 1 //automatic control of the car
#define SWITCH_POSITION_WAYPOINT 2	//set waypoint
#define SWITCH_POSITION_RESET 3		//reset the main MCU
#define PULSE_LENGTH_LOW 1250
#define PULSE_LENGTH_HIGH 1750


int state_led = 13;			//MCU state led
int hard_reset_pin = 12;	//used to reset the external MCU
int mode_1 = 11;			//output line 1 to external MCU
int mode_2 = 10;			//output line 2 to external MCU
int multiplexor = 9;		//multiplexor toggle
int ch_3_in = 8;			//pin to monitor radio channel 3
int pulse_in_length = 0;	//pulse in from the RX
int switch_position = SWITCH_POSITION_MANUAL;	//the initial position will be MANUAL


void setup(){
	pinMode(state_led, OUTPUT);
	pinMode(hard_reset_pin, OUTPUT);
	pinMode(mode_1, OUTPUT);
	pinMode(mode_2, OUTPUT);
	pinMode(multiplexor, OUTPUT);
	pinMode(ch_3_in, INPUT);

	digitalWrite(hard_reset_pin, LOW);
	digitalWrite(mode_1, LOW);
	digitalWrite(mode_2, LOW);
	digitalWrite(multiplexor, LOW);
	digitalWrite(state_led, LOW);

	set_vehile_state();
	flash_led();
}

void loop(){
	get_pulse_length();
	determine_switch_position();
	set_vehile_state();
	flash_led();
	check_if_reset_requested();
}

void get_pulse_length(){
	static int temp_pulse_length = 0;

	//servos receive a HIGH pulse of 1.0ms~2.0ms (1500us~2000us) at a duty cycle of 40Hz~200Hz (i.e. 5000us~25000us), depending on the RX/TX set up. The max cycle time would be 27000us. The pulse function will time out after 30000us if no pulse is received. pulseIn will return 0 if time out occurs.
	temp_pulse_length = pulseIn(ch_3_in, HIGH, 30000);

	if(temp_pulse_length > 500) pulse_in_length = temp_pulse_length;	//if a good pulse is received (i.e. not 0), pulse_in_length is accepted and saved

	return;
}

void determine_switch_position(){
	if(pulse_in_length < PULSE_LENGTH_LOW) switch_position = SWITCH_POSITION_MANUAL;
	else if((pulse_in_length >= PULSE_LENGTH_LOW) && (pulse_in_length < PULSE_LENGTH_HIGH)) switch_position = SWITCH_POSITION_AUTOMATIC;
	else switch_position = SWITCH_POSITION_WAYPOINT;
	
	return;
}

void set_vehile_state(){
	switch(switch_position){
		case SWITCH_POSITION_WAYPOINT:
			digitalWrite(multiplexor, HIGH);	//multiplexor HIGH puts car in MANUAL mode
			digitalWrite(mode_1, LOW);			//set the pins states
			digitalWrite(mode_2, LOW);
			digitalWrite(multiplexor, LOW);		//now enable the main MCU control
			break;

		case SWITCH_POSITION_AUTOMATIC:
			digitalWrite(multiplexor, HIGH);	//multiplexor HIGH puts car in MANUAL mode
			digitalWrite(mode_1, LOW);			//set the pins states
			digitalWrite(mode_2, HIGH);
			digitalWrite(multiplexor, LOW);		//now enable the main MCU control
			break;
		
		case SWITCH_POSITION_RESET:
			digitalWrite(multiplexor, HIGH);	//multiplexor HIGH puts car in MANUAL mode
			digitalWrite(mode_1, HIGH);			//set the pins states
			digitalWrite(mode_2, LOW);
			digitalWrite(multiplexor, LOW);		//now enable the main MCU control
			break;
		
		default:	//default state is SWITCH_POSITION_MANUAL
			digitalWrite(multiplexor, HIGH);	//multiplexor HIGH puts car in MANUAL mode
			digitalWrite(mode_1, HIGH);			//set the pins states
			digitalWrite(mode_2, HIGH);
			digitalWrite(multiplexor, HIGH);		//now enable the main MCU control
	}
	
	return;
}

void flash_led(){
	static long led_time_old = 0;
	
	switch(switch_position){
		case SWITCH_POSITION_WAYPOINT:
			if((millis() - led_time_old) > 250){
				digitalWrite(state_led, !digitalRead(state_led));
				led_time_old = millis();
			}
			break;

		case SWITCH_POSITION_RESET:
			for(int i = 0; i<3; i++){	//flashes the LED several times over ~5 seconds to indicate reset
				delay(250);
				digitalWrite(state_led, !digitalRead(state_led));
				delay(125);
				digitalWrite(state_led, !digitalRead(state_led));
				delay(125);
				digitalWrite(state_led, !digitalRead(state_led));
				delay(250);
				digitalWrite(state_led, !digitalRead(state_led));
			}
			break;

		case SWITCH_POSITION_AUTOMATIC:
			digitalWrite(state_led, HIGH);
			break;

		default:
			digitalWrite(state_led, LOW);   //default position of switch is MANUAL
			break;
	}
	
	return;
}

void check_if_reset_requested(){
	static long toggle_timer = millis();
	static int high_position_counter = 0;
	static int low_position_counter = 0;
	static long previous_pulse_length = 0;
	
	if(pulse_in_length >= PULSE_LENGTH_HIGH){	//check to see if ch3 is toggled HIGH
		if(previous_pulse_length >= PULSE_LENGTH_HIGH) ;	//check to see if the previous ch3 pulse was also HIGH. if yes, do nothing. if no, increment the counter.
		else{
			if(high_position_counter == 0) toggle_timer = millis();	//start the counter
			high_position_counter++;
			previous_pulse_length = pulse_in_length;
		}
	}

	if(pulse_in_length <= PULSE_LENGTH_LOW){	//checks to see if ch3 has just been toggled LOW
		if(previous_pulse_length <= PULSE_LENGTH_LOW) ;
		else{
			low_position_counter++;
			previous_pulse_length = pulse_in_length;
		}
	}
	
	if((millis() - toggle_timer) >= TIME_TO_FLIP_SWITCH){	//if reset time limit reached, reset the count, else reset the main MCU board
		high_position_counter = 0;
		low_position_counter = 0;
	}
	else if((high_position_counter > RESET_SWITCH_COUNTER) && (low_position_counter > RESET_SWITCH_COUNTER)){
		switch_position = SWITCH_POSITION_RESET;
		set_vehile_state();
		delay(1000);	//reset state will be held for ~2s (that should be enough time for the teensy to receive the command and reset itself...hopefully not before it is ready to receice another command to reset itself
		switch_position = SWITCH_POSITION_MANUAL;
		set_vehile_state();

		switch_position = SWITCH_POSITION_RESET;	//these 2 lines are only to display that RESET was actally sent
		flash_led();		

		switch_position = SWITCH_POSITION_MANUAL;
		flash_led();

		//DO NOT USE THIS CODE UNLESS YOU READ AND UNDERSTAND THE FOLLOWING!!!
		//The code below will toggle the RESET pin on the main MCU board. However, the teensy 3.1 reset pin is ***NOT*** 5V tolerant. Do ***NOT*** use this unless you drop to the correct voltage.
		// {
			// digitalWrite(hard_reset_pin, LOW);
			// delay(250);
			// digitalWrite(hard_reset_pin, HIGH);
		// }
	}
	
	return;
}