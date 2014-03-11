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

void setup(){
	pinMode(rest_pin, OUTPUT);
	pinMode(ch_3_in, INPUT);
	pinMode(ch_3_state_1, OUTPUT);
	pinMode(ch_3_state_2, OUTPUT);
	pinMode(state_led, OUTPUT);
	pinMode(multiplexor, OUTPUT);

	digitalWrite(rest_pin, HIGH);

	state(SWITCH_STATE_MANUAL);
	
	val = digitalRead(inPin);   // read the input pin

	ioinit();	//sets up the arudino by adding interrupts, etc
}

void state(int switch_status){

	switch (switch_status) {
		case SWITCH_STATE_1:
			digitalWrite(ch_3_state_1, HIGH);




			break;
		case SWITCH_STATE_2:
			// statements
			break;
		case SWITCH_STATE_AUTOMATIC:
			// statements
			break;
		default:	//default state is SWITCH_STATE_MANUAL
			// statements
	}


}



void loop(){
	digitalWrite(ledPin, HIGH);   // sets the LED on
	delay(1000);                  // waits for a second
	digitalWrite(ledPin, LOW);    // sets the LED off
	delay(1000);                  // waits for a second
}

int channel_3_state(int ch_3_time){


}

ISR(INT0_vect) //This is the external interrupt 0
{
	if(REMOTE_CONTROL_REQUEST)
	{
		TCNT0 = 0;
		counter = 0;
	}
	else
	{
		pulse_length = (counter * 200) + TCNT0; //COnvert the useconds to miliseconds
		if((pulse_length < 2350)&&(pulse_length >= 800)) //Checking if got a valid pulse width 
		{
			valid_frames++;
		}
		else
		{
			valid_frames=0;
		}
	}
}

ISR(TIM0_COMPA_vect) //This is a timer interrupt, executed every 200us
{
	counter++;
	counter2++;
	timer_toggle++;

	if(counter2 >= 5)
	{
		counter2 = 0;

		if(timer_out >= LIMIT)
		timer_out = LIMIT;
		else
		timer_out++;
	}
}

void remote_reset(void){	//used to remotely reset the external MCU
//TRIGGER_AMOUNT = number of swichings required to reset the MCU
//LIMIT = time to make a valid transition. timer restarts if over the limit

	static unsigned char last_state = 0;
	static unsigned char rst_trigger = 0;

	if(last_state != state){	//check if we moved the switch
		if(timer_out < LIMIT){	//If true, check if timer is within limit
			rst_trigger++;		//Increment the trigger counter

			if(rst_trigger >= TRIGGER_AMOUNT){	//If yes, reset the external MCU
				MODE_LED_OFF;
				delay(RESET_TIME);	//Hold ATmega in reset for 250ms

				ATMEGA_RUN; //Release ATmega
				rst_trigger = 0;
			}  //If not do nothing

		}

		else //If the timer out overpass the timer, cancel everything..
		rst_trigger = 0;//Restarting the counter

		timer_out = 0; //Restarting vairable
		last_state = state; //Saving the last state

		}
}


#define MUX_ATMEGA_CONTROL (PORTB |= (1 << 1)) //PB01 used to control the multiplexor
#define MUX_RADIO_CONTROL (PORTB &= ~(1 << 1))

#define MODE_LED_ON (PORTB |= (1 << 3)) //PB03 mode (and extra autopilot state)...
#define MODE_LED_OFF (PORTB &= ~(1 << 3))

#define ATMEGA_RUN (PORTB |= (1 << 4)) //Must be keeped always HIGH, used to reset the "autopilot"..
#define ATMEGA_RESET (PORTB &= ~(1 << 4))

#define REMOTE_CONTROL_REQUEST (PINB & (1 << 2)) //Pin that read the RC signal (PWM)

#define LIMIT 300 //time
#define TRIGGER_AMOUNT 15 //Number of switches you need to do, to reset the Autopilot...
#define RESET_TIME 250 //Amount of time to hold the ATmega in reset, during a Autopilot reboot - In miliseconds

#define DIODE_TESTS_REPEATS 2

#define VALID_FRAMES_REQUIERED 75

unsigned char state = 0;
volatile unsigned int counter = 0;
volatile unsigned int counter2 = 0;
volatile unsigned int pulse_length = 0;
volatile unsigned int timer_out = 0;
volatile unsigned long timer_toggle=0;
volatile unsigned long valid_frames=0;

void ioinit(void);
void remote_reset(void);


int main(void)
{
	//Init the pins and hardware timers
	ioinit();



	// Setup an eternal loop
	while(1)
	{
		wdt_reset(); //Reseting WatchDog...

		if(valid_frames>=VALID_FRAMES_REQUIERED)//Verify if we have the requiered valid frames
		{
			valid_frames=VALID_FRAMES_REQUIERED; //Just void an overflow 
			
			if((pulse_length < 1350)&&(pulse_length >= 800))
			state = 0; //the state will be 0
			else
			{
				if((pulse_length > 1750)&&(pulse_length <= 2300)) 
				state = 1; //Enter to state 1
				else
				{
					state = 2;
				}
			}

			remote_reset();

		}
		else //Else activate fail mode... 
		{
			state = 1;
		}

		switch(state)//System state, you can change this as you desire
		{
		case 0: //This switch pulled down...
			MUX_RADIO_CONTROL; //Puts the pins low in state 0
			MODE_LED_OFF;
			timer_toggle=0;
			break;

		case 1: //This switch middle position
			if(timer_toggle>=2000)
			{
				MODE_LED_OFF;//Put just the mux pin HIGH
				MUX_ATMEGA_CONTROL;
				timer_toggle=3000; //Just void an overflow 
			}
			break;

		case 2: //Switch Up position...
			if(timer_toggle>=2000)
			{
				MODE_LED_ON; //Puts both pins high
				MUX_ATMEGA_CONTROL;
				timer_toggle=3000;//Just void an overflow 
			}
			break;
		}
	}//end of while(1)

}//end of int main(void)


void ioinit(void){   


	//Configuring the External interrupts
	MCUCR |= ((0<<ISC01)|(1<<ISC00)); //Please read page 52 of datasheet

	//Is configured to execute an interrupt every 200 useconds
	//This is the timer we going to use to measure the pulses lengths
	//This is timer #0 , see page 67 of data sheet.
	TCCR0A |= (1 << WGM01); //See page 81 and 82, activating CTC mode
	TCCR0B |=  (1 << CS01) ; //See page 82 and 83, using prescaler 8 =)
	OCR0A = 200; //prescaler 8/8mhz= 1us resolution, that means that
	//the interrupt will be generated every 200 useconds
	TIMSK |= (1 << OCIE0A); //See page 84, Enabling interrupt

		for(int x = 0 ; x < 4 ; x++) //Blinking LEDs
		{   
			wdt_reset();
			MODE_LED_ON;
			MUX_RADIO_CONTROL;
			_delay_ms(750);
			
			wdt_reset();
			MODE_LED_OFF;
			MUX_ATMEGA_CONTROL;
			_delay_ms(750);
			
		}
		
	MODE_LED_OFF;
	MUX_RADIO_CONTROL;
	
}

