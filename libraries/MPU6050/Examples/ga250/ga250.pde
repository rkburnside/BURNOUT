/* MM 0.2

attempting to add multi-waypoint functionality

This is the final release of version 0.1. At this point the car
will steer to a pre-defined waypoint under manual throttle.

The goal of this release is to test the functionality of the
Encoder and gyro pair. I will have the car drive forward under
manual control, and the system will update the x,y position as
it goes.
Pin Assignments:


A0 - Analog input from gyro
A1 - analog input for temp, from gyro
A2 - nc
A3 - LCD  (RS?)
A4 - LCD  (enable?)
A5 - LCD
D0 - RX
D1 - TX
D2 - nc (normally Ch1 in)
D3 - Steering input (connected to Ch2 in on board)
D4 - MUX enble input (input only. manual if low, auto if high)
D5 - Mode input (switched by Ch3)
D6 - LCD
D7 - LCD
D8 - LCD
D9 - nc (internally connected to MUX 1)  **consider connecting to MUX 3
D10 - Steering contorl out (internally connected to MUX 2
D11 - ESC control out (connect to MUX 3)
D12 - LED status
D13 - LED status
*/
#include <SoftwareSerial.h>
#define rxPin 8
#define txPin 10
#define ledPin 3
#define AZPin 12
#define DEBUG 0  // debug state  1=cal gyro, 2=cal encoder, 3=watch angle, 4=read waypoints
#define GYRO_CAL 8650000 	// this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_TIME 1 // this is the interval in ms between gyro samples
#define GYRO_LIMIT 100000 // defines how many gyro samples are taken between angle calculations
#define CLICK_MAX 3 //
// these are used for setting and clearing bits in special control registers on ATmega
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

volatile boolean gyro_flag = false, encoder_flag = false, cal_flag;
boolean manual, automatic, aux=false;
volatile long gyro_sum = 0, gyro_count = 0, gyro_null=0, angle=0, clicks = 0;
unsigned long time;
long angle_last, angle_target, proximity, steer_us, angle_diff, speed, speed_old, speed_new;
double x_wp[10], y_wp[10];
double x=0, y=0;
//byte clicks = 0;
int temperature, click_temp;
int wpr_count=1, wpw_count=1, wp_total;

byte muxtemp;
byte adtemp;



// set up a new serial port
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

ISR(ADC_vect) {    	//ADC interrupt

    uint8_t high,low;	// I think uint8_t is the same as byte.

    //	low = ADCL;    	//Make certain to read ADCL first, it locks the values
    //	high = ADCH;    	//and ADCH releases them.

    //	aval = (high << 8) | low;

    /* for further brevity at cost of clarity

    aval = ADCL | (ADCH << 8);

also seems to work  	*/
    gyro_sum += ADCL | (ADCH << 8);
    //	adcbin = adcbin + aval;	//accumulate the ADC values
    gyro_count++;    	//iterate the counter
    if (gyro_count == GYRO_LIMIT) {
   	 angle += (gyro_sum - gyro_null);
   	 if ((angle > GYRO_CAL) && (!cal_flag)) angle -= GYRO_CAL; //if we are calculating null, don't roll-over
   	 if ((angle < 0) && (!cal_flag)) angle += GYRO_CAL;
   	 gyro_sum = 0;
   	 gyro_count =0;
   	 gyro_flag = true;
    }

}  

void calculate_null() {

    cal_flag = true;        	// tell ADC ISR that we are calibrating,
    gyro_flag = false;        	// this will be set, already, but need to begin on new cycle
    while (!gyro_flag) ;    	// wait for start of new cycle
    angle = 0;                	// reset the angle. angle will act as accumulator for null calculation
    gyro_null = 0;            	// make sure to not subract any nulls here
    for (int i=0; i <= 1; i++){
   	 while (!gyro_flag);
   	 gyro_flag = false;  // start another round
    }
    gyro_null = angle/1;            	// calculate the null
    cal_flag = false;        	// stop calibration
    angle = 0;
    // should print null here
    //while (true);
}

void calibrate_gyro() {
/*    lcd.clear();
    lcd.print("calibrating gyro"); */
    cal_flag = true;        	// tell ADC ISR that we are calibrating,
    gyro_flag = false;        	// this will be set, already, but need to begin on new cycle
    while (!gyro_flag);    	// wait for start of new cycle
    angle = 0;                	// reset the angle
/*    do {
   	 get_mode();
   	 lcd.clear();
   	 lcd.print(angle);
   	 delay(20);
    } while (aux);  */  	// keep summing unitil we turn the mode switch off. angle will  not roll-over
    cal_flag = false;        	// stop calibration
    // should print angle here
/*    lcd.clear();
    lcd.print("total angle is:");
    lcd.setCursor(0, 1);
    lcd.print(angle); */
    angle = 0;
    gyro_count = 0;
    while (true);
}

void watch_angle() {
/*    lcd.clear();
    lcd.print("angle watch");
    do {
   	 get_mode();
   	 lcd.clear();
   	 lcd.print(angle*360.0/GYRO_CAL);
   	 delay(100);
    } while (aux);    	// keep summing unitil we turn the mode switch off. */
}

long get_temp() {
	ADMUX =	muxtemp;
	ADCSRA = adtemp;
	long temp = 0;
    for (int i=0; i <= 5; i++){
   	 temp += analogRead(0);
    }
    return temp;
}

void set_gyro_adc() {
    //ADMUX should default to 000, which selects internal reference.
    ADMUX = B0;   //completely reset the MUX. should be sampling only on A0, now
    ADMUX |= (1 << MUX0);  // use internal Use ADC1
    //this section sets the prescalar for the ADC. 111 = 128 = 9.6kSps, 011 = 64 = 19.2kSps, 101=38.4ksps
    ADCSRA = B0;   //  
	ADCSRA |= B101;   // sets the prescalar 111=256, 110=128, 101=64, 100=32

    ADCSRA |= (1 << ADEN);  // Enable ADC
    ADCSRA |= (1 << ADFR); // Enable auto-triggering

    ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
    sei();                 	// Enable Global Interrupts
    ADCSRA |= (1 << ADSC);  // Start A2D Conversions

    /* alternatively, use:  (not tested yet)
    sbi(ADCSRA, ADPS0);
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS2);
    sbi(ADCSRA, ADEN);
    sbi(ADCSRA, ADATE);
    sbi(ADCSRA, ADIE);
    sei();
    sbi(ADCSRA, ADSC);
    */
}

void print_here() {
    mySerial.print("I'm here!");
    while (true);
}

void setup() {
    // Pin assignments:
//    get_temp();      	// this needs to be called before starting the gyro ADC
//    set_gyro_adc();    	// sets up free running ADC for gyro
//    calculate_null();
	// define pin modes for tx, rx, led pins:
	muxtemp = ADMUX;
	adtemp = ADCSRA;
	pinMode(rxPin, INPUT);
	pinMode(txPin, OUTPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(AZPin, OUTPUT);
	digitalWrite(AZPin, HIGH);
	// set the data rate for the SoftwareSerial port
	mySerial.begin(4800);
	delay(500);
	mySerial.println("Hello World - SoftwareSerial");
	pinMode(0, INPUT);
	pinMode(1, OUTPUT);
	pinMode(2, INPUT);
	Serial.begin(300);
	delay(500);
	Serial.println("testing...");
    
}

void loop() {
	set_gyro_adc();
	calculate_null();
	cli();
	mySerial.print(gyro_null);
	mySerial.print(", ");
	mySerial.println(get_temp());

	delay(200);
	
	//while (true) ;
}
