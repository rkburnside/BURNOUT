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

#include <LiquidCrystal.h>
#include <Servo.h>
#include <EEPROM.h>

#define DEBUG 0  // debug state  1=cal gyro, 2=cal encoder, 3=watch angle, 4=read waypoints
#define GYRO_CAL 8650000 	// this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_TIME 1 // this is the interval in ms between gyro samples
#define GYRO_LIMIT 1000 // defines how many gyro samples are taken between angle calculations
#define MODE 5 // digital pin for mode select
#define TMISO 4 //digital pin for autopilot enable/disable
#define CLICK_MAX 3 //
#define SERVO_LIM 300 //
#define WP_SIZE 20 // number of bytes for each waypoint
// these are used for setting and clearing bits in special control registers on ATmega
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

LiquidCrystal lcd(A3, A4, A5, 8, 7, 6);
volatile boolean gyro_flag = false, encoder_flag = false, cal_flag;
boolean manual, automatic, aux=false;
volatile long gyro_sum = 0, gyro_count = 0, gyro_null=0, angle=0, clicks = 0;
unsigned long time;
long angle_last, angle_target, proximity, steer_us, angle_diff, Vcur, speed_old, speed_new;
double x_wp[10], y_wp[10];
double x=0, y=0;
//byte clicks = 0;
int temperature, click_temp;
int wpr_count=1, wpw_count=1, wp_total;
const int InterruptPin = 2 ;            	// intterupt on digital pin 2
Servo steering, esc;


template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
	const byte* p = (const byte*)(const void*)&value;
	int i;
	for (i = 0; i < sizeof(value); i++)
      EEPROM.write(ee++, *p++);
	return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
	byte* p = (byte*)(void*)&value;
	int i;
	for (i = 0; i < sizeof(value); i++)
      *p++ = EEPROM.read(ee++);
	return i;
}


struct position_structure {

    /* Using structures to contain location information. Will record old position
and new position. The actual structures will be position[0] and position[1]
but will use pointers old_pos and new_pos to access them. This way we can
simply swap the pointers instead of copying entire structure from one to the
other. Access data in the structures as follows: old_pos->x or new_pos->time, etc.
this is equivalent to (*old_pos).x.
*/

    double x;
    double y;
    //boolean last;
} waypoint;

void encoder_interrupt() {
    clicks++;
}

void calculate_parameters() {

	// calculate Speed
    speed_new = micros();
    Vcur = speed_new - speed_old;
	if (Vcur > 30000) Vcur = 30000;
    speed_old = speed_new;
	
	
	
	
	// calculate new position and update
    x += sin((angle + angle_last) * 3.14159/GYRO_CAL);
    y += cos((angle + angle_last) * 3.14159/GYRO_CAL);
	
	// find angle to new WP
    angle_last = angle;
    angle_target = atan2((x_wp[wpr_count] - x),(y_wp[wpr_count] - y)) * GYRO_CAL/2.0/3.14159;
    proximity = abs(x_wp[wpr_count]-x) + abs(y_wp[wpr_count]-y);
    // print stuff to LCD
    //lcd.clear();
    //lcd.print(x);
    //lcd.print(",");
    //lcd.print(y);

    angle_diff = angle_target - angle;
    if (angle_diff < -GYRO_CAL/2) angle_diff += GYRO_CAL;
    if (angle_diff > GYRO_CAL/2) angle_diff -= GYRO_CAL;
    //steer_us=map(angle_diff, -GYRO_CAL/2, GYRO_CAL/2, -SERVO_LIM, SERVO_LIM);
	//steer_us = (angle_diff + GYRO_CAL/2.0) * (SERVO_LIM + SERVO_LIM) / (GYRO_CAL/2.0 + GYRO_CAL/2.0) + SERVO_LIM;    
    steer_us = (float)angle_diff/GYRO_CAL*SERVO_LIM*4.0;
    //int temp =
    //if (steer_us < -
    
    //lcd.clear();
    //lcd.print(steer_us);
    steer_us += 1425;  //1407
    
    //if (proximity < 70) read_waypoint();
    if (proximity < 25) {
   	 wpr_count++;
   	 lcd.clear();
   	 lcd.print("read WP # ");
   	 lcd.print(wpr_count);
   	 lcd.setCursor(0, 1);
   	 lcd.print(x_wp[wpr_count]);
   	 lcd.print(" , ");
   	 lcd.print(y_wp[wpr_count]);
    }
    
    get_mode();
    if (automatic) steering.writeMicroseconds(steer_us);
    
    //temp5 = micros() - temp5;	// about 725 us for this loop!
    //lcd.clear();
    //lcd.print(temp5);

    //lcd.setCursor(0, 1);
    //lcd.print(angle_diff*360.0/GYRO_CAL);
    //lcd.print(" , ");
    //lcd.print(proximity);
}

void navigate_waypoint() {
    angle_diff = angle - angle_target;
    if (angle_diff < -GYRO_CAL/2) angle_diff += GYRO_CAL;
    if (angle_diff > GYRO_CAL/2) angle_diff -= GYRO_CAL;
    map(steer_us, -GYRO_CAL, GYRO_CAL, -SERVO_LIM, SERVO_LIM);
    steering.writeMicroseconds(steer_us);
}

void store_waypoint() {

}

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
    for (int i=0; i <= 50; i++){
   	 while (!gyro_flag);
   	 gyro_flag = false;  // start another round
    }
    gyro_null = angle/50;            	// calculate the null
    cal_flag = false;        	// stop calibration
    angle = 0;
    // should print null here
    lcd.clear();
    lcd.print(gyro_null);
    lcd.print("null");
    //while (true);
}

void calibrate_gyro() {
    lcd.clear();
    lcd.print("calibrating gyro");
    cal_flag = true;        	// tell ADC ISR that we are calibrating,
    gyro_flag = false;        	// this will be set, already, but need to begin on new cycle
    while (!gyro_flag);    	// wait for start of new cycle
    angle = 0;                	// reset the angle
    do {
   	 get_mode();
   	 lcd.clear();
   	 lcd.print(angle);
   	 delay(20);
    } while (aux);    	// keep summing unitil we turn the mode switch off. angle will  not roll-over
    cal_flag = false;        	// stop calibration
    // should print angle here
    lcd.clear();
    lcd.print("total angle is:");
    lcd.setCursor(0, 1);
    lcd.print(angle);
    angle = 0;
    gyro_count = 0;
    while (true);
}

void watch_angle() {
    lcd.clear();
    lcd.print("angle watch");
    do {
   	 get_mode();
   	 lcd.clear();
   	 lcd.print(angle*360.0/GYRO_CAL);
   	 delay(100);
    } while (aux);    	// keep summing unitil we turn the mode switch off.
}

void get_temp() {
    for (int i=0; i <= 500; i++){
   	 temperature += analogRead(1);
    }
    temperature /= 500;
}

void get_mode() {
    if (!digitalRead(TMISO)) {
   	 manual = true;
   	 automatic = false;
   	 aux = false;
    }
    else if (!digitalRead(MODE)) {
   	 manual = false;
   	 automatic = false;
   	 aux = true;
    }
    else {
   	 manual = false;
   	 automatic = true;
   	 aux = false;
    }
}

void set_gyro_adc() {
    //ADMUX should default to 000, which selects internal reference.
    ADMUX = B0;   //completely reset the MUX. should be sampling only on A0, now
    ADMUX |= (1 << REFS0);  // use internal ref, AVcc
    //this section sets the prescalar for the ADC. 111 = 128 = 9.6kSps, 011 = 64 = 19.2kSps, 101=38.4ksps
    ADCSRA |= (1 << ADPS0);  // set prescale bit 0
    ADCSRA |= (1 << ADPS1);  // set prescale bit 1
    ADCSRA |= (1 << ADPS2);  // set prescale bit 2
    //maybe try this instead:
    //ADCSRA |= B111;   // sets the prescalar 111=128, 110=64, 101=32, 100=16

    ADCSRA |= (1 << ADEN);  // Enable ADC
    ADCSRA |= (1 << ADATE); // Enable auto-triggering

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
    lcd.clear();
    lcd.print("I'm here!");
    while (true);
}

void set_waypoint() {
    waypoint.x = x;
    waypoint.y = y;
    //waypoint.last = false
    EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
    lcd.clear();
    lcd.print("set WP # ");
    lcd.print(wpw_count);
    lcd.setCursor(0, 1);
    lcd.print(waypoint.x);
    lcd.print(" , ");
    lcd.print(waypoint.y);
    wpw_count++;
    while(aux) get_mode();
}    

void load_waypoints() {
    int temp = 1;
    while (temp <= 10) {
   	 EEPROM_readAnything(temp*WP_SIZE, waypoint);
   	 x_wp[temp] = waypoint.x;
   	 y_wp[temp] = waypoint.y;
   	 temp++;
    }
}

void read_waypoint() {
    long temp = micros();
    EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
    //x_wp = waypoint.x;
    //y_wp = waypoint.y;
    //waypoint.last = false
    //EEPROM_writeAnything(wp_count*WP_SIZE, waypoint);
    lcd.clear();
    lcd.print("read WP # ");
    lcd.print(wpr_count);
    lcd.setCursor(0, 1);
    lcd.print(waypoint.x);
    lcd.print(" , ");
    lcd.print(waypoint.y);
    wpr_count++;
    lcd.clear();
    lcd.print(micros() - temp);
}    

void update_speed() {
	Verr = Vdes - (int)Vcur
	Vcmd += Kp * (float)Verr;	// Vcmd should have a range of 0 to 500. the null of apx. 1500 will be added later
	if (Vcmd > 500) Vcmd = 500;  //limit Vcmd to 500
	esc.writeMicroseconds(Vcmd + 1503); // add null and write
}	
	
	
void setup() {
    // Pin assignments:
    pinMode(TMISO, INPUT);
    pinMode(MODE, INPUT);
    lcd.begin(16, 2);	// set up the LCD's number of columns and rows:
    lcd.print("Minuteman!");    	// Print a message to the LCD.

    pinMode(InterruptPin, INPUT);	 
    attachInterrupt(0, encoder_interrupt, CHANGE);  // interrupt 0 is on digital pin 2
    //print_here();
    get_mode();
    //read_waypoint();
    //get_mode();
    load_waypoints();
    wpr_count = 1;
    get_temp();      	// this needs to be called before starting the gyro ADC
    set_gyro_adc();    	// sets up free running ADC for gyro
    calculate_null();
    steering.attach(10);
    esc.attach(11);
    steering.writeMicroseconds(1425);
    esc.writeMicroseconds(1503);
    wp_total = EEPROM.read(0);
    
}

void loop() {
    long temp;
    /* in the main loop here, we should wait for thing to happen, then act on them. Watch clicks and wait
for it to reach CLICK_MAX, then calculate position and such.
*/
    get_mode();
    //lcd.clear();
    //lcd.print(gyro_sum);
    //delay(1000);
    if (clicks >= CLICK_MAX) {
   	 clicks = 0;
   	 calculate_parameters();
   	 //click_temp++;
   	 //lcd.clear();
   	 //lcd.print(click_temp);
    }

/*/***** This is where the speed is set. "Channel[]" is used by the servo library. Channel[0], in particular,
	holds the number of servo that is currently being output. This is true, only if the servo library
	is using TIMER1, which is the default on the UNO. If you are using a different arduino or a different timer
	you may need change it. Servo pulses are generated on 20ms intervals. We only want to update the speed
	once per servo output. We can tell when a new cycle is starting by monitoring Channel[0]. It will be set
	to -1 at the beginning of the "refresh period" (the 20 ms wait between cycles) This is a good time to update
	the speed. Although it will create a nominal 20ms lag by updating at the beginning of the cycle, it is easier
	implement here, and the lag will be neglegible. */

	int temp2 = Channel[0];	  //we only want to read Channel[0] once, lest it change between reading (plausible?)
	if (temp2 < 0) {		// if it's negative, then we must be in the refresh period
		if (temp2 != temp_old) update_speed();  // but only update the speed if the previous result wasn't -1
		temp_old = temp2;
	}
	
	
//**** This is where the waypoint is set
    if (aux && DEBUG == 0) {
   	 temp = millis();
   	 while (aux) get_mode();
   	 temp = millis() - temp;
   	 if (temp > 500 && temp < 1500) set_waypoint();
   	 if (temp >= 1500) read_waypoint();
    }


//***** old debug stuff that probably needs to be phased out    
	
    if (aux && DEBUG == 1) calibrate_gyro();
    if (aux && DEBUG == 3) watch_angle();
    if (aux && DEBUG == 4) {
   	 temp = millis();
   	 while (aux) get_mode();
   	 read_waypoint();
    }
}
