/* GYRINO 0.2

Pin Assignments:


A0 - Temperature
A1 - Gyro Angle
A2 - nc
A3 - nc
A4 - nc
A5 - nc
D0 - orange out
D1 - orange out
D2 - orange in
D3 - red led
D4 - blue led
D5 - unused jumper
D6 - nc
D7 - nc
D8 - yellow input, RX
D9 - nc
D10 - green/white ouput, TX
D11 - mosi
D12 - AZ (auto zero) miso
D13 - sck
*/
#define redPin 3
#define bluePin 4
#define AZPin 12
#define DEBUG 0  // debug state  1=cal gyro, 2=cal encoder, 3=watch angle, 4=read waypoints
//#define GYRO_CAL 8650000 	// this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_TIME 1 // this is the interval in ms between gyro samples
//#define GYRO_LIMIT 100 // defines how many gyro samples are taken between angle calculations
#define CLICK_MAX 3 //
// these are used for setting and clearing bits in special control registers on ATmega
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

volatile boolean gyro_flag = false, encoder_flag = false, cal_flag;
boolean manual, automatic, aux=false;
volatile long gyro_sum = 0, gyro_count = 0, gyro_null=0, angle=0, clicks = 0, GYRO_CAL = 8650000, GYRO_LIMIT = 500;
unsigned long time;
long angle_last, angle_target, proximity, steer_us, angle_diff, speed, speed_old, speed_new;
double x_wp[10], y_wp[10];
double x=0, y=0;
//byte clicks = 0;
int temperature, click_temp;
int wpr_count=1, wpw_count=1, wp_total;

const int clkaPin = 0;    // Master controls the clk
const int clkbPin = 1;  // Master controls the clk
const int clkPin = 2;    // Master controls the clk
const int txPin = 10;       // Master transmit pin
const int rxPin = 8;       // Master receive pin

//long angle = 0;
long limit = 0;
long deg = 0;
byte crc;

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
	set_gyro_adc();
	digitalWrite(txPin, HIGH);	// put master on-hold while doing null
    cal_flag = true;        	// tell ADC ISR that we are calibrating,
    gyro_flag = false;        	// this will be set, already, but need to begin on new cycle
    while (!gyro_flag) ;    	// wait for start of new cycle
    angle = 0;                	// reset the angle. angle will act as accumulator for null calculation
    gyro_null = 0;            	// make sure to not subract any nulls here
    for (int i=0; i <= 1; i++){
   	 while (!gyro_flag);
   	 gyro_flag = false;  		// start another round
    }
    gyro_null = angle/1;       // calculate the null
    cal_flag = false;        	// stop calibration
    angle = 0;
	digitalWrite(txPin, LOW);
	tx_bytes(gyro_null, 4);
	//crc = 0b10101010;
}

void calculate_null2() {
	digitalWrite(txPin, HIGH);
	delay(2000);
	digitalWrite(txPin, LOW);
	tx_bytes(12345, 4);
	crc = 0b10101010;
}

void calibrate_gyro() {
    cal_flag = true;        	// tell ADC ISR that we are calibrating,
    gyro_flag = false;        	// this will be set, already, but need to begin on new cycle
    while (!gyro_flag);    	// wait for start of new cycle
    angle = 0;                	// reset the angle
	while (digitalRead(rxPin));
    cal_flag = false;        	// stop calibration
	tx_bytes(angle, 4);
}

void get_temp() {
    for (int i=0; i <= 500; i++){
   	 temperature += analogRead(1);
    }
    temperature /= 500;
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

void tx_byte(byte mybyte) {
  for (int i=0; i < 8; i++) {
	while (!digitalRead(clkPin)) ;
    digitalWrite(txPin, bitRead(mybyte, i));
	while (digitalRead(clkPin));
  }
}

void wait_clk(){
	for (long i=0; i<64000; i++) {
		if (digitalRead(clkPin)) return;
	}
	wait_command();
}

byte rx_byte() {
	byte mybyte = 0;
	for (int i=0; i < 8; i++) {
		//wait_clk();
		while (!digitalRead(clkPin));
		bitWrite(mybyte, i, digitalRead(rxPin));
		while (digitalRead(clkPin));
	}
	return mybyte;
}

void request_angle() {
}

void ping() {
}

void reset() {
}

void reset_gyro() {
}

void tx_long(long temp) {
	for (byte i=0; i <4; i++) {
		tx_byte(lowByte(temp));
		temp = temp >> 8;
	}
}

void tx_bytes(long temp, byte bytes) {
	crc = 0;
	for (byte i=0; i < bytes; i++) {
		byte low = lowByte(temp);
		tx_byte(low);
		crc = crc ^ low;
		temp = temp >> 8;
	}
}

long rx_bytes(byte bytes) {
	byte temp[4];
	long output = 0;
	for (byte i=0; i < bytes; i++) {
		temp[i] = rx_byte();
	}
	for (byte i=1; i <= bytes; i++) {
		output = output << 8;
		output += temp[bytes - i];
	}
	return output;
}


void tx_int(int temp) {
	for (byte i=0; i <2; i++) {
		tx_byte(lowByte(temp));
		temp = temp >> 8;
	}
}

int rx_int() {  // receive an int 
}

void send_angle() {
	//angle = 1234567;
	tx_bytes(angle, 4);
}

void send_rand() {
	tx_bytes(random(1000), 4);
}

void send_angle_deg() {
	float temp = angle * 3600.0 / limit;  // this may be very slow. may be faster to use integer math
	tx_int((int)temp);
}

void send_temp() {
	noInterrupts();
	digitalWrite(txPin, HIGH);	// put master on-hold while doing null
	//delay(500);
	long temp = 0;
	for (int i = 1; i<50; i++) {
		temp += analogRead(0);
	}
	digitalWrite(txPin, LOW); // re-enable master
	tx_bytes(temp, 4);
}

void set_angle() {
	float temp = deg * 3600.0 * limit;
	angle = (int)temp;
}

void wait_command() {
	switch (rx_byte()) {
		case 'a':
			send_angle();
		break;
	
		case 'S':
			send_angle_deg();
		break;
	
		case 'n':
			calculate_null();
		break;
	
		case 'r':
			send_rand();
		break;
	
		case 's':
			set_angle();
		break;

		case 'c':
			calibrate_gyro();
		break;
		
		case 't':
			send_temp();
		break;
	}
	tx_byte(crc);	// send the acknowledge byte
}

void echo() {
	delay(700);
	while(true) {
		digitalWrite(redPin, HIGH);
		tx_byte(rx_byte());
		digitalWrite(redPin, LOW);
	}
}

void setup() {
	pinMode(bluePin, OUTPUT);
	digitalWrite(bluePin, LOW);
	pinMode(redPin, OUTPUT);
	digitalWrite(redPin, HIGH);
	pinMode(clkPin, INPUT);
	pinMode(clkaPin, INPUT);
	pinMode(clkbPin, INPUT);	
	pinMode(txPin, OUTPUT);
	pinMode(rxPin, INPUT);
	pinMode(AZPin, OUTPUT);
	digitalWrite(AZPin, LOW);
	set_gyro_adc();
    
}

void loop() {
	//echo();
	echo();
	wait_command();
}

