#define mydelay  delayMicroseconds(100);  //50 works well with at 16MHz
const int  clkPin = 3;    // Master controls the clk
const int txPin = 4;       // Master transmit pin
const int rxPin = 5;       // Master receive pin
byte crc = 0;

void tx_byte(byte mybyte) {
  digitalWrite(clkPin, LOW);
  mydelay
  for (int i=0; i < 8; i++) { 
    digitalWrite(txPin, bitRead(mybyte, i));
	mydelay
    digitalWrite(clkPin, HIGH);
	mydelay
	digitalWrite(clkPin, LOW);
	mydelay
  }
}

byte rx_byte() {
	byte mybyte = 0;
	digitalWrite(clkPin, LOW);
	mydelay
	for (int i=0; i < 8; i++) {
		digitalWrite(clkPin, HIGH);
		mydelay
		bitWrite(mybyte, i, digitalRead(rxPin));
		mydelay
		digitalWrite(clkPin, LOW);
		mydelay
	}	
	return mybyte;
}

void tx_bytes(long temp, byte bytes) {
	for (byte i=0; i < bytes; i++) {
		tx_byte(lowByte(temp));
		temp = temp >> 8;
	}
}

long rx_bytes(byte bytes) {
	byte temp[4];
	long output = 0;
	crc = 0;
	for (byte i=0; i < bytes; i++) {
		temp[i] = rx_byte();
		crc = crc ^ temp[i];
	}
	for (byte i=1; i <= bytes; i++) {
		output = output << 8;
		output += temp[bytes - i];
	}
	return output;
}

void test_com() {
	for (byte i = 'a'; i < 'z'; i++) {
		tx_byte(i);
		Serial.print(i);
		Serial.print(rx_byte());
		mydelay
	}
	Serial.println(' ');
}

long get_angle() {
	tx_byte('a');
	long temp = rx_bytes(4);
	//return temp;
	if (rx_byte() == crc) return temp;
	else return 999999;    //need to come up with a number to return that won't be an angle
}

long get_rand() {
	tx_byte('r');
	long temp = rx_bytes(4);
	//return temp;
	if (rx_byte() == crc) return temp;
	else return 999999;    //need to come up with a number to return that won't be an angle
}

long calculate_null() {
	tx_byte('n');
	mydelay
	while(digitalRead(rxPin));
	long temp = rx_bytes(4);
	//return temp;
	//if (rx_byte() == 0b10101010) return temp;
	if (rx_byte() == crc) return temp;
	else return 999999;    //need to come up with a number to return that won't be an angle
}

long get_temp() {
	tx_byte('t');
	mydelay
	while(digitalRead(rxPin));
	long temp = rx_bytes(4);
	//return temp;
	//if (rx_byte() == 0b10101010) return temp;
	if (rx_byte() == crc) return temp;
	else return 999999;    //need to come up with a number to return that won't be an angle
}

long calibrate_gyro() {
	tx_byte('c');
	digitalWrite(txPin, HIGH);
	Serial.println("calibrating gyro");
	Serial.println("press any key to finish");
	Serial.flush();
	while(Serial.available() == 0);
	digitalWrite(txPin, LOW);
	mydelay
	long temp = rx_bytes(4);
	if (rx_byte() == crc) return temp;
	else return 999999;    //need to come up with a number to return that won't be an angle
}

long set_calibration() {
	byte temp[20];
	Serial.println("enter a calibration value: ");
	tx_byte('c');
        mydelay
        tx_byte(get_num());
        mydelay
}

long get_num() {
	char num[20];
	byte tmp;
	Serial.flush();
	for (byte i = 0; i < 20; i++) {
		while (Serial.available() == 0);
		tmp = Serial.read();
		Serial.print(tmp);
		if (tmp == '\r') return atol(num);
		num[i] = tmp;
		num[i+1] = 0;
	}
	return 0;
}

void test_send() {
	while (true) {
		long number = random(-30000, 30000);
		Serial.print(number);
		Serial.print(", ");
		tx_bytes(number, 4);
		long whatever = rx_bytes(4);
		if (rx_byte() != crc) whatever = 99999;
		Serial.println(whatever);
		delay(500);
	}
}

void ping() {
}

void reset() {
}

void menu() {
	while (true) {
	//test_com();
		Serial.println("Select option:");
		Serial.println("n: Calculate Null");
		Serial.println("a: Read Angle");
		Serial.println("c: Calibrate Gyro");
		Serial.println("s: Set Calibration Value");
		Serial.println("z: toggle auto zero");
		Serial.flush();
		while (Serial.available() == 0);
		switch(Serial.read()) {
			case 'n':
				Serial.println(calculate_null());
				break;
			case 'c':
				Serial.println(calibrate_gyro());
				break;
			case 's':
				Serial.println(set_calibration());
				break;
			case 'a':
				Serial.println(get_angle());
				break;
		}
		
	}
}

void setup()  {
	Serial.begin(9600);

	// define pin modes for tx, rx, led pins:
	pinMode(clkPin, OUTPUT);
	pinMode(txPin, OUTPUT);
	pinMode(rxPin, INPUT);
}

void loop() {
	test_send();
	long temp = get_rand();
	long null = get_rand();
	delay(1000);
	Serial.print(temp);
	Serial.print(", ");
	Serial.println(null);
}
