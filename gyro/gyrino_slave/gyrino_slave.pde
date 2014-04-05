const int  clkPin = 2;    // Master controls the clk
const int txPin = 10;       // Master transmit pin
const int rxPin = 8;       // Master receive pin

long angle = 0;
long limit = 0;
long deg = 0;
byte crc;

// communications should be sent LSbit and LSByte first

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
	angle = 1234567;
	tx_bytes(angle, 4);
}

void send_rand() {
	tx_bytes(random(1000), 4);
}

void send_angle_deg() {
	float temp = angle * 3600.0 / limit;  // this may be very slow. may be faster to use integer math
	tx_int((int)temp);
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
	
		case 'r':
			send_rand();
		break;
	
		case 's':
			set_angle();
		break;
	}
	tx_byte(crc);	// send the acknowledge byte
}

void echo() {
	delay(700);
	while(true) {
		tx_byte(rx_byte());
	}
}

void interpret() {
}	

void setup()  {
  //Serial.begin(9600);

  // define pin modes for tx, rx, led pins:
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(clkPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);
}

void loop() {
	echo();
	//wait_command();
}