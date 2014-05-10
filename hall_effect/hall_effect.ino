//hall effect sensor check

int sensor = 10;    // variable to store the servo position 

void setup() 
{
	Serial.begin(115200);
} 

void loop() 
{
	Serial.println(digitalRead(sensor));
	delay(100);
} 
