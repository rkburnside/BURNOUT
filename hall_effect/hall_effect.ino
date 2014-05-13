//hall effect sensor check

int	click_calibration_counter = 0;

void setup(){
	attachInterrupt(0, click_calibration_increment, CHANGE);	//according to the teensy documentation, all pins can be interrupts

	Serial.begin(115200);
} 

void loop(){
} 

void click_calibration_increment(){
	click_calibration_counter++;
	Serial.print("ebay_sensor: ");
	Serial.println(click_calibration_counter);
	return ;
}