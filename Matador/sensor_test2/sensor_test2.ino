
// pins for the LEDs:
const int close1 = 4;   //10 cm sensor
const int close2 = 5;   //10 cm sensor
const int far1 = 0;    //10-80 cm sensor
const int far2 = 1;    //10-80 cm sensor
int sensorValue = 0;        // value read from the pot
int outputValue = 0; 

void setup() {
  // initialize serial:
  Serial.begin(115200);    // start the serial port
  // make the pins outputs:
  pinMode(close1, INPUT);   // the close sensors are digital
  pinMode(close2, INPUT); 
}

void loop() {
  int val[4];
  val[0] = digitalRead(close1);
  val[1] = digitalRead(close2);
  val[2] = analogRead(far1);    // the far sensors are analog
  val[3] = analogRead(far2);
  
  
  
   sensorValue = analogRead(far1);            
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);  
  // change the analog out value:
  analogWrite(far1, outputValue);           

  // print the results to the serial monitor:
  Serial.print("sensor = " );                       
  Serial.print(sensorValue);      
  Serial.print("\t output = ");      
  Serial.println(outputValue);   
  
  
  
  
  Serial.print(val[0]);
  Serial.print("\t");      // this prints a tab
  Serial.print(val[1]);
  Serial.print("\t");
  Serial.print(val[2]);
  Serial.print("\t");
  Serial.println(val[3]);  //serial.println make a new line

  delay(200);   // wait 100 ms between loops
 }
