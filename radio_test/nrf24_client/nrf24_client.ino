// nrf24_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_NRF24 class. RH_NRF24 class does not provide for addressing or
// reliability, so you should only use RH_NRF24 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example nrf24_server.
// Tested on Uno with Sparkfun NRF25L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module

#include <SPI.h>
#include <RH_NRF24.h>

// Singleton instance of the radio driver
//RH_NRF24 nrf24;
RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini


void setup() 
{
  Serial.begin(115200);
  Serial.setTimeout(10000);
  while (!Serial) 
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(8))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps , RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");    
  Serial.println("setup complete");    
}


void loop()
{
  char buff[32] = {0};
  int length = Serial.readBytesUntil('$', buff, 30);
  if (length > 0){
	Serial.print("Sending:" );
	Serial.println(length);
	Serial.println(buff);
	//buff[length] = '\0';
	nrf24.send((uint8_t *)buff, 28);
		// Send a message to nrf24_server
	//nrf24.waitPacketSent();
  // Now wait for a reply
	delay(50);
  }
}

