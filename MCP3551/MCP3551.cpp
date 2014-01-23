/******************************************************************************************************************************************************
 * Arduino MCP3551 library - Version 0.1
 *
 * Copyright (c) 2014 Sybe Sijbesma.  All rights reserved.
 *
 * SPI communication using MCP3551/3 ADC devices in single mode conversion.
 *
 * This code is licensed under a GPLv3 License.
 *
 * Conversion is started using startConversion(), no result
 * WaitConversion() waits for conversion to end, signaled by MISO low.
 *		result = 0 good conversion
 *		result = 1 timeout occurred (> 75 ms)
 *		result = 2 strange reading OVH and OVL, both are true
 *
 **********************************************************************************************/

#include <MCP3551.h>

MCP3551::MCP3551 (uint8_t CSPIN)
: CSPin(CSPIN) {
    pinMode(SCK, OUTPUT);
	pinMode(MISO, INPUT);
    digitalWrite(SCK, LOW);
	pinMode(CSPin,OUTPUT);
	digitalWrite(CSPin,HIGH);
	SPI.setClockDivider(SPI_CLOCK_DIV8); 
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	SPI.begin();
	};
MCP3551::~MCP3551 (){};


void MCP3551::startConversion()
{
	digitalWrite(CSPin,HIGH);  //enable device and start ADC conversion
	digitalWrite(CSPin,LOW);   //enable device and start ADC conversion
	digitalWrite(CSPin,HIGH);  //set single conversion mode
	digitalWrite(CSPin,LOW);   //enable device
	lastConversion = millis();
}

int MCP3551::waitConversion()
{
	union{
		int32_t adcResult;
		uint8_t aByte[4];
	} c ;
	
	while (true) {
		digitalWrite(CSPin, HIGH);
		digitalWrite(CSPin,LOW);											//enable device
		if ((millis() - lastConversion > 75) & (digitalRead(MISO) == 1)) {	//Timeout
			adcValue = 0;
			digitalWrite(CSPin,HIGH);										//disable the devices
			return 1;
		}
		if (digitalRead(MISO) == 0) {
			c.aByte[3] = 0x00;
			c.aByte[2] = SPI.transfer(0);
			c.aByte[1] = SPI.transfer(0);
			c.aByte[0] = SPI.transfer(0);
			OVH = ((c.aByte[2] & B01000000));
			OVL = ((c.aByte[2] & B10000000));
			if (OVH && OVL) {												// This should never occur
				adcValue = 0;
				return 2;
			}
			if (OVH && !OVL) c.aByte[2] &= B00111111;						// clear both flags
			adcValue = c.adcResult;
			digitalWrite(CSPin,HIGH);										//disable the devices
			return 0;
		} 
		delay(1);
	}
}

boolean MCP3551::getOVH()
{
	return OVH;
}

boolean MCP3551::getOVL()
{
	return OVL;
}
