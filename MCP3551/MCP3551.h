/******************************************************************************************************************************************************
 * Arduino MCP3551 library - Version 0.2
 * Copyright (c) 2014 Sybe Sijbesma.  All rights reserved.
 *
 * SPI communication using MCP3551/3 ADC devices in single mode conversion.
 *
 * This code is licensed under a GPLv3 License.
 *
 * Conversion is started using startConversion
 * WaitConversion waits for conversion to end signaled by MISO low
 *		errorCode = 0 good conversion
 *		errorCode = 1 waiting for conversion to end
 *		errorCode = 2 strange reading OVH and OVL ar both true
 *
 **********************************************************************************************/
#include <Arduino.h>
#include <pins_arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <stdlib.h>
#include <pgmspace.h>

#ifndef MCP3551_H
#define MCP3551_H

class MCP3551
{
	public:

	MCP3551 (uint8_t CSPIN);
	~MCP3551 ();
	void startConversion();			//start conversion in single conversion mode
	int waitConversion();			//wait for conversion to end
	boolean getOVH();				//Overload bit high
	boolean getOVL();				//Overload bit low
	int32_t adcValue;				//Resulting data

	private:

	unsigned long lastConversion;	//remember the last time we have polled the !CS Signal.
	uint8_t CSPin;					//pin connected to !CS
	boolean OVH, OVL;				//Overload bits high and low
};

#endif