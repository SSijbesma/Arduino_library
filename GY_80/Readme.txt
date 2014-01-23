Device GY-80 combines a gyroscope, accelerometer, digital compass and a barometric pressure in one circuit.
These devices are:
- L3G4200D, 3 axis gyroscope from ST
- ADXL345,  3 axis accelerometer from Analog Devices
- HMC5883L, 3 axis digital compass from Honeywell
- BMP085,   Barometric Pressure and Temperature Sensor from Bosch

Though all sensors can be address individually using separate libraries, it is simpler to use a combined library.
GY_80.h combines the following header files:
- L3G.h for L3G4200D		Copyright (c) 2012 Pololu Corporation.		
- ADXL345.h for ADXL345		Copyright (C) 2012 Anil Motilal Mahtani Mirchandani(anil.mmm@gmail.com)
- HMC5883L.h for HMC588L.h	Copyright (C) 2011 Love Electronics (loveelectronics.co.uk) / 2012 bildr.org (Arduino 1.0 compatible)
- BMP085.h for BMP085		Written by Limor Fried/Ladyada for Adafruit Industries.  

GY_80.cpp combines all cpp files that correspond with the asbove mentioned header files.

A GY_80 class has been added that encapsulates all devices.

These files have been created by: Copyright (c) 2014 Sybe Sijbesma (Sybe@Sijbesma.org)

Function init() is used to initialise all devices.
If init() was not used before the first read action, it will be initialised
Function readAll() is used to retrieve all data measurements and stores the outcome in the corresponding variables defined within GY_80.
Function readBarometer() retrieves all data from BMP085
Function readGyro() retrieves all data from L3G
Function readAcceleration() retrieves all data from ADXL345 
Function readCompass() retrieves all data from HMC5883L

class GY_80
{
  public:
    GY_80(void);
	BMP085 barometer;
	L3G gyro;
	ADXL345 acceleration;
	HMC5883L compass;
	
    void init();				// inits all devices and sets _init = true
    void readAll();				//All devices
	void readBarometer();		//BMP085
	void readGyro();			//L3G
	void readAcceleration();	//ADXL345
	void readCompass();			//HMC5883L

	float temperature;
	float pressure;
	float altitude;
	float g_x;
	float g_y;
	float g_z;
	float acc_x;
	float acc_y;
	float acc_z;
	float pitch;
	float roll;
	
	float gausraw_X;
	float gausraw_Y;
	float gausraw_Z;

	float gausscaled_X;
	float gausscaled_Y;
	float gausscaled_Z;
	float headingrads;
	float headingDegrees;
  private:
    boolean _init;		// _init is set during init()
};