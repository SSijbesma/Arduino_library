#include <Arduino.h>
#include <GY_80.h>

GY_80::GY_80(void)
{
	BMP085  barometer;
	L3G gyro;
	ADXL345 acceleration;
	compass = HMC5883L();
	_init = false;
}

void GY_80::init()
{
int error;
	Wire.begin();
	barometer.begin();
	gyro.init();
	acceleration.begin();
	error = compass.SetScale(1.3);
	if(error != 0) // If there is an error, print it out.
		Serial.println(compass.GetErrorText(error));
	if(error == 0) // If there is an error, print it out.
		Serial.println("No error");
	error = compass.SetMeasurementMode(Measurement_Continuous);		
	_init = true;	
}

void GY_80::readAll()
{
	double Xg, Yg, Zg;
	
	if (!_init) {
		init();
	}
	temperature = barometer.readTemperature();
	pressure = barometer.readPressure();
	altitude = barometer.readAltitude();
	gyro.read();
	g_x = gyro.g.x;
	g_y = gyro.g.y;	
	g_z = gyro.g.z;
	acceleration.read(&Xg, &Yg, &Zg);
	acc_x = Xg;
	acc_y = Yg;
	acc_z = Zg;
	roll  = (atan2(-Yg, Zg)*180.0)/M_PI;
	pitch = (atan2(Xg, sqrt(Yg*Yg + Zg*Zg))*180.0)/M_PI;
	MagnetometerRaw raw = compass.ReadRawAxis();
    MagnetometerScaled scaled = compass.ReadScaledAxis();
	gausraw_X = raw.XAxis;
	gausraw_Y = raw.YAxis;
	gausraw_Z = raw.ZAxis;
	gausscaled_X = raw.XAxis;
	gausscaled_Y = raw.YAxis;
	gausscaled_Z = raw.ZAxis;
	headingrads = atan2(scaled.YAxis, scaled.XAxis);
	if(headingrads < 0)
      headingrads += 2*PI;
	if(headingrads > 2*PI)
		headingrads -= 2*PI;
	float headingDegrees = headingrads * 180/M_PI; 
}

void GY_80::readBarometer()
{
	if (!_init) {
		init();
	}
	temperature = barometer.readTemperature();
	pressure = barometer.readPressure();
	altitude = barometer.readAltitude();
}

void GY_80::readGyro()
{
	if (!_init) {
		init();
	}
	gyro.read();
	g_x = gyro.g.x;
	g_y = gyro.g.y;	
	g_z = gyro.g.z;
}

void GY_80::readAcceleration()
{
	double Xg, Yg, Zg;
	
	if (!_init) {
		init();
	}
	acceleration.read(&Xg, &Yg, &Zg);
	acc_x = Xg;
	acc_y = Yg;
	acc_z = Zg;
	roll  = (atan2(-Yg, Zg)*180.0)/M_PI;
	pitch = (atan2(Xg, sqrt(Yg*Yg + Zg*Zg))*180.0)/M_PI;
}

void GY_80::readCompass()
{
	if (!_init) {
		init();
	}
	MagnetometerRaw raw = compass.ReadRawAxis();
    MagnetometerScaled scaled = compass.ReadScaledAxis();
	gausraw_X = raw.XAxis;
	gausraw_Y = raw.YAxis;
	gausraw_Z = raw.ZAxis;
	gausscaled_X = raw.XAxis;
	gausscaled_Y = raw.YAxis;
	gausscaled_Z = raw.ZAxis;
	headingrads = atan2(scaled.YAxis, scaled.XAxis);
	if(headingrads < 0)
      headingrads += 2*PI;
	if(headingrads > 2*PI)
		headingrads -= 2*PI;
	float headingDegrees = headingrads * 180/M_PI; 
}

#define L3G4200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
#define L3G4200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3GD20_ADDRESS_SA0_LOW    (0xD4 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)

// Public Methods //////////////////////////////////////////////////////////////

bool L3G::init(byte device, byte sa0)
{
  _device = device;
  switch (_device)
  {
    case L3G4200D_DEVICE:
      if (sa0 == L3G_SA0_LOW)
      {
        address = L3G4200D_ADDRESS_SA0_LOW;
        return true;
      }
      else if (sa0 == L3G_SA0_HIGH)
      {
        address = L3G4200D_ADDRESS_SA0_HIGH;
        return true;
      }
      else
        return autoDetectAddress();
      break;
    case L3GD20_DEVICE:
      if (sa0 == L3G_SA0_LOW)
      {
        address = L3GD20_ADDRESS_SA0_LOW;
        return true;
      }
      else if (sa0 == L3G_SA0_HIGH)
      {
        address = L3GD20_ADDRESS_SA0_HIGH;
        return true;
      }
      else
        return autoDetectAddress();
      break;

    default:
      return autoDetectAddress();
  }
}

// Turns on the L3G's gyro and places it in normal mode.
void L3G::enableDefault(void)
{
  // 0x0F = 0b00001111
  // Normal power mode, all axes enabled
  writeReg(L3G_CTRL_REG1, 0x0F);
}

// Writes a gyro register
void L3G::writeReg(byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads a gyro register
byte L3G::readReg(byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G::read()
{
  Wire.beginTransmission(address);
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  Wire.write(L3G_OUT_X_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)6);
  while (Wire.available() < 6);
  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);
}

void L3G::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float L3G::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void L3G::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

bool L3G::autoDetectAddress(void)
{
  // try each possible address and stop if reading WHO_AM_I returns the expected response
  address = L3G4200D_ADDRESS_SA0_LOW;
  if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
  address = L3G4200D_ADDRESS_SA0_HIGH;
  if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
  address = L3GD20_ADDRESS_SA0_LOW;
  if (readReg(L3G_WHO_AM_I) == 0xD4) return true;
  address = L3GD20_ADDRESS_SA0_HIGH;
  if (readReg(L3G_WHO_AM_I) == 0xD4) return true;
  return false;
}

HMC5883L::HMC5883L()
{
  m_Scale = 1;
}

MagnetometerRaw HMC5883L::ReadRawAxis()
{
  uint8_t* buffer = Read(DataRegisterBegin, 6);
  MagnetometerRaw raw = MagnetometerRaw();
  raw.XAxis = (buffer[0] << 8) | buffer[1];
  raw.ZAxis = (buffer[2] << 8) | buffer[3];
  raw.YAxis = (buffer[4] << 8) | buffer[5];
  return raw;
}

MagnetometerScaled HMC5883L::ReadScaledAxis()
{
  MagnetometerRaw raw = ReadRawAxis();
  MagnetometerScaled scaled = MagnetometerScaled();
  scaled.XAxis = raw.XAxis * m_Scale;
  scaled.ZAxis = raw.ZAxis * m_Scale;
  scaled.YAxis = raw.YAxis * m_Scale;
  return scaled;
}

int HMC5883L::SetScale(float gauss)
{
	uint8_t regValue = 0x00;
	if(gauss == 0.88)
	{
		regValue = 0x00;
		m_Scale = 0.73;
	}
	else if(gauss == 1.30)
	{
  	    regValue = 0x01;
		m_Scale = 0.92;
	}
	else if(gauss == 1.9)
	{
		regValue = 0x02;
		m_Scale = 1.22;
	}
	else if(gauss == 2.5)
	{
		regValue = 0x03;
		m_Scale = 1.52;
	}
	else if(gauss == 4.0)
	{
		regValue = 0x04;
		m_Scale = 2.27;
	}
	else if(gauss == 4.7)
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(gauss == 5.6)
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(gauss == 8.1)
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	if (m_Scale == 1) {
		Serial.println(m_Scale);
		return ErrorCode_1_Num;
	}
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	Write(ConfigurationRegisterB, regValue);
	return 0;
}

int HMC5883L::SetMeasurementMode(uint8_t mode)
{
	Write(ModeRegister, mode);
	return 0;
}

void HMC5883L::Write(int address, int data)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t* HMC5883L::Read(int address, int length)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_Address);
  Wire.requestFrom(HMC5883L_Address, length);
  uint8_t buffer[length];
  if(Wire.available() == length)
  {
	  for(uint8_t i = 0; i < length; i++)
	  {
		  buffer[i] = Wire.read();
	  }
  }
  Wire.endTransmission();
  return buffer;
}

char* HMC5883L::GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return ErrorCode_1;
	return "Error not defined.";
}

int ADXL345::writeRegister(byte reg_addr, int nbytes, byte *buffer)
{
	int written_bytes;
	
	Wire.beginTransmission(ADXL345::ADDRESS);
	Wire.write(reg_addr);
	written_bytes = Wire.write(buffer, nbytes);
	Wire.endTransmission();
	return written_bytes;
}

int ADXL345::readRegister(byte reg_addr, int nbytes, byte *buffer)
{
	int idx = 0;

	Wire.beginTransmission(ADXL345::ADDRESS);
	Wire.write(reg_addr);
	Wire.endTransmission(); 
	Wire.requestFrom(ADXL345::ADDRESS, nbytes);
	while(Wire.available() && idx < nbytes)
	{ 
		buffer[idx++] = Wire.read();
	}
	return idx;
}

ADXL345::ADXL345()
{
	zG[0] = -20;
	zG[1] =  15;
	zG[2] = -23;
}

void ADXL345::begin()
{
	byte data = 0x08;
	writeRegister(ADXL345::POWER_CTL, 1, &data);
}

void ADXL345::end()
{
	byte data = 0x00;
	writeRegister(ADXL345::POWER_CTL, 1, &data);
}

void ADXL345::read(double *x, double *y, double *z)
{
	byte buffer[6];

	readRegister(ADXL345::DATAX0, 6, buffer);
	*x = ((buffer[0] + (buffer[1] << 8)) - zG[0])/256.0;
	*y = ((buffer[2] + (buffer[3] << 8)) - zG[1])/256.0;
	*z = ((buffer[4] + (buffer[5] << 8)) - zG[2])/256.0;
}

void ADXL345::read(int *x, int *y, int *z)
{
	byte buffer[6];

	readRegister(ADXL345::DATAX0, 6, buffer);
	
	*x = buffer[0] + (buffer[1] << 8);
	*y = buffer[2] + (buffer[3] << 8);
	*z = buffer[4] + (buffer[5] << 8);
}

void ADXL345::setRange(range_t range)
{	
	switch(range)
	{
		case RANGE_16G:
		case RANGE_8G:
		case RANGE_4G:
		case RANGE_2G:
			writeRegister(ADXL345::DATA_FORMAT, 1, (byte *)&range);
			break;
	}
}

void ADXL345::setZeroG(double x, double y, double z)
{
	zG[0] = x*256.0;
	zG[1] = y*256.0;
	zG[2] = z*256.0;
}

void ADXL345::setZeroG(int x, int y, int z)
{
	zG[0] = x;
	zG[1] = y;
	zG[2] = z;
}

BMP085::BMP085() {
}

boolean BMP085::begin(uint8_t mode) {
  if (mode > BMP085_ULTRAHIGHRES) 
    mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;
  if (read8(0xD0) != 0x55) return false;
  /* read calibration data */
  ac1 = read16(BMP085_CAL_AC1);
  ac2 = read16(BMP085_CAL_AC2);
  ac3 = read16(BMP085_CAL_AC3);
  ac4 = read16(BMP085_CAL_AC4);
  ac5 = read16(BMP085_CAL_AC5);
  ac6 = read16(BMP085_CAL_AC6);

  b1 = read16(BMP085_CAL_B1);
  b2 = read16(BMP085_CAL_B2);

  mb = read16(BMP085_CAL_MB);
  mc = read16(BMP085_CAL_MC);
  md = read16(BMP085_CAL_MD);
}

uint16_t BMP085::readRawTemperature(void) {
  write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  delay(5);
  return read16(BMP085_TEMPDATA);
}

uint32_t BMP085::readRawPressure(void) {
  uint32_t raw;

  write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER) 
    delay(5);
  else if (oversampling == BMP085_STANDARD) 
    delay(8);
  else if (oversampling == BMP085_HIGHRES) 
    delay(14);
  else 
    delay(26);

  raw = read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= read8(BMP085_PRESSUREDATA+2);
  raw >>= (8 - oversampling);

 /* this pull broke stuff, look at it later?
  if (oversampling==0) {
    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
  }
 */

  return raw;
}


int32_t BMP085::readPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();
  // do temperature calculations
  X1=(UT-(int32_t)(ac6))*((int32_t)(ac5))/pow(2,15);
  X2=((int32_t)mc*pow(2,11))/(X1+(int32_t)md);
  B5=X1 + X2;
  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;
  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );
  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + (int32_t)3791)>>4);
  return p;
}


float BMP085::readTemperature(void) {
  int32_t UT, X1, X2, B5;     // following ds convention
  float temp;

  UT = readRawTemperature();

  // step 1
  X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2,15);
  X2 = ((int32_t)mc * pow(2,11)) / (X1+(int32_t)md);
  B5 = X1 + X2;
  temp = (B5+8)/pow(2,4);
  temp /= 10;
  
  return temp;
}

float BMP085::readAltitude(float sealevelPressure) {
  float altitude;

  float pressure = readPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return altitude;
}


/*********************************************************************/

uint8_t BMP085::read8(uint8_t a) {
  uint8_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
#if (ARDUINO >= 100)
  ret = Wire.read(); // receive DATA
#else
  ret = Wire.receive(); // receive DATA
#endif
  Wire.endTransmission(); // end transmission

  return ret;
}

uint16_t BMP085::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
#if (ARDUINO >= 100)
  ret = Wire.read(); // receive DATA
  ret <<= 8;
  ret |= Wire.read(); // receive DATA
#else
  ret = Wire.receive(); // receive DATA
  ret <<= 8;
  ret |= Wire.receive(); // receive DATA
#endif
  Wire.endTransmission(); // end transmission

  return ret;
}

void BMP085::write8(uint8_t a, uint8_t d) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
  Wire.write(d);  // write data
#else
  Wire.send(a); // sends register address to read from
  Wire.send(d);  // write data
#endif
  Wire.endTransmission(); // end transmission
}
