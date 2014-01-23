#ifndef GY_80_h
#define GY_80_h

#include <Arduino.h>
#include <Wire.h>


/* BMP085 definitions */
#define BMP085_DEBUG 0
#define BMP085_I2CADDR 0x77
#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)
#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34


class BMP085 {
 public:
  BMP085();
  boolean begin(uint8_t mode = BMP085_ULTRAHIGHRES);  // by default go highres
  float readTemperature(void);
  int32_t readPressure(void);
  float readAltitude(float sealevelPressure = 101325); // std atmosphere
  uint16_t readRawTemperature(void);
  uint32_t readRawPressure(void);
  
 private:
  uint8_t read8(uint8_t addr);
  uint16_t read16(uint8_t addr);
  void write8(uint8_t addr, uint8_t data);

  uint8_t oversampling;

  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;
};


/* ADXL345 definitions */
class ADXL345
{
	private:
		/* ADXL345 Registers */
		enum info_t 
		{
			DEVID          = 0x00,
			THRESH_TAP     = 0x1D,
			OFSX           = 0x1E,
			OFSY           = 0x1F,
			OFSZ           = 0x20,
			DUR            = 0x21,
			Latent         = 0x22,
			Window         = 0x23,
			THRESH_ACT     = 0x24,
			THRESH_INACT   = 0x25,
			TIME_INACT     = 0x26,
			ACT_INACT_CTL  = 0x27,
			THRESH_FF      = 0x28,
			TIME_FF        = 0x29,
			TAP_AXES       = 0x2A,
			ACT_TAP_STATUS = 0x2B,
			BW_RATE        = 0x2C,
			POWER_CTL      = 0x2D,
			INT_ENABLE     = 0x2E,
			INT_MAP        = 0x2F,
			INT_SOURCE     = 0x30,
			DATA_FORMAT    = 0x31,
			DATAX0         = 0x32,
			DATAX1         = 0x33,
			DATAY0         = 0x34,
			DATAY1         = 0x35,
			DATAZ0         = 0x36,
			DATAZ1         = 0x37,
			FIFO_CTL       = 0x38,
			FIFO_STATUS    = 0x39,
			ADDRESS        = 0xA7 >> 1
		};


		int zG[3];
		int readRegister(byte reg_addr, int nbytes, byte *buffer);
		int writeRegister(byte reg_addr, int nbytes, byte *buffer);
	
	public:
		enum range_t
		{
			RANGE_16G = 0x0B,
			RANGE_8G  = 0x0A,
			RANGE_4G  = 0x09,
			RANGE_2G  = 0x08
		};

		ADXL345();
	
		void begin();
		void end();
	
		//G Reading
		void read(double *x, double *y, double *z);
	
		//Raw reading
		void read(int *x, int *y, int *z);
	
		//Unit must be res_t
		void setRange(range_t range);

		//Unit must be G
		void setZeroG(double x, double y, double z);
	
		//Unit must be "Raw"
		void setZeroG(int x, int y, int z);
};


/* HMC5883L definitions */

#define HMC5883L_Address 0x1E
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03

#define ErrorCode_1 "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1"
#define ErrorCode_1_Num 1

struct MagnetometerScaled
{
	float XAxis;
	float YAxis;
	float ZAxis;
};

struct MagnetometerRaw
{
	int XAxis;
	int YAxis;
	int ZAxis;
};

class HMC5883L
{
	public:
	  HMC5883L();

	  MagnetometerRaw ReadRawAxis();
	  MagnetometerScaled ReadScaledAxis();
  
	  int SetMeasurementMode(uint8_t mode);
	  int SetScale(float gauss);

	  char* GetErrorText(int errorCode);

	protected:
	  void Write(int address, int byte);
	  uint8_t* Read(int address, int length);

	private:
	  float m_Scale;
};

/*L3G definitions */
#define L3G_DEVICE_AUTO 0
#define L3G4200D_DEVICE 1
#define L3GD20_DEVICE   2
// SA0 states
#define L3G_SA0_LOW  0
#define L3G_SA0_HIGH 1
#define L3G_SA0_AUTO 2
// register addresses
#define L3G_WHO_AM_I      0x0F
#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27
#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D
#define L3G_FIFO_CTRL_REG 0x2E
#define L3G_FIFO_SRC_REG  0x2F
#define L3G_INT1_CFG      0x30
#define L3G_INT1_SRC      0x31
#define L3G_INT1_THS_XH   0x32
#define L3G_INT1_THS_XL   0x33
#define L3G_INT1_THS_YH   0x34
#define L3G_INT1_THS_YL   0x35
#define L3G_INT1_THS_ZH   0x36
#define L3G_INT1_THS_ZL   0x37
#define L3G_INT1_DURATION 0x38

class L3G
{
  public:
    typedef struct vector
    {
      float x, y, z;
    } vector;

    vector g; // gyro angular velocity readings

    bool init(byte device = L3G_DEVICE_AUTO, byte sa0 = L3G_SA0_AUTO);

    void enableDefault(void);

    void writeReg(byte reg, byte value);
    byte readReg(byte reg);

    void read(void);

    // vector functions
    static void vector_cross(const vector *a, const vector *b, vector *out);
    static float vector_dot(const vector *a,const vector *b);
    static void vector_normalize(vector *a);

  private:
      byte _device; // chip type (4200D or D20)
      byte address;

      bool autoDetectAddress(void);
};


class GY_80
{
  public:
    GY_80(void);
	BMP085 barometer;
	L3G gyro;
	ADXL345 acceleration;
	HMC5883L compass;
	
    void init();
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
    boolean _init;	
};

#endif