#include <GY_80.h>
#include <Wire.h>


GY_80 Device = GY_80();

void setup()
{
  /* add setup code here */
  Serial.begin(9600);
  Device.init();
}

void loop()
{
  /* add main program code here */
  Device.readAll();
  Serial.print(Device.temperature);	
  Serial.print('\t');
  Serial.print(Device.pressure);	
  Serial.print('\t');
  Serial.print(Device.altitude);	
  Serial.print('\t');
  Serial.print(Device.g_x);	
  Serial.print('\t');
  Serial.print(Device.g_y);	
  Serial.print('\t');
  Serial.print(Device.g_z);	
  Serial.print('\t');
  Serial.print(Device.acc_x);	
  Serial.print('\t');
  Serial.print(Device.acc_y);	
  Serial.print('\t');
  Serial.print(Device.acc_z);	
  Serial.print('\t');
  Serial.print(Device.gausscaled_X);	
  Serial.print('\t');
  Serial.print(Device.gausscaled_Y);	
  Serial.print('\t');
  Serial.println(Device.gausscaled_Z);	
}
