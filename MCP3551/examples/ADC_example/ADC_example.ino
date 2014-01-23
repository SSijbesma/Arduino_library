/*
 This example demonstrate the usage of the A/D Converter with MCP3551 library.
 Connect SCK to CLK		   (D13 on Uno or Duemilanove, SCK for micro,  D13 on Nano)
 Connect SDO/!RDY to MOSI  (D12 on Uno or Duemilanove, MISO for micro, D12 on Nano)
 Connect !CS to MCPPIN     (D10 on Uno or Duemilanove, D10 for micro,  D10 on Nano)
*/
#include <MCP3551.h>
#include <SPI.h>

const int MCPPin = 10;

MCP3551 myADC(MCPPin); //Create an instance of MCP3551 connected to the MCPPIN

boolean bits[][8] = {
                     {LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH},
                     {LOW, LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH},
                     {LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH},
					 {LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH} 
					 };

int portSelect [4] = {5, 6, 7, 8};
double temp[5];
boolean firstTime = true;
int cycle = 0;

void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
int cnt, i;
boolean r;
int errorCode;

  for (cnt = 0; cnt < 5; cnt++) {	   
	for (i = 0; i < 4; i++) {
		digitalWrite(portSelect[i], bits[i][cnt]);
	}
	digitalWrite(portSelect[3], HIGH);
	myADC.startConversion();
	errorCode = myADC.waitConversion();
	if (errorCode == 0) {
		Serial.print(errorCode);
		Serial.print(" T");
  		Serial.print(cnt);
		Serial.print(":");
		Serial.print(myADC.adcValue * 0.0023842);
		if (cycle < 2) {
			temp[cnt] = myADC.adcValue * 0.0023842;} else {
			temp[cnt] = 0.9 * temp[cnt] + 0.1 * myADC.adcValue * 0.0023842;
			}
		Serial.print("  ");
		Serial.println(temp[cnt]);
		if (cnt == 4) {
			Serial.println();
			cycle++;
		}
	} else {
		Serial.println(errorCode);
	}
	digitalWrite(portSelect[3], bits[3][cnt]);
	delay(100);
//	digitalWrite(portSelect[3], HIGH);
  }
}



