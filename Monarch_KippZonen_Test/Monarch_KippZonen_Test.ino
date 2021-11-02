#include "Margay.h"
#include <MonarchSW.h>
#include "MCP3421.h"  //Inlcude ADC lib

MCP3421 CMP3(0x6B);  //Init with addres 0x6B

MonarchSW PyroUp(UP); //Initialzie Upward facing Monarch short wave

String Header = "Pyro [uV], "; //Information header
uint8_t I2CVals[3] = {0x40, 0x1D, 0x6B}; 
unsigned long UpdateRate = 15; //Number of seconds between readings 

Margay Logger(Model_1v0);

void setup() {
	Header = Header + PyroUp.GetHeader();
	Logger.begin(I2CVals, sizeof(I2CVals), Header); //Pass header info to logger
	Init();
}

void loop() {
	Logger.Run(Update, UpdateRate);
}

String Update() {
	Init(); //DEBUG!
	float Val1 = CMP3.GetVoltage()*(1.0e4); //Return val in uV, account for gain of amp (100 V/V)
	return String(Val1) + "," + PyroUp.GetString();
}

void Init() 
{
	PyroUp.begin();
	CMP3.Begin();
	CMP3.SetResolution(18); //Set for full 18 bit resolution
}
