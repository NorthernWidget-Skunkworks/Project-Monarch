//Dyson_Driver_ShortWave.ino

#include "Wire.h"
//Commands
#define CONF_CMD 0x00
#define ALS_CMD 0x04
#define WHITE_CMD 0x05

#define UVA_CMD 0x07
#define UVB_CMD 0x09
#define COMP1_CMD 0x0A
#define COMP2_CMD 0x0B

#define VIS_ADR 0x48
#define UV_ADR 0x10
#define ADC_ADR 0x49

//Compensation constants
float a = 1.92;
float b = 0.55;
float c = 2.46;
float d = 0.63;

unsigned int Config = 0; //Global config value


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}

uint8_t InitVEML(uint8_t Adr) 
{
	Wire.begin();  // Arduino Wire library initializer

    Wire.beginTransmission(Adr);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    return Wire.endTransmission(true); //Return sucess or failue of I2C connection

    //Setup gain and integration values manually! 
}

uint8_t InitADC() 
{
	//Setup ADC system 
}

float GetUV(uint8_t Sel) //Select A or B using Sel value (0 or 1) 
{
	Config = ReadByte(UV_ADR, CONF_CMD, 0);
	// long ConversionTime = (1 << ((Config & 0x70) >> 4))*50; //Calculate ms conversion time = (2^UV_IT) * 50
	// if((Config | 0x02) >> 1) { //Test single shot bit
	// 	while((millis() - StartTime) < ConversionTime);  //Wait if more time is needed
	// }
	//In either case, measurment process is the same 

	float Comp1 = ReadWord(UV_ADR, COMP1_CMD);
	float Comp2 = ReadWord(UV_ADR, COMP2_CMD);

	float UV = 0;
	float UV_Comp = 0;
	if(Sel == 0) {
		UV = ReadWord(UV_ADR, UVA_CMD);
		UV_Comp = UV - a*Comp1 - b*Comp2;
	}

	if(Sel == 1) {
		UV = ReadWord(UV_ADR, UVB_CMD);
		UV_Comp = UV - c*Comp1 - d*Comp2;
	}

	return UV_Comp;
}

// unsigned int GetALS() 
// {
// 	return ReadWord(VIS_ADR, ALS_CMD);
// }

unsigned int GetWhite() 
{
	return ReadWord(VIS_ADR, WHITE_CMD);
}

float GetLux() 
{	//Add non-linear correction! 
	// GetGain(); //Update global values
	// GetIntTime(); 
	float Gain = 0.125; //Hardcode max range for gain and int time 
	float IntTime = 25.0;  
	float Resolution = (1.8432/((float)IntTime/25.0))*(0.125/Gain);
	return ReadWord(VIS_ADR, ALS_CMD)*Resolution; //Return scaled Lux mesurment
}

uint8_t SendCommand(uint8_t Adr, uint8_t Command)
{
    Wire.beginTransmission(Adr);
    Wire.write(Command);
    return Wire.endTransmission(false);
}

uint8_t WriteWord(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
	Wire.beginTransmission(Adr);
	Wire.write(Command); //Write Command value
	Wire.write(Data & 0xFF); //Write LSB
	Wire.write((Data >> 8) & 0xFF); //Write MSB
	uint8_t Error = Wire.endTransmission();
	return Error;
}

uint8_t WriteConfig(uint8_t Adr, uint8_t NewConfig)
{
	Wire.beginTransmission(Adr);
	Wire.write(CONF_CMD);  //Write command code to Config register
	Wire.write(NewConfig);
	uint8_t Error = Wire.endTransmission();

	if(Error == 0) {
		Config = NewConfig; //Set global config if write was sucessful 
		return 0;
	}
	else return Error; //If write failed, return failure condition
}

int ReadByte(uint8_t Adr, uint8_t Command, uint8_t Pos) //Send command value, and high/low byte to read, returns desired byte
{
	uint8_t Error = SendCommand(Adr, Command);
	Wire.requestFrom(Adr, 2, true);
	uint8_t ValLow = Wire.read();
	uint8_t ValHigh = Wire.read();
	if(Error == 0) {
		if(Pos == 0) return ValLow;
		if(Pos == 1) return ValHigh;
	}
	else return -1; //Return error if read failed

}

int ReadWord(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
	uint8_t Error = SendCommand(Adr, Command);
	Wire.requestFrom(Adr, 2, true);
	uint8_t ByteLow = Wire.read();  //Read in high and low bytes (big endian)
	uint8_t ByteHigh = Wire.read();
	if(Error == 0) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
	else return -1; //Return error if read failed
}