//Dyson_Driver_ShortWave.ino
#include "SlowSoftI2CMaster.h"
#include "WireS.h"
//Commands
#define CONF_CMD 0x00
#define ALS_CMD 0x04
#define WHITE_CMD 0x05

#define UVA_CMD 0x07
#define UVB_CMD 0x09
#define COMP1_CMD 0x0A
#define COMP2_CMD 0x0B

#define ADC_CONF 0x01
#define ADC_CONV 0x00
#define ADC0 0x4200
#define ADC1 0x5200
#define ADC2 0x6200
#define ADC3 0x7200

#define VIS_ADR 0x48
#define UV_ADR 0x10
#define ADC_ADR 0x49

#define READ 0x01
#define WRITE 0x00

#define BUF_LENGTH 64 //Length of I2C Buffer, verify with documentation 

//Compensation constants
float a = 1.92;
float b = 0.55;
float c = 2.46;
float d = 0.63;

volatile uint8_t ADR = 0x40; //Use arbitraty address, change using generall call??

unsigned int Config = 0; //Global config value

uint8_t Reg[26] = {0}; //Initialize registers
bool StartSample = false; //Flag used to start a new converstion 
const unsigned int UpdateRate = 5; //Rate of update

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_B2, PIN_A7, true);  //Initialize software I2C

volatile bool StopFlag = false; //Used to indicate a stop condition 
volatile uint8_t RegID = 0; //Used to denote which register will be read from
volatile bool RepeatedStart = false; //Used to show if the start was repeated or not

void setup() {
  // Serial.begin(115200); //DEBUG!
  // Serial.println("begin"); //DEBUG!
  Wire.begin(ADR);  //Begin slave I2C
  InitVEML(0x48); //Init Vis (VEML6030)
  InitVEML(0x10); //Init UV (VEML6075)
  InitADC(); //Init ADC (ADS1115)

  //Setup I2C slave
	Wire.onAddrReceive(addressEvent); // register event
	Wire.onRequest(requestEvent);     // register event
	Wire.onReceive(receiveEvent);
	Wire.onStop(stopEvent);

  si.i2c_init(); //Begin I2C master
}

void loop() {
	static unsigned int Count = 0; //Counter to determine update rate
	if(StartSample == true) {
		//Read new values in

		SplitAndLoad(0x0B, GetALS()); //Load ALS value
		SplitAndLoad(0x0D, GetWhite()); //Load white value
		SplitAndLoad(0x02, long(GetUV(0))); //Load UVA
		SplitAndLoad(0x07, long(GetUV(1))); //Load UVB
		SplitAndLoad(0x10, GetLuxGain()); //Load lux multiplier 
		SplitAndLoad(0x13, GetADC(0));
		SplitAndLoad(0x15, GetADC(1));
		SplitAndLoad(0x17, GetADC(2));

		StartSample = false; //Clear flag when new values updated  
	}
	if(Count++ == UpdateRate) {  //Fix update method??
		StartSample = true; //Set flag if number of updates have rolled over 
		Count = 0;
	}
	delay(100);
}

uint8_t InitVEML(uint8_t Adr) 
{
	uint8_t CMD = 0; 
	if(Adr = 0x48) CMD = 0x23;
	else CMD = 0;
    si.i2c_start((Adr << 1) | WRITE);
    si.i2c_write(0x00);
    si.i2c_write(0x00);
    si.i2c_write(CMD);
    si.i2c_stop(); 

    return 0; //Fix dummy
    //Setup gain and integration values manually! 
}

uint8_t InitADC() 
{
	//Setup ADC system 
	WriteWord_LE(ADC_ADR, ADC_CONF, ADC0); //Set to single shot mode with 
}

unsigned int GetADC(unsigned int Num)
{
	unsigned int ADC_Config = ADC0 | (Num << 12); //Use to select which ADC to get data from
	WriteWord_LE(ADC_ADR, ADC_CONF, ADC_Config); //Setup registers
	delay(300);  //Wait for next sample to be read
	return ReadWord_LE(ADC_ADR, ADC_CONV); //Read from register
}

float GetUV(uint8_t Sel) //Select A or B using Sel value (0 or 1) 
{
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

unsigned int GetALS() 
{
	return ReadWord(VIS_ADR, ALS_CMD);
}

unsigned int GetWhite() 
{
	return ReadWord(VIS_ADR, WHITE_CMD);
}

unsigned int GetLuxGain() 
{	//Add non-linear correction! 
	float Gain = 0.125; //Hardcode max range for gain and int time 
	float IntTime = 25.0;  
	float Resolution = (1.8432/((float)IntTime/25.0))*(0.125/Gain);
	// unsigned int Resolution = (512/(IntTime/25.0))/(Gain/0.125);
	return int(Resolution/0.0036); //Return Lux scaler
}



uint8_t SendCommand(uint8_t Adr, uint8_t Command)
{
    si.i2c_start((Adr << 1) | WRITE);
    bool Error = si.i2c_write(Command);
    return 1; //DEBUG!
}

uint8_t WriteWord(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
	si.i2c_start((Adr << 1) | WRITE);
	si.i2c_write(Command); //Write Command value
	si.i2c_write(Data & 0xFF); //Write LSB
	uint8_t Error = si.i2c_write((Data >> 8) & 0xFF); //Write MSB
	si.i2c_stop();
	return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteWord_LE(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
	si.i2c_start((Adr << 1) | WRITE);
	si.i2c_write(Command); //Write Command value
	si.i2c_write((Data >> 8) & 0xFF); //Write MSB
	si.i2c_write(Data & 0xFF); //Write LSB
	si.i2c_stop();
	// return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteConfig(uint8_t Adr, uint8_t NewConfig)
{
	si.i2c_start((Adr << 1) | WRITE);
	si.i2c_write(CONF_CMD);  //Write command code to Config register
	uint8_t Error = si.i2c_write(NewConfig);
	si.i2c_stop();
	if(Error == true) {
		Config = NewConfig; //Set global config if write was sucessful 
		return 0;
	}
	else return -1; //If write failed, return failure condition
}

int ReadByte(uint8_t Adr, uint8_t Command, uint8_t Pos) //Send command value, and high/low byte to read, returns desired byte
{
	bool Error = SendCommand(Adr, Command);
	si.i2c_rep_start((Adr << 1) | READ);
	uint8_t ValLow = si.i2c_read(false);
	uint8_t ValHigh = si.i2c_read(false);
	si.i2c_stop();
	Error = true; //DEBUG!
	if(Error == true) {
		if(Pos == 0) return ValLow;
		if(Pos == 1) return ValHigh;
	}
	else return -1; //Return error if read failed

}

int ReadWord(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
	bool Error = SendCommand(Adr, Command);
	si.i2c_rep_start((Adr << 1) | READ);
	uint8_t ByteLow = si.i2c_read(false);  //Read in high and low bytes (big endian)
	uint8_t ByteHigh = si.i2c_read(false);
	si.i2c_stop();
	// if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
	// else return -1; //Return error if read failed
	return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

int ReadWord_LE(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
	bool Error = SendCommand(Adr, Command);
	si.i2c_stop();
	si.i2c_start((Adr << 1) | READ);
	uint8_t ByteHigh = si.i2c_read(false);  //Read in high and low bytes (big endian)
	uint8_t ByteLow = si.i2c_read(false);
	si.i2c_stop();
	// if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
	// else return -1; //Return error if read failed
	return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

void SplitAndLoad(uint8_t Pos, unsigned int Val) //Write 16 bits
{
	uint8_t Len = sizeof(Val);
	for(int i = Pos; i < Pos + Len; i++) {
		Reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
	}
}

void SplitAndLoad(uint8_t Pos, long Val)  //Write 32 bits
{
	uint8_t Len = sizeof(Val);
	for(int i = Pos; i < Pos + Len; i++) {
		Reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
	}
}

boolean addressEvent(uint16_t address, uint8_t count)
{
	RepeatedStart = (count > 0 ? true : false);
	return true; // send ACK to master
}

void requestEvent()
{	
	//Allow for repeated start condition 
	if(RepeatedStart) {
		for(int i = 0; i < 2; i++) {
			Wire.write(Reg[RegID + i]);
		}
	}
	else {
		Wire.write(Reg[RegID]);
	}

}

void receiveEvent(int DataLen) {
    //Write data to appropriate location
    if(DataLen == 2){
	    //Remove while loop?? 
	    while(Wire.available() < 2); //Only option for writing would be register address, and single 8 bit value
	    uint8_t Pos = Wire.read();
	    uint8_t Val = Wire.read();
	    //Check for validity of write??
	    Reg[Pos] = Val; //Set register value
	}

	if(DataLen == 1){
		RegID = Wire.read(); //Read in the register ID to be used for subsequent read
	}
}

void stopEvent() {
	StopFlag = true;
	//End comunication
}

