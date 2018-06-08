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
  Serial.begin(115200); //DEBUG!
  Serial.println("begin"); //DEBUG!
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
		Serial.println(GetUV(0)); //DEBUG!
		Serial.println(GetUV(1)); //DEBUG!
		Serial.println(GetWhite()); //DEBUG!
		Serial.println(GetALS()); //DEBUG!

		// unsigned int Val = 0; //Temp value for getting data and moving to regs
		SplitAndLoad(0x0B, GetALS()); //Load ALS value
		SplitAndLoad(0x0D, GetWhite()); //Load white value
		SplitAndLoad(0x02, long(GetUV(0))); //Load UVA
		SplitAndLoad(0x07, long(GetUV(1))); //Load UVB

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
    si.i2c_start((Adr << 1) | WRITE);
    si.i2c_write(0x00);
    si.i2c_write(0x00);
    si.i2c_write(0x00);
    si.i2c_stop(); 

    return 0; //Fix dummy
    //Setup gain and integration values manually! 
}

uint8_t InitADC() 
{
	//Setup ADC system 
}

float GetUV(uint8_t Sel) //Select A or B using Sel value (0 or 1) 
{
	// Config = ReadByte(UV_ADR, CONF_CMD, 0);
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

unsigned int GetALS() 
{
	return ReadWord(VIS_ADR, ALS_CMD);
}

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
//   RW = (address & 0x01); //Test for read/write
// 	if(count == 0) StartFlag = true; //Set flag to start comunication to slave
// //StartFlag = true; //DEBUG!
// 	if(count > 0) RestartFlag = true;  //Set read flag after multiple
// //	slaveAddr = AddressRemap[address - ADDR]; //Convert input address to on board address
//   slaveAddr = 0x49; //DEBUG!
	RepeatedStart = (count > 0 ? true : false);
	return true; // send ACK to master
}

void requestEvent()
{	
	//Allow for repeated start condition 
//	StopFlag = false;
//	while(!StopFlag) {  //Read values back until you get a stop flag 
//  		Wire.write(Reg[RegID++]);
//	}
	if(RepeatedStart) {
		for(int i = 0; i < BUF_LENGTH; i++) {
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

