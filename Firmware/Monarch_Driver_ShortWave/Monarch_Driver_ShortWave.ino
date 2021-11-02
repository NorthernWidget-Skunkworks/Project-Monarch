//Monarch_Driver_ShortWave.ino
//v0.0.0
#include "SlowSoftI2CMaster.h"
#include "WireS.h"
// #include <EEPROM.h> //DEBUG!
//Commands

#define CTRL 0x00  //Define location of onboard control/confiuration register

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

#define LOW_LIM_VIS 10000  //Lower limit to the auto ranging of the VEML6030
#define HIGH_LIM_VIS 55000 //Upper limit to the auto ranging of the VEML6030

#define ADR_SEL_PIN 7 //Digital pin 7 is used to test which device address should be used
// #define ADR_ALT 0x41 //Alternative device address

//Global values for gain and int time of visable light sensor
uint8_t Gain = 0;
unsigned int IntTime = 0;
uint8_t GainValsVis[4] = {0b10, 0b11, 0b00, 0b01}; //Gain values for visible sensor
uint8_t GainsVis[4] = {1, 2, 8, 16}; //Gain multipliers for visable sensor 
uint8_t IntTimeValsVis[6] = {0b1100, 0b1000, 0b0000, 0b0001, 0b0010, 0b0011}; //Integration time values for visible sensor

//Compensation constants
float a = 1.92;
float b = 0.55;
float c = 2.46;
float d = 0.63;

volatile uint8_t ADR = 0x40; //Use arbitraty address, change using generall call??
const uint8_t ADR_Alt = 0x41; //Alternative device address  //WARNING! When a #define is used instead, problems are caused

uint8_t Config = 0; //Global config value

uint8_t Reg[26] = {0}; //Initialize registers
bool StartSample = true; //Flag used to start a new converstion, make a conversion on startup
// const unsigned int UpdateRate = 5; //Rate of update
const unsigned int UpdateRate[] = {5, 10, 60, 300}; //FIX with better numbers! 

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_B2, PIN_A7, true);  //Initialize software I2C

volatile bool StopFlag = false; //Used to indicate a stop condition 
volatile uint8_t RegID = 0; //Used to denote which register will be read from
volatile bool RepeatedStart = false; //Used to show if the start was repeated or not

void setup() {
  Serial.begin(115200); //DEBUG!
  Serial.println("begin"); //DEBUG!
  Reg[CTRL] = 0x00; //Set Config to POR value
  pinMode(ADR_SEL_PIN, INPUT_PULLUP);
  pinMode(10, OUTPUT); //DEBUG!
  pinMode(9, OUTPUT); //DEBUG!
  digitalWrite(10, HIGH); //DEBUG!
  digitalWrite(9, LOW); //DEBUG!
  if(!digitalRead(ADR_SEL_PIN)) ADR = ADR_Alt; //If solder jumper is bridged, use alternate address //DEBUG!
  Wire.begin(ADR);  //Begin slave I2C
  // EEPROM.write(0, ADR);
  InitVEML(0x48); //Init Vis (VEML6030)
  InitVEML(0x10); //Init UV (VEML6075)
  InitADC(); //Init ADC (ADS1115)

  //Setup I2C slave
	Wire.onAddrReceive(addressEvent); // register event
	Wire.onRequest(requestEvent);     // register event
	Wire.onReceive(receiveEvent);
	Wire.onStop(stopEvent);

  si.i2c_init(); //Begin I2C master

  // AutoRange_Vis(); //Auto range for given light conditions
	digitalWrite(9, HIGH); //DEBUG!
	digitalWrite(10, LOW); //DEBUG!
}

void loop() {
	// static unsigned int Count = 0; //Counter to determine update rate
	// uint8_t Ctrl = Reg[CTRL]; //Store local value to improve efficiency
	uint8_t UpdateRateBits = Reg[CTRL] & 0x03; 
	static unsigned long Timeout = millis() % (UpdateRate[3]*1000); //Take mod with longest update rate 

	// digitalWrite(10, HIGH); //DEBUG!
	if(StartSample == true) {

		// Config = Reg[CTRL]; //Update local register val
		//Read new values in
		if(BitRead(Reg[CTRL], 2) == 0) {  //Only auto range if configured in Ctrl register 
			AutoRange_Vis();  //Run auto range
			delay(800); //Wait for new sample
		}
		// digitalWrite(9, HIGH); //DEBUG!
		Reg[CTRL] = Reg[CTRL] &= 0x7F; //Clear ready flag only while new vals being written
		SplitAndLoad(0x0B, GetALS()); //Load ALS value
		SplitAndLoad(0x0D, GetWhite()); //Load white value
		SplitAndLoad(0x02, long(GetUV(0))); //Load UVA
		SplitAndLoad(0x07, long(GetUV(1))); //Load UVB
		SplitAndLoad(0x10, GetLuxGain()); //Load lux multiplier 
		SplitAndLoad(0x13, GetADC(0));
		SplitAndLoad(0x15, GetADC(1));
		SplitAndLoad(0x17, GetADC(2));

		Reg[CTRL] = Reg[CTRL] |= 0x80; //Set ready flag
		digitalWrite(9, LOW); //DEBUG!
		StartSample = false; //Clear flag when new values updated  
	}

	//Make sure there is not a protnetial logic problem when changing update rate!!!!!!
	if(millis() % (UpdateRate[3]*1000) - Timeout > UpdateRate[UpdateRateBits]*1000) {  
		StartSample = true; //Set flag if number of updates have rolled over 
		Timeout = millis() % (UpdateRate[3]*1000); //Restart timer
		// digitalWrite(10, LOW); //DEBUG!
	}

	if(BitRead(Reg[CTRL], 3) == 1) {  //If manual autorange is commanded
		AutoRange_Vis(); //Call autorange
		delay(800); //Wait for new data
		Reg[CTRL] &= 0xF7; //Clear auto range bit to inform user autorange is complete
	}

	// if(Reg[CTRL] != Config) {
	// 	Config = Reg[CTRL]; //Update local register 
	// 	Timeout = millis() % (UpdateRate[3]*1000); //Reset counter if control register changes
	// }
	delay(100);
}

uint8_t InitVEML(uint8_t Adr) 
{
	uint8_t CMD = 0; 
	if(Adr == 0x48) CMD = 0x13;  //DEBUG! Replace w/0x23 or 0x13 
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
		// Serial.println(UV);
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
	float GainTemp = float(Gain); //Hardcode max range for gain and int time 
	float IntTimeTemp = float(IntTime);  
	float Resolution = (1.8432/IntTimeTemp)/GainTemp;
	// unsigned int Resolution = (512/(IntTime/25.0))/(Gain/0.125);
	Serial.println(int(Resolution/0.0036)); //DEBUG!
	return int(Resolution/0.0036); //Return Lux scaler
}


uint8_t AutoRange_Vis() 
{	
	// WriteWord(VIS_ADR, CONF_CMD, (GainValsVis[0] << 11) | (IntTimeValsVis[0] << 6)); //Write min gain vals
	WriteWord(VIS_ADR, CONF_CMD, 0x1300); //DEBUG! Replace w/ 0x1300
	delay(800); //Wait for new sample
	unsigned int Val = GetALS();
	Serial.print("Val = "); Serial.println(Val); //DEBUG!
	// Serial.println(Val); //DEBUG!
	if(Val > HIGH_LIM_VIS) {
		Gain = GainsVis[0];  //Set global values
		IntTime = pow(2, 0); 
	}

	bool InRange = false; //Flag to test for range
	unsigned int ValTest = 0;
	uint8_t GainTemp = 0;  //Index of desired gain
	uint8_t IntTimeTemp = 0; //Index of desired integration time
	//DEBUG!!!!!!
	// while(!InRange && IntTimeTemp < 6) {
	// 	ValTest = (Val*int(GainsVis[GainTemp])*(1 << int(IntTimeTemp)));
	// 	Serial.print("Val = "); Serial.println(ValTest); //DEBUG!
	// 	Serial.print("D = "); Serial.print(GainsVis[GainTemp]); Serial.print("\t"); Serial.println(IntTimeTemp);
	// 	if(ValTest < LOW_LIM_VIS) {
	// 		GainTemp++;
	// 	}
	// 	if(GainTemp > 3 && IntTimeTemp < 5) {
	// 		GainTemp = 0;
	// 		IntTimeTemp++;
	// 	}

	// 	if(GainTemp == 3 && IntTimeTemp == 5) InRange = true; //Set if max value is reached 
	// 	if(ValTest > LOW_LIM_VIS && ValTest < HIGH_LIM_VIS) InRange = true; //Found correct gain and int vals
	// }
	GainTemp = 0; //DEBUG!
	IntTimeTemp = 0;  //DEBUG!
	//DEBUG!
	// Serial.print(GainValsVis[GainTemp], BIN); Serial.print("\t"); Serial.println(IntTimeValsVis[IntTimeTemp], BIN);  //DEBUG!
	// WriteWord(VIS_ADR, CONF_CMD, (GainValsVis[GainTemp] << 11) | (IntTimeValsVis[IntTimeTemp] << 6));  //Write new gain value
	// Serial.print((GainValsVis[GainTemp] << 11) | (IntTimeValsVis[IntTimeTemp] << 6), HEX); //DEBUG!
	Gain = GainsVis[GainTemp];  //Set global values
	IntTime = pow(2, IntTimeTemp); 
}
// float GetGain(uint8_t GainBits)
// {
// 	uint8_t X0 = ReadBit(GainBits, 1);
// 	uint8_t X1 = ReadBit(GainBits, 0);
// 	Gain = pow(2, X0 - 3*X1);
// 	return Gain; 
// }

// unsigned int Gain2Bits(float GainVal)
// {
// 	for(int i = 0; i < 4; i++) {  //Use linear search to avoid float math and increase speed
// 		if(GainVals[i] == GainVal) {
// 			return (i << 11); //if entries match, return bits
// 		}
// 	}
// 	return 0x1000; //Return gain of 1/8 if not a valid gain value 
// }

// unsigned int GetIntTime()
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	int X2X1 = (Config >> 6) & 0x03; 
// 	int X3 = ReadBit(Config, 8);
// 	int X4 = ReadBit(Config, 9);
// 	IntTime = 100*pow(2, X2X1)/pow(2, X3 + X4);  //Do some ugly math to go from bit pattern to value
// 	return IntTime; 
// }

// unsigned int IntTime2Bits(unsigned int Time)
// {
// 	uint8_t X2X1 = (int)(log(Time/100)/log(2));
// 	uint8_t X4X3 = (int)(((Time % 100) / 25) + 2*(Time % 50)/25);
// 	return ((X4X3 << 2) | (X2X1)) << 6; 
// }

// uint8_t SetGain(unsigned int GainVal)
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	// Serial.print("Config = "); //DEBUG!
// 	// Serial.print(Config); //DEBUG! 
// 	// Serial.print(" "); //DEBUG!
// 	// Serial.println((Config & 0xE7FF) | GainVal); //DEBUG!
// 	return WriteConfig((Config & 0xE7FF) | GainVal);
// }

// uint8_t AutoRange()  //Automatically finds maximum gain and resolution values for given irradiance
// {
// 	// PowerSaveOff(); //Turn power save off for fastest reading
// 	SetIntTime(IT25); //Set to minimum integration time
// 	SetGain(GAIN_1_8); //Set to minmum gain
// 	// Serial.print("UnRanged Result ="); Serial.println(ReadWord(CONF_CMD), HEX); //DEBUG!
// 	// PowerOn();
// 	// long StartTime = millis(); //DEBUG! 
// 	// while(millis() - StartTime < 50) {  //DEBUG!
// 	// 	Serial.println(ReadWord(INT_CMD), HEX); //DEBUG!
// 	// }  //DEBUG!
// 	delay(30); //Wait for new sample
// 	float TestLux = GetLux(); //Get new lux value
// 	unsigned long HighLux = 120796;  //Start at max value
// 	unsigned int NewIntTime = 25; //Default to min value
// 	float NewGainHigh = 0.125; //Default to min value
// 	float NewGainLow = 0.25; //Default to 2nd lowest value
// 	float NewGain = 0.125; //Default to min value
// 	// Serial.print("LuxTest = "); //DEBUG!
// 	// Serial.println(TestLux); //DEBUG!
// 	//Increment through lux ranges to find desired gain range
// 	if(TestLux < 236) {  //If Lux is too small to measure at max values (<1.8432) or in minimum range, simply set to highest gain and integration time
// 		NewIntTime = 800;
// 		NewGain = 2;
// 	}
// 	else {  //If lux is not outside low range, search for a value
// 		for(int i = 0; i < 6; i++) {
// 			if(TestLux < HighLux && TestLux >= HighLux/2.0) {
// 				NewIntTime = NewIntTime * ceil(pow(2, i));
// 				break; //breakout of for loop since result is found
// 			}
// 			else HighLux = ceil(HighLux/2.0); //If not found, go to next lux range
// 		}

// 		if(TestLux < HighLux * 0.0625) NewGain = 2; //If below the lowest max for integration range, set to max gain
// 		else {  //Otherwise search for a new value 
// 			for(int g = 1; g < 3; g++) {
// 				if(TestLux < HighLux * (0.125/NewGainHigh) && TestLux >= HighLux * (0.125/NewGainLow)) {
// 					NewGain = NewGainHigh;
// 					break; //Break loop once gain is found
// 				}
// 				else {
// 					NewGainHigh = NewGainLow;
// 					NewGainLow = GetGain(GainValBits[g + 1]);
// 				}
// 			}
// 		}
// 	}

// 	// Serial.print("AutoRange Vals = "); Serial.print(NewGain); Serial.print(" "); Serial.println(NewIntTime); //DEBUG!
// 	unsigned int GainBits = Gain2Bits(NewGain); //Convert new gain value
// 	unsigned int IntBits = IntTime2Bits(NewIntTime); //Convert to new integration time
// 	// Serial.print("Bits = "); Serial.print(GainBits, HEX); Serial.print(" "); Serial.println(IntBits, HEX); //DEBUG!

// 	SetGain(GainBits);
// 	SetIntTime(IntBits);
// 	// Serial.print("AutoRanged Result ="); Serial.println(ReadWord(CONF_CMD), HEX); //DEBUG!
// }

// float GetGain()
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	// Serial.println(Config); //DEBUG!
// 	int X0 = ReadBit(Config, 11);
// 	int X1 = ReadBit(Config, 12);
// 	Gain = pow(2, X0 - 3*X1);
// 	return Gain; 
// }

// unsigned int GetIntTime()
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	int X2X1 = (Config >> 6) & 0x03; 
// 	int X3 = ReadBit(Config, 8);
// 	int X4 = ReadBit(Config, 9);
// 	IntTime = 100*pow(2, X2X1)/pow(2, X3 + X4);  //Do some ugly math to go from bit pattern to value
// 	return IntTime; 
// }

// float GetLux() 
// {	//Add non-linear correction! 
// 	GetGain(); //Update global values
// 	GetIntTime(); 
// 	float Resolution = (1.8432/((float)IntTime/25.0))*(0.125/Gain);
// 	// Serial.print("TEST = "); //DEBUG!
// 	// Serial.print(Resolution); //DEBUG!
// 	// Serial.print(" "); //DEBUG!
// 	// Serial.print(IntTime); //DEBUG! 
// 	// Serial.print(" "); //DEBUG!
// 	// Serial.println(Gain); //DEBUG!
// 	return GetALS()*Resolution; //Return scaled Lux mesurment
// }

// uint8_t ReadBit(unsigned int Data, uint8_t Pos)
// {
// 	return (Data >> Pos) & 0x01; 
// }

bool BitRead(uint8_t Val, uint8_t Pos) //Read the bit value at the specified position
{
	return (Val >> Pos) & 0x01;
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
	Serial.print("Error = "); Serial.println(Error); //DEBUG!
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

void receiveEvent(int DataLen) 
{
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

void stopEvent() 
{
	StopFlag = true;
	//End comunication
}
