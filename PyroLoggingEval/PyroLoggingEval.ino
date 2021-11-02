#include "Wire.h"
#include "math.h"
#include <SPI.h>
#include <SdFat.h>
#include <MCP3421.h>
#include <DS3231_Logger.h>

#define UVA_ADR 0x02
#define UVB_ADR 0x06
#define ALS_ADR 0x0B
#define WHITE_ADR 0x0D
#define LUXMUL_ADR 0x10
#define IR_MID_ADR 0x13
#define IR_SHORT_ADR 0x15
#define THERM_ADR 0x17

#define XAXIS 0x32
#define YAXIS 0x34
#define ZAXIS 0x36

#define ADR 0x40  //Address of Pyranometer
#define ACCEL_ADR 0x1D //Address of accelerometer (ADXL343)

const uint8_t RedLED = 13;
const uint8_t GreenLED = 15;
const uint8_t BlueLED = 14;

const uint8_t BuiltInLED = 20;

const unsigned long RED = 0xFFFF0000L;
const unsigned long GREEN = 0xFF00FF00L;
const unsigned long BLUE = 0xFF0000FFL;
const unsigned long MAROON = 0xFF800000L;
const unsigned long GOLD = 0xFFFFD700L;
const unsigned long ORANGE = 0xFFFFA500L;
const unsigned long PURPLE = 0xFF800080L;
const unsigned long CYAN = 0xFF00FFFF;
const unsigned long BLACK_ALERT = 0x802019FF;
const unsigned long OFF = 0x00;

const float LuxRes = 0.0036; //Min resolution of lux measurement

const float KippAndZonenConv = 1.257e-5; // [V/W/m^2] for a given kipp and zonen pyranometer 

const float A = 0.003354016;
const float B = 0.0003074038;
const float C = 1.019153E-05;
const float D = 9.093712E-07;

////////////////////////////////////SD////////////////////////////////////////
const int SD_CS = 4;
char FileNameC[11]; //Used for file handling
char FileNameTestC[11]; //Used for file handling
boolean SD_Init = false;
const uint8_t SD_CD = 1;
SdFat SD;
byte  keep_SPCR;

////////////////////////////////////CLOCK//////////////////////////////////////
DS3231_Logger RTC;

MCP3421 adc;

String LogTimeDate = "2063/04/05 20:00:00";

const int Ext3v3Ctrl = 19; 

bool GlobalError = false;

void setup() {
	// put your setup code here, to run once:
	pinMode(Ext3v3Ctrl, OUTPUT);
  	digitalWrite(Ext3v3Ctrl, LOW); //Turn on power by default, turn off later to sleep

  	pinMode(RedLED, OUTPUT);
	pinMode(GreenLED, OUTPUT);
	pinMode(BlueLED, OUTPUT);
	pinMode(BuiltInLED, OUTPUT);
	LED_Color(OFF); //Turn off intially 

	digitalWrite(BuiltInLED, LOW); //Turn LED On

	Serial.begin(115200); 
	Serial.println("Welcome to the Sphere...");
	Wire.begin();
	RTC.Begin(); //Initalize RTC
	InitAccel();
	adc.Begin();
	adc.SetResolution(18);
	// adc.SetGain(8);
	// SD.begin(SD_CS);
	pinMode(SD_CS, OUTPUT);
	SDTest();
	InitLogFile();

	digitalWrite(BuiltInLED, HIGH); //Turn LED off
	if(GlobalError == false) LED_Color(GREEN);
	else LED_Color(RED);
	delay(2000);
	LED_Color(OFF);

}


void loop() {
  // put your main code here, to run repeatedly:
  GetTime(); //FIX!
  String Data = String(GetUVA()) + "," + String(GetUVB()) + "," + String(GetALS()) + "," + String(GetWhite()) + "," + String(GetLux()) + "," + String(GetIR_Short()) + "," + String(GetIR_Mid()) + "," + String(GetWatts()) + "," + String(GetAngle(3)) + "," + String(GetAngle(4)) + "," + LogTimeDate;
  LogStr(Data);
  Serial.println(LogTimeDate);
  delay(1000);
}

void LED_Color(unsigned long Val) {
	int Red = 0; //Red led color
	int Green = 0;  //Green led color
	int Blue = 0;  //Blue led color
	int Lum = 0;  //Luminosity

	//Parse all values from single Val
	Blue = Val & 0xFF;
	Green = (Val >> 8) & 0xFF;
	Red = (Val >> 16) & 0xFF;
	Lum = (Val >> 24) & 0xFF;
	//  Lum = 255 - Lum; //Invert since LEDs are open drain

	analogWrite(RedLED, 255 - (Red * Lum)/0xFF);
	analogWrite(GreenLED, 255 - (Green * Lum)/0xFF);
	analogWrite(BlueLED, 255 - (Blue * Lum)/0xFF);
}

void GetTime() {
	//Update global time string
	// DateTime TimeStamp = RTC.now();
	// LogTimeDate = String(TimeStamp.year()) + "/" + String(TimeStamp.month()) + "/" + String(TimeStamp.day()) + " " + String(TimeStamp.hour()) + ":" + String(TimeStamp.minute()) + ":" + String(TimeStamp.second());  
	LogTimeDate = RTC.GetTime(0);
}

float GetWatts() 
{
	float Val = adc.GetVoltage();
	Serial.println(Val, 9); //DEBUG!
	return Val/KippAndZonenConv;
}

void InitLogFile(){
    String FileName = "Pyro";
    int FileNum = 1;
    String NumString = "01";
    (FileName + "01"+ ".txt").toCharArray(FileNameC, 11);
    while(SD.exists(FileNameC)) {
      FileNum += 1;
      NumString = String(FileNum, DEC);
      (FileName + NumString + ".txt").toCharArray(FileNameC, 11);
    }
    (FileName + NumString + ".txt").toCharArray(FileNameC, 11);
  
    LogStr("UVA, UVB, ALS, White, Lux [lx], IR Short, IR Mid, KippAndZonen [W/m^2], Roll [deg], Pitch [deg], Time [UTC]"); 
}

int LogStr(String Val) {
  File DataFile = SD.open(FileNameC, FILE_WRITE);

  // if the file is available, write to it:
  if (DataFile) {
    DataFile.println(Val);
	   // return 0;
  }
  // if the file isn't open, pop up an error:
  else {
	   // return -1;
  }

  DataFile.close();
}

// uint8_t InitMonarch() 
// {

// }

void SDTest() {
	bool SDError = false;
	// bool SD_Test = true;

	pinMode(SD_CD, INPUT);
	bool CardPressent = digitalRead(SD_CD);

	Serial.print("SD: ");

	if(CardPressent) Serial.println(" NO CARD");

	if (!SD.begin(SD_CS)) {
		Serial.println("SD Failed!");
		GlobalError = true;
  	}

	String FileNameTest = "HWTest";
	(FileNameTest + ".txt").toCharArray(FileNameTestC, 11);
	SD.remove(FileNameTestC); //Remove any previous files

	randomSeed(analogRead(A7)); //Seed with a random number to try to endsure randomness
	int RandVal = random(30557); //Generate a random number between 0 and 30557 (the number of words in Hamlet)
	char RandDigits[6] = {0};
	sprintf(RandDigits, "%d", RandVal); //Convert RandVal into a series of digits
	int RandLength = (int)((ceil(log10(RandVal))+1)*sizeof(char)); //Find the length of the values in the array

	File DataWrite = SD.open(FileNameTestC, FILE_WRITE);
	if(DataWrite) {
	DataWrite.println(RandVal);
	DataWrite.println("\nHe was a man. Take him for all in all.");
	DataWrite.println("I shall not look upon his like again.");
	DataWrite.println("-Hamlet, Act 1, Scene 2");
	}
	DataWrite.close();

	char TestDigits[6] = {0};
	File DataRead = SD.open(FileNameTestC, FILE_READ);
	if(DataRead) {
	DataRead.read(TestDigits, RandLength);

	for(int i = 0; i < RandLength - 1; i++){ //Test random value string
	  if(TestDigits[i] != RandDigits[i]) {
	    // SDError = true;
	    GlobalError = true;
	  }
	}
	}
	DataRead.close();

	//Print SD file info??
	//  if(!SDError && !GlobalError) {


	keep_SPCR=SPCR; 
  
	if(SDError && !CardPressent) Serial.println("FAIL");
  else Serial.println("PASS");

	//  SPI.end();
	//  digitalWrite(SD_CS, LOW);
	//  pinMode(5, OUTPUT);
	//  digitalWrite(5, LOW);
	//  pinMode(6, OUTPUT);
	//  digitalWrite(6, LOW);
	//  pinMode(7, OUTPUT);
	//  digitalWrite(7, LOW);
}

uint8_t InitAccel()  //Add variable address ability??
{
  WriteByte(ACCEL_ADR, 0x2D, 0x08); //Turn on accelerometer
  WriteByte(ACCEL_ADR, 0x31, 0x08); //
  WriteByte(ACCEL_ADR, 0x38, 0x00); //Bypass FIFO
  WriteByte(ACCEL_ADR, 0x2C, 0x0A); //Setup rate
}

float GetG(uint8_t Axis) 
{
  WriteByte(ACCEL_ADR, 0x2D, 0x08); //Turn on accelerometer
  // WriteByte(ACCEL_ADR, XAXIS);
  // delay(100);
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(XAXIS + 2*Axis);
  Wire.endTransmission();
  Wire.beginTransmission(ACCEL_ADR);
  Wire.requestFrom(ACCEL_ADR, 2);
  int LSB = Wire.read();
  int MSB = Wire.read();
  Wire.endTransmission();
  // return ReadWord(ACCEL_ADR, XAXIS); 
  // Serial.print(MSB, HEX); Serial.println(LSB, HEX); //DEBUG!
  float g = ((MSB << 8) | LSB)*(0.0039); 
  return g; 
}

float GetAngle(uint8_t Axis)
{
  float ValX = GetG(0); //Used to get g values
  float ValY = GetG(1);
  float ValZ = GetG(2);
  float Val = 0;
  switch(Axis) {
    case(0):
      Val = asin(ValX); 
      break;
    case(1):
      Val = asin(ValY);
      break;
    case(2):
      Val = acos(ValZ);
      break;
    case(3):
      Val = atan(ValX/(sqrt(pow(ValY, 2) + pow(ValZ, 2))))*(180.0/3.14); //Return pitch angle
      break;
    case(4):
      Val = atan(ValY/(sqrt(pow(ValX, 2) + pow(ValZ, 2))))*(180.0/3.14); //Return roll angle
      break;
  }

  return Val; 
}

long GetUVA() 
{
  long LSW = ReadWord(ADR, UVA_ADR); //Read low word
  long MSW = ReadWord(ADR, UVA_ADR + 2); //Read high word
  return (MSW << 16) | LSW; //Return concatenated result
}

long GetUVB()
{
  long LSW = ReadWord(ADR, UVB_ADR); //Read low word
  long MSW = ReadWord(ADR, UVB_ADR + 2); //Read high word
  return (MSW << 16) | LSW; //Return concatenated result
}

unsigned int GetALS()
{
  return ReadWord(ADR, ALS_ADR); //Read back als value
}

unsigned int GetWhite()
{
  return ReadWord(ADR, WHITE_ADR); //Read back white value
}

float GetLux()
{
  return float(ReadWord(ADR, ALS_ADR))*float(ReadWord(ADR, LUXMUL_ADR))*LuxRes;  //Multiply lux value by set gain from LuxMul
}

float GetIR_Short()
{
  return float(ReadWord(ADR, IR_SHORT_ADR))*(1.25e-4);
}

float GetIR_Mid()
{
  return float(ReadWord(ADR, IR_MID_ADR))*(1.25e-4);
}

float GetTemp()
{
  float Val = float(ReadWord(ADR, THERM_ADR))*(1.25e-4);
  return TempConvert(Val, 3.3, 10000.0, A, B, C, D, 10000.0);
}

float TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25)
{
  // float Rt = (Vcc/V)*R - R;  //Use if thermistor is on TOP side of voltage divider
  float Rt = -R/(1 - (Vcc/V)); //Use if thermistor is on BOTTOM side of voltage divider  
  float LogRt = log(Rt/R25);
  float T = 1.0/(A + B*LogRt + C*pow(LogRt, 2.0) + D*pow(LogRt, 3.0));
  return T;
}

void PrintAllRegs() 
{
  for(int i = 0; i < 0x1A; i++) {
    Serial.print("Reg"); Serial.print(i, HEX); Serial.print(":\t"); //Print register number
    Serial.println(ReadByte(ADR, i)); //Print register value
  }
  Serial.print("\n\n");
}

uint8_t WriteByte(uint8_t Adr, uint8_t Pos, uint8_t Val)
{
  Wire.beginTransmission(Adr);
  Wire.write(Pos);  //Identify register
  Wire.write(Val);  //Write desired value to register
  Wire.endTransmission(); //End I2C message
}

// uint8_t WriteByte(uint8_t Adr, uint8_t Val)
// {
//   Wire.beginTransmission(Adr);
//   Wire.write(Val);  //Write desired value to register
//   Wire.endTransmission(); //End I2C message
// }

uint8_t ReadByte(uint8_t Adr, uint8_t Pos)
{
  Wire.beginTransmission(Adr);
  Wire.write(Pos);  //Read from desired position
  Wire.endTransmission(true); 
  Wire.requestFrom(Adr, 1); //Read a single byte
  while(Wire.available() < 1); //Wait for byte to be read in
  return Wire.read(); //Read the desired value back
}

unsigned int ReadWord(uint8_t Adr, uint8_t Pos)
{
  unsigned int LSB = ReadByte(Adr, Pos);
  unsigned int MSB = ReadByte(Adr, Pos + 1);
  return (MSB << 8) | LSB; //Read the desired value back
}
