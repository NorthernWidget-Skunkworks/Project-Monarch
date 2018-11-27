/******************************************************************************
DysonDemoRead.cpp
Demo script for Dyson shortwave pyronometer interface
Bobby Schulz @ Northern Widget LLC
6/13/2018
https://github.com/NorthernWidget-Skunkworks/Project-Dyson

This script is used to demonstrate the interface to the Dyson shortwave pyranometer module and provide 
conversions for some of the values into the appropriate human-readable units.

"The laws of nature are constructed in such a way as to make the universe as interesting as possible."
-Freeman Dyson

Distributed as-is; no warranty is given.
******************************************************************************/

#include "Wire.h"
#include "math.h"

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

#define ADR 0x41  //Address of Pyranometer
#define ACCEL_ADR 0x1D //Address of accelerometer (ADXL343)

const float LuxRes = 0.0036; //Min resolution of lux measurement

const float A = 0.003354016;
const float B = 0.0003074038;
const float C = 1.019153E-05;
const float D = 9.093712E-07;

void setup() {
  pinMode(19, OUTPUT); //Setup power switch pin
  digitalWrite(19, LOW); //Turn on power on Margay v1
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Welcome to the Machine...");
}

void loop() {
   PrintAllRegs();
  Serial.print("UVA = "); 
  Serial.println(GetUVA());
  Serial.print("UVB = "); 
  Serial.println(GetUVB());
  Serial.print("ALS = ");
  Serial.println(GetALS());
  Serial.print("White = ");
  Serial.println(GetWhite());
  Serial.print("Lux = ");
  Serial.println(GetLux());
  Serial.print("IR Short = ");
  Serial.println(GetIR_Short());
  Serial.print("IR Mid = ");
  Serial.println(GetIR_Mid());
  Serial.print("Temp = ");
  Serial.println(GetTemp() - 273.15);
  Serial.print("Tilt = ");
  for(int i = 3; i < 5; i++) {
    Serial.print(GetAngle(i));
    Serial.print(",");
  }
  Serial.print("\n\n");
  delay(1000); 

}

uint8_t InitDyson() 
{

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

float TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25){
  // float Rt = (Vcc/V)*R - R;  //Use if thermistor is on TOP side of voltage divider
  float Rt = -R/(1 - (Vcc/V)); //Use if thermistor is on BOTTOM side of voltage divider  
  float LogRt = log(Rt/R25);
  float T = 1.0/(A + B*LogRt + C*pow(LogRt, 2.0) + D*pow(LogRt, 3.0));
  return T;
}

void PrintAllRegs() {
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



