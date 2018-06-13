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

#define UVA_ADR 0x02
#define UVB_ADR 0x06
#define ALS_ADR 0x0B
#define WHITE_ADR 0x0D
#define LUXMUL_ADR 0x10
#define IR_MID_ADR 0x13
#define IR_SHORT_ADR 0x15
#define THERM_ADR 0x17

#define ADR 0x40  //Address of Pyranometer

const float LuxRes = 0.0036; //Min resolution of lux measurement

const float A = 0.003354016;
const float B = 0.0003074038;
const float C = 1.019153E-05;
const float D = 9.093712E-07;

void setup() {
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
  delay(1000);

}

long GetUVA() 
{
  long LSW = ReadWord(UVA_ADR); //Read low word
  long MSW = ReadWord(UVA_ADR + 2); //Read high word
  return (MSW << 16) | LSW; //Return concatenated result
}

long GetUVB()
{
  long LSW = ReadWord(UVB_ADR); //Read low word
  long MSW = ReadWord(UVB_ADR + 2); //Read high word
  return (MSW << 16) | LSW; //Return concatenated result
}

unsigned int GetALS()
{
  return ReadWord(ALS_ADR); //Read back als value
}

unsigned int GetWhite()
{
  return ReadWord(WHITE_ADR); //Read back white value
}

float GetLux()
{
  return float(ReadWord(ALS_ADR))*float(ReadWord(LUXMUL_ADR))*LuxRes;  //Multiply lux value by set gain from LuxMul
}

float GetIR_Short()
{
  return float(ReadWord(IR_SHORT_ADR))*(1.25e-4);
}

float GetIR_Mid()
{
  return float(ReadWord(IR_MID_ADR))*(1.25e-4);
}

float GetTemp()
{
  float Val = float(ReadWord(THERM_ADR))*(1.25e-4);
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
    Serial.println(ReadByte(i)); //Print register value
  }
  Serial.print("\n\n");
}

uint8_t WriteByte(uint8_t Pos, uint8_t Val)
{
  Wire.beginTransmission(ADR);
  Wire.write(Pos);  //Identify register
  Wire.write(Val);  //Write desired value to register
  Wire.endTransmission(); //End I2C message
}

uint8_t ReadByte(uint8_t Pos)
{
  Wire.beginTransmission(ADR);
  Wire.write(Pos);  //Read from desired position
  Wire.endTransmission(true); 
  Wire.requestFrom(ADR, 1); //Read a single byte
  while(Wire.available() < 1); //Wait for byte to be read in
  return Wire.read(); //Read the desired value back
}

unsigned int ReadWord(uint8_t Pos)
{
  unsigned int LSB = ReadByte(Pos);
  unsigned int MSB = ReadByte(Pos + 1);
  return (MSB << 8) | LSB; //Read the desired value back
}



