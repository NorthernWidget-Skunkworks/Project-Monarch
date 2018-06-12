//DysonDemoRead.ino
#include "Wire.h"

#define UVA_ADR 0x02
#define UVB_ADR 0x06
#define ALS_ADR 0x0B
#define WHITE_ADR 0x0D
#define LUXMUL_ADR 0x10

#define ADR 0x40  //Address of Pyranometer

const float LuxRes = 0.0036; //Min resolution of lux measurment

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
  delay(1000);

}

long GetUVA() 
{
  long LSW = ReadWord(UVA_ADR); //Read low word
  long MSW = ReadWord(UVA_ADR + 2); //Read high word
  return (MSW << 16) | LSW; //Return concatonated result
}

long GetUVB()
{
  long LSW = ReadWord(UVB_ADR); //Read low word
  long MSW = ReadWord(UVB_ADR + 2); //Read high word
  return (MSW << 16) | LSW; //Return concatonated result
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
  // Serial.print("LuxMul = "); Serial.println(float(ReadWord(ALS_ADR))*float(ReadWord(LUXMUL_ADR))*LuxRes);
  return float(ReadWord(ALS_ADR))*float(ReadWord(LUXMUL_ADR))*LuxRes;  //Multiply lux value by set gain from LuxMul
}

float GetIR_Short()
{

}

float GetIR_Long()
{

}

void PrintAllRegs() {
  // Wire.beginTransmission(ADR);
  // Wire.write(0x00);  //Start from beginning 
  // Wire.endTransmission(false); 
  // Wire.requestFrom(ADR, 0x1A); //Get all values
  // delay(10);
  // int i = 0; //Counter
  // while(Wire.available() > 0) {
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
  // Wire.beginTransmission(ADR);
  // Wire.write(Pos);  //Read from desired position
  // Wire.endTransmission(true); 
  // Wire.requestFrom(ADR, 2); //Read a single byte
  // while(Wire.available() < 2); //Wait for byte to be read in
  // unsigned int LSB = Wire.read();
  // unsigned int MSB = Wire.read();
  unsigned int LSB = ReadByte(Pos);
  unsigned int MSB = ReadByte(Pos + 1);
  return (MSB << 8) | LSB; //Read the desired value back
}



