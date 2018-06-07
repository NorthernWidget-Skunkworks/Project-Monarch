//DysonShortWave_I2C_Test.ino
// #include <Arduino.h>
#include "WireS.h"                              // I2C library for ATtiny841 (and other modern ATtinys)
#include <SlowSoftI2CMaster.h>
// #define F_CPU 8000000                          // clock speed: 8MHz (internal) - modify as needed

//Define I2C Master values
#define I2C_TIMEOUT 1000
#define I2C_PULLUP 1
//#define SDA_PORT PORTB
//#define SDA_PIN PIN_B2 // = D2
//#define SCL_PORT PORTA
//#define SCL_PIN 7 // = D3

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_B2, PIN_A7, true);
//#define I2C_7BITADDR 0x68 // DS1307
#define ADDRLEN 1 // address length, usually 1 or 2 bytes

//ADD ABILITY TO SWITCH ADDRESS OFFSET BASED ON STATE OF ADR PIN!
const byte ADDR = 0x66;  //DEBUG! //Default to 0x64 
const byte Mask = 0x03; //Do not match on lowest 2 bits of the address
volatile uint8_t slaveAddr;  //Used to store the slave info

bool DataInFlag = false; //Flag for input data to sub-slaves
bool DataOutFlag = false; //Flag for output data to master
bool StartFlag = false; //Flag for starting the transmission to sub-slave
bool RestartFlag = false; //Flag for multiple start conditions
bool StopFlag = false; //Flag for stopping transmission to sub-slave

byte DataIn[8] = {0}; //Buffer of 8 bytes for input data
byte DataOut[8] = {0}; //Buffer of 8 bytes for output data 
byte Dummy[8] = {0}; //DEBUG!
byte Reg = 0; //Value of register to read data from
// const byte NumBytes = 4;

uint8_t AddressRemap[4] = {0x10, 0x48, 0x49, 0x53};  //Used to remap from offset address to sub-slave addresses
uint8_t RW = 0; //Used to set read vs write

volatile uint8_t NumBytes = 0; //Number of bytes recieved by I2C
void setup() {
  Serial.begin(115200); //DEBUG! 
   Serial.println("begin"); //DEBUG!
	Wire.begin(ADDR, Mask);      //Join bus with base address and mask value
  	// Wire.begin(2, (3 << 1 | 1));      // join i2c bus with addresses #2 and #3
  	Wire.onAddrReceive(addressEvent); // register event
  	Wire.onRequest(requestEvent);     // register event
	Wire.onReceive(receiveEvent);
	Wire.onStop(stopEvent);

	si.i2c_init(); //Begin I2C master

}

void loop() {

	if(StartFlag && !RestartFlag) {
  Serial.println("S"); //DEBUG!
		si.i2c_start((slaveAddr << 1) | RW);  //Start comunication
		StartFlag = false; //Clear flag
	}

	// if(DataInFlag) {
	// 	//Write data out to devices
		
	// }

	if(DataOutFlag) {
  Serial.println((Dummy[0] << 8) | Dummy[1]); //DEBUG!
  Serial.print("DO"); Serial.print(RestartFlag); Serial.println(StopFlag);//DEBUG!
		//Read data out to master
    Wire.write(0x03); //DEBUG!
    Wire.write(0x03); //DEBUG!
		uint8_t Val = 0; //Local val to pass data
		if(RestartFlag) si.i2c_write(DataIn[0]); //Write register value
		while(!StopFlag) {  //Wait for stop flag
			Val = si.i2c_read(true); //Read from sub-slave
			Wire.write(Val);  //Write to master
     Serial.println(Val); //DEBUG! 
		}
    RestartFlag = false; //Clear flag after use 
		DataOutFlag = false; //Clear data out flag
    DataInFlag = false; //Clear in case of a restarted sequence
	}

//	if(StopFlag && DataInFlag) {  //Do not write data out until stop flag, since could be a multiple start sequence
    if(DataInFlag) { //DEBUG!
		//End I2C communication 
   Serial.print("DI"); Serial.println(NumBytes); //DEBUG!
		for(int i = 0; i < NumBytes; i++) {
			si.i2c_write(DataIn[i]);
      Serial.println(DataIn[i]); //DEBUG!
		}
		si.i2c_stop();  //Stop comunication
		DataInFlag = false; //Clear data out flag
		StopFlag = false; //Clear stop flag	
    Serial.println("ST_C"); //DEBUG!
	}

	if(StopFlag && !DataInFlag && !DataOutFlag) {  //Stop only if all other flags are cleared
    Serial.println("ST"); //DEBUG!
		si.i2c_stop(); //Stop communication
		StopFlag = false; //Clear flag after stopping 
	}

 if(DataInFlag) {
    Serial.println("DI_B"); //DEBUG!
 }

 if(StopFlag) {
    Serial.println("ST_B"); //DEBUG!
 }
	// delay(10);
}

boolean addressEvent(uint16_t address, uint8_t count)
{
  RW = (address & 0x01); //Test for read/write
	if(count == 0) StartFlag = true; //Set flag to start comunication to slave
//StartFlag = true; //DEBUG!
	if(count > 0) RestartFlag = true;  //Set read flag after multiple
//	slaveAddr = AddressRemap[address - ADDR]; //Convert input address to on board address
  slaveAddr = 0x49; //DEBUG!
	return true; // send ACK to master
}

void requestEvent()
{
	DataOutFlag = true;
  uint8_t Val = 0x03; //Local val to pass data
  uint8_t Counter = 0; //DEBUG!
//  if(RestartFlag) si.i2c_write(DataIn[0]); //Write register value
//  while(!StopFlag) {  //Wait for stop flag
//    Val = si.i2c_read(true); //Read from sub-slave
//    Wire.write(Val);  //Write to master
////    Dummy[Counter++] = Val; //DEBUG!
////   Serial.println(Val); //DEBUG! 
//  }
  digitalWrite(PIN_A4, LOW); //Hold clock
  uint8_t Val1 = si.i2c_read(true);
  uint8_t Val2 = si.i2c_read(true);
  digitalWrite(PIN_A4, HIGH); //Release clock
  Wire.write(Val1);
  Wire.write(Val2);
//	while(Wire.available() == 0); //Wait until bytes are available
	// Reg = Wire.read(); //Read value of register to read from

	// switch (slaveAddr) {
	// case 0x64:
	// 	//Addess 0x64
	// 	break;

	// case 0x65:
	// 	//Address 0x65
	// 	break;

	// case 0x66:
	// 	//Address 0x66
	// 	break;

	// case 0x67:
	// 	//Address 0x67
	// 	break;
 //  }
}

void receiveEvent(int DataLen) {
    //Write data to appropriate location
    NumBytes = DataLen; //Copy length val

    while(Wire.available() == 0);  //Wait for data to arrive  //DEBUG!
    for(int i = 0; i < NumBytes; i++) {  //Read data into register
    	DataIn[i] = Wire.read();
    }

    DataInFlag = true; //Set flag for output
 //    switch (slaveAddr) {
	// case 0x64:
	// 	//Addess 0x64
	// 	break;

	// case 0x65:
	// 	//Address 0x65
	// 	break;

	// case 0x66:
	// 	//Address 0x66
	// 	break;

	// case 0x67:
	// 	//Address 0x67
	// 	break;
 //  }
}

void stopEvent() {
	StopFlag = true; //Set flag to end comunication
}



