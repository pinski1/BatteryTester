/** Library for LTC4151 
 * http://cds.linear.com/docs/en/datasheet/4151ff.pdf
 */
#include <Arduino.h>
#include <Wire.h>

/** Registers */
#define REG_SENSE_A		0x00 // R/W** ADC Current Sense Voltage Data (8 MSBs)
#define REG_SENSE_B		0x01 // R/W** ADC Current Sense Voltage Data (4 LSBs)
#define REG_VIN_C		0x02 // R/W** ADC VIN Voltage Data (8 MSBs)
#define REG_VIN_D		0x03 // R/W** ADC VIN Voltage Data (4 LSBs)
#define REG_ADIN_E		0x04 // R/W** ADC ADIN Voltage Data (8 MSBs)
#define REG_ADIN_F		0x05 // R/W** ADC ADIN Voltage Data (4 LSBs)
#define REG_CONTROL_G	0x06 // R/W Controls ADC Operation Mode and Test Mode
// all others are ignored
#define MASK_BUSY_BIT	0x0008

/** Constants */
#define AIN_LSB			20 // uV
#define VIN_LSB			25000 // uV
#define XIN_LSB			500 // uV

class LTC4151 {
private:
	unsigned int _senseResistor;
public:
	LTC4151();
	void begin();
	void setSenseResistor(unsigned int senseResistor);
	unsigned int getVoltage(void);
	unsigned int getCurrent(void);
	unsigned int getAux(void);


};


void LTC4151::setSenseResistor(unsigned int senseResistor) {
	_senseResistor = senseResistor;
}

unsigned int LTC4151::getVoltage(void) {
	
	unsigned int readVoltage;
	
	// read AIN registers
	Wire.beginTransmission(_address);
	Wire.write(byte(REG_VIN_C));
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	if( 2 <= Wire.available())
	{
		readVoltage = Wire.read();
		readVoltage = (readVoltage << 8);
		readVoltage = readVoltage | Wire.read();
	}
	
	// assemble voltage across resistor
	if(readVoltage & MASK_BUSY_BIT)// if bit[3] 1 = busy, 0 complete
	readVoltage >>= 4;
	
	// multiply by VIN_LSB uV
	readVoltage *= VIN_LSB;
	
	// covert to millivolts
	readVoltage /= 1000;
	
	return readVoltage;
}

unsigned int LTC4151::getCurrent(void) {
	
	unsigned int readCurrent;
	
	// read AIN registers
	Wire.beginTransmission(_address);
	Wire.write(byte(REG_SENSE_A));
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	if( 2 <= Wire.available())
	{
		readCurrent = Wire.read();
		readCurrent = (readCurrent << 8);
		readCurrent = readCurrent | Wire.read();
	}
	
	// assemble voltage across resistor
	if(readCurrent & MASK_BUSY_BIT)// if B3 1 = busy, 0 complete
	readCurrent >>= 4;
	
	// multiply by AIN_LSB uV
	readCurrent *= AIN_LSB;
	
	// divide by _senseResistor
	readCurrent /= _senseResistor;
	
	
	return readCurrent;
}

unsigned int LTC4151::getAux(void) {
	
	unsigned int readAux;
	
	// read ADIN registers
	Wire.beginTransmission(_address);
	Wire.write(byte(REG_ADIN_E));
	Wire.endTransmission();
	Wire.requestFrom(_address, 2);
	if( 2 <= Wire.available())
	{
		readAux = Wire.read();
		readAux = (readAux << 8);
		readAux = readAux | Wire.read();
	}
	
	// assemble voltage across resistor
	if(readAux & MASK_BUSY_BIT)// if B3 1 = busy, 0 complete
	readAux >>= 4;
	
	// multiply by XIN_LSB uV
	readAux *= XIN_LSB;
	
	// convert to millivolts
	readAux /= 1000;
	
	
	return readAux;
}



