/***************************************************
  This is a library for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1748
  ----> https://www.adafruit.com/products/1749

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MLX90614.h"

Adafruit_MLX90614::Adafruit_MLX90614(uint8_t i2caddr) {
  _addr = i2caddr;
}


boolean Adafruit_MLX90614::begin(void) {
  Wire.begin();
  return true;
}

//////////////////////////////////////////////////////

double Adafruit_MLX90614::readObjectTempF(void) {
  return (readTemp(MLX90614_TOBJ1) * 9 / 5) + 32;
}


double Adafruit_MLX90614::readAmbientTempF(void) {
  return (readTemp(MLX90614_TA) * 9 / 5) + 32;
}

double Adafruit_MLX90614::readObjectTempC(void) {
  return readTemp(MLX90614_TOBJ1);
}


double Adafruit_MLX90614::readAmbientTempC(void) {
  return readTemp(MLX90614_TA);
}


void Adafruit_MLX90614::readRawTemps(uint16_t *data){
  data[0] = read16(MLX90614_TA); //load the ambient temp
  data[1] = read16(MLX90614_TOBJ1); //load the object temp
}

void Adafruit_MLX90614::readEPPROM(void){
  for (uint8_t i=0x20; i<0x3F; i++) {
    Serial.print(i, HEX); Serial.print(" = ");
    Serial.println(read16(i), HEX);
  }
}

uint8_t Adafruit_MLX90614::setAddress(uint8_t new_address){
  //note the device address 0x00 means all devices will respond / listen
  write16(0x00, MLX90614_ADDR,  0x00); //write zeros to clear the epprom
  delay(150); //wait for the EEPROM to respond per the datasheet
  write16(0x00, MLX90614_ADDR, new_address);//write the new address
  delay(100); //see above

  return new_address == read16(MLX90614_ADDR);
}

/*********************************************************************/
float Adafruit_MLX90614::readTemp(uint8_t reg) {
  float temp;

  temp = read16(reg);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

//grabbed from https://chromium.googlesource.com/chromiumos/platform/vboot_reference/+/master/firmware/lib/crc8.c
uint8_t Adafruit_MLX90614::crc8(const uint8_t *ptr, int len)
{
	const uint8_t *data = ptr;
	unsigned crc = 0;
	int i, j;
	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for(i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}
	return (uint8_t)(crc >> 8);
}

uint16_t Adafruit_MLX90614::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(false); // end transmission

  Wire.requestFrom(_addr, (uint8_t)3);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret |= Wire.read() << 8; // receive DATA

  uint8_t pec = Wire.read();

  return ret;
}

//slave addresss, device address, data
void Adafruit_MLX90614::write16(uint8_t address, uint8_t command, uint16_t data){
  uint8_t lsb = data;
  uint8_t msb = data >> 8;
  uint8_t array[3];
  array[0] = command;
  array[1] = lsb;
  array[2] = msb;
  uint8_t crc = crc8(array, 3);

  Wire.beginTransmission(address);
  Wire.write(array[0]);
  Wire.write(array[1]);
  Wire.write(array[2]);
  Wire.write(crc);
  Wire.endTransmission();
}
