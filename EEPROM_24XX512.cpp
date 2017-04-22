/* Part of EEPROM (24XX512) Arduino library
 * Copyright (C) 2017 Kane Wallmann
 * See LICENCE.txt for license (MIT)*/

#include "EEPROM_24XX512.h"

EEPROM_24XX512::EEPROM_24XX512(int i2c_addr) : i2c_addr(i2c_addr), state(STATE_RANDOM)
{
	this->wp = false;
}

EEPROM_24XX512::EEPROM_24XX512(int i2c_addr, int pin_wp) : i2c_addr(i2c_addr), pin_wp(pin_wp), state(STATE_RANDOM)
{
	this->wp = true;
	pinMode(pin_wp, OUTPUT);

	// Start with WP on
	this->setWriteProtect(true);
}

void EEPROM_24XX512::beginWrite(int addr)
{
	this->write_count = 0;
	this->write_addr = addr;
	this->state = STATE_WRITE;

	Wire.beginTransmission(this->i2c_addr);
	Wire.write(byte(addr >> 8));
	Wire.write(byte(addr & 0x00FF));
}

void EEPROM_24XX512::endWrite()
{
	Wire.endTransmission();
	this->state = STATE_RANDOM;
}

void EEPROM_24XX512::beginRead(int addr)
{
	this->read_addr = addr;
	this->state = STATE_READ;

	Wire.beginTransmission(this->i2c_addr);
	Wire.write(byte(addr >> 8));
	Wire.write(byte(addr & 0x00FF));
	Wire.endTransmission();
}

void EEPROM_24XX512::endRead()
{
	Wire.requestFrom(this->i2c_addr, 1);
	this->state = STATE_RANDOM;
}

void EEPROM_24XX512::setWriteProtect(bool enabled)
{
	if( this->wp )
	{
		digitalWrite(this->pin_wp, enabled ? HIGH : LOW);
	}
}
