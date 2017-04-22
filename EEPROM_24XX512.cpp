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

void EEPROM_24XX512::beginWrite(unsigned int address)
{
	if( this->state != STATE_RANDOM )
	{
		return;
	}

	this->write_count = 0;
	this->write_addr = address;
	this->state = STATE_WRITE;

	Wire.beginTransmission(this->i2c_addr);
	Wire.write(byte(address >> 8));
	Wire.write(byte(address & 0x00FF));
}

void EEPROM_24XX512::endWrite()
{
	if( this->state != STATE_WRITE )
	{
		return;
	}

	Wire.endTransmission();
	this->state = STATE_RANDOM;
}

void EEPROM_24XX512::beginRead(unsigned int address)
{
	if( this->state != STATE_RANDOM )
	{
		return;
	}

	this->read_addr = address;
	this->state = STATE_READ;

	Wire.beginTransmission(this->i2c_addr);
	Wire.write(byte(address >> 8));
	Wire.write(byte(address & 0x00FF));
	Wire.endTransmission();
}

void EEPROM_24XX512::endRead()
{
	if( this->state != STATE_READ )
	{
		return;
	}

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

unsigned int EEPROM_24XX512::writeBytes(unsigned int address, const byte* byte_data, unsigned int count)
{
	if( this->state != STATE_RANDOM )
	{
		return 0;
	}

	this->beginWrite(address);
	unsigned int result = this->writeBytes(byte_data, count);
	this->endRead();
	return result;
}

unsigned int EEPROM_24XX512::writeBytes(const byte* byte_data, unsigned int count)
{
	if( this->state != STATE_WRITE )
	{
		return 0;
	}

	unsigned int bytes_available = EEPROM_24XX512_MAX_ADDR - this->write_addr;
	count = min(count, bytes_available);

	// Increment through number of data to write
	for( unsigned int a = 0; a < count; a++ )
	{
		// Write a byte of the data and increment counter
		Wire.write(byte_data[a]);
		this->write_count++;

		// If we have filled the write buffer (128 bytes), we have to send stop bit to flush then start again
		if( this->write_count == EEPROM_24XX512_WRITE_BUFFER_SIZE )
		{
			this->endWrite();
			this->beginWrite(this->write_addr + this->write_count);

			// this->beginWrite() resets write_count and updates this->write_addr
		}
	}

	// Update our write address counter
	this->write_addr += this->write_count;

	return count;
}

unsigned int EEPROM_24XX512::readBytes(byte* byte_data, unsigned int count)
{
	if( this->state != STATE_READ )
	{
		return 0;
	}

	// Check if enough bytes are remaining to read this much data
	unsigned int bytes_remaining = EEPROM_24XX512_MAX_ADDR - this->read_addr;
	count = min(count, bytes_remaining);

	Wire.requestFrom(this->i2c_addr, count, false);

	for( unsigned int a = 0; a < count; a++ )
	{
		if( Wire.available() )
		{
			byte_data[a] = Wire.read();

			// Increment read address counter
			this->read_addr++;
		}
		else
		{
			return a;
		}
	}

	// Return number of elements read
	return count;
}

bool EEPROM_24XX512::isReady()
{
	if( this->state != STATE_RANDOM )
	{
		return false;
	}

	Wire.beginTransmission(this->i2c_addr);
	Wire.write(byte(0));
	Wire.write(byte(0));
	return Wire.endTransmission() == 0;
}
