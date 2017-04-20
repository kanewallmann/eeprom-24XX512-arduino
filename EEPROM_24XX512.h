/* Part of EEPROM (24XX512) Arduino library
 * Copyright (C) 2017 Kane Wallmann
 * See LICENCE.txt for license (MIT)*/

#ifndef __EEPROM_24XX512_H
#define __EEPROM_24XX512_H

#include <Arduino.h>
#include <Wire.h>

#define EEPROM_24XX512_WRITE_BUFFER_SIZE   	 (128)        // Internal write buffer size per data sheet
#define EEPROM_24XX512_MAX_ADDR              (64000)      // Max memory size per data sheet

typedef enum
{
	STATE_RANDOM,
	STATE_READ,
	STATE_WRITE,
}
		EepromState;

/**
 * A class to read and write data to and from a Microchip 24XX512 or compatible EEPROM IC. This class might
 * work with other i2c compatible EEPROMs but has not been tested with anything but the one mentioned.
 *
 * This class features a sequential read/write operating mode which is more efficient than random read/writes.
 *
 * Use Wire.setClock() to set the speed of the i2c bus.
 *
 * Family devices:	24AA512, 24LC512, 24FC512
 * Prerequisites: 	1.  Pins SCL and SDA on EEPROM connected to pins SCL and SDA on Arduino
 * 					1a. If not using Write Protect function, tie WP to low to enable writing
 * 					2.  Call Wire.begin() before beginRead() or beginWrite()
 *
 * Usage:
 *
 * EEPROM_24XX512 eeprom( 0x50 );		// i2c address pins all low
 *
 * Wire.begin();
 *
 * int someInts[5] = { 1, 2, 3, 4, 5 };
 *
 * // Sequential writes
 * eeprom.beginWrite( 0 );
 * eeprom.write( someInts[0] );
 * eeprom.write( someInts[1] );
 * eeprom.writeArray( someInts+3, 3 );
 * eeprom.endWrite();
 *
 * // Random write
 * eeprom.write( 6, (int)6 );
 *
 * // EEPROM now has { 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006 } at address 0
 *
 * byte someBytes[4];
 *
 * // Random read from address 0
 * eeprom.readArray( 0, someBytes, 4 );
 *
 * // someBytes now contains { 0x00, 0x01, 0x00, 0x02 }
 */
class EEPROM_24XX512
{
public:
	/**
	 * If A2,A1 and A0 pins are all low i2c_addr value should be 0b1010000 or 0x50
	 *
	 * @param i2c_addr 		A 7 bit i2c address, should be 0b1010AAA where AAA is pins A2,A1 and A0 on the IC
	 */
	EEPROM_24XX512(int i2c_addr);

	/**
	 * If A2,A1 and A0 pins are all low i2c_addr value should be 0b1010000 or 0x50
	 *
	 * You must call setWriteProtect( false ) if you wish to write to the chip after calling this constructor
	 *
	 * @param i2c_addr 		A 7 bit i2c address, should be 0b1010AAA where AAA is pins A2,A1 and A0 on the IC
	 * @param pin_wp 		Pin connected to WP on the EEPROM, enables control of Write Protect function
	 */
	EEPROM_24XX512(int i2c_addr, int pin_wp);

	/**
	 * Start a write session
	 *
	 * Calls to write() and writeArray() between this method and endWrite() will be sequential
	 *
	 * @param addr 			The address of the first byte written
	 */
	void beginWrite(int addr);

	/**
	 * End a sequential write session
	 */
	void endWrite();

	/**
	 * Start a read session
	 *
	 * Calls to read() and readArray() between this method and endRead() will be sequential
	 *
	 * @param addr 			The address of the first byte to read
	 */
	void beginRead(int addr);

	/**
	 * Ends a sequential read session
	 */
	void endRead();

	/**
	 * Enables or disables chip-level write protection
	 *
	 * If pin_wp wasn't supplied in the constructor of this object, this method does nothing
	 *
	 * @param enabled 	If true, writes have no effect
	 */
	void setWriteProtect(bool enabled);

	/**
	 * Read some data from EEPROM address, not to be called between beginRead() and endRead()
	 *
	 * @tparam T 		The type of data to read
	 * @param address	The EEPROM address to read from
	 * @param data 		Pointer to memory containing at least count * elements to read from
	 * @param count 	Number of elements to read
	 * @return 			Returns number of elements actually read, may be less than requested due to size restraint
	 */
	template<typename T>
	int readArray(int address, T *data, size_t count);

	/**
	 * Read some data, must be called between beginRead() and endRead()
	 *
	 * @tparam T 		The type of data to read
	 * @param data 		Pointer to memory containing at least count * elements to read from
	 * @param count 	Number of elements to read
	 * @return 			Returns number of elements actually read, may be less than requested due to size restraint
	 */
	template<typename T>
	int readArray(T *data, size_t count);

	/**
	 * Writes some data, not to be called between beginWrite() and endWrite()
	 *
	 * @tparam T 		The type of data to write
	 * @param address	The EEPROM address to write to
	 * @param data 		Points to memory containing at least count * elements to write to
	 * @param count 	Number of elements to write
	 * @return 			Returns the number of elements actually written, may be less than requested due to size restraint
	 */
	template<typename T>
	int writeArray(int address, const T *data, size_t count);

	/**
	 * Writes some data, must be called between beginWrite() and endWrite()
	 *
	 * @tparam T 		The type of data to write
	 * @param data 		Points to memory containing at least count * elements to write to
	 * @param count 	Number of elements to write
	 * @return 			Returns the number of elements actually written, may be less than requested due to size restraint
	 */
	template<typename T>
	int writeArray(const T *data, size_t count);

	/**
	 * Reads a single T from current address pointer, not to be called between beginRead() and endRead()
	 *
	 * @tparam T		The type of data to read
	 * @param address	The EEPROM address to read from
	 * @param data		The memory location to store the result
	 * @return			true on successful read, false if there wasn't enough memory to read the full type
	 */
	template<typename T>
	bool read(int address, T *data);

	/**
	 * Reads a single T from current address pointer, must be called between beginRead() and endRead()
	 *
	 * @tparam T		The type of data to read
	 * @param data		The memory location to store the result
	 * @return			true on successful read, false if there wasn't enough memory to read the full type
	 */
	template<typename T>
	bool read(T *data);

	/**
	 * Writes a single T to the current address point, must be called between beginWrite() and endWrite()
	 *
	 * @tparam T		The type of data to write
	 * @param address	The EEPROM address to write to
	 * @param data		The data to write
	 * @return			true on successful write, false if there wasn't enough space to write
	 */
	template<typename T>
	bool write(int addr, const T data);

	/**
	 * Writes a single T to the current address point, must be called between beginWrite() and endWrite()
	 *
	 * @tparam T	The type of data to write
	 * @param data	The data to write
	 * @return		true on successful write, false if there wasn't enough space to write
	 */
	template<typename T>
	bool write(const T data);

private:
	int i2c_addr;
	int write_count;
	int write_addr;
	int read_addr;
	bool wp;
	int pin_wp;
	EepromState state;
};

template<typename T>
int EEPROM_24XX512::readArray(int address, T *d, size_t count)
{
	if( this->state != STATE_RANDOM )
	{
		return 0;
	}

	this->beginRead(address);
	int result = this->readArray(d, count);
	this->endRead();
	return result;
}

template<typename T>
int EEPROM_24XX512::readArray(T *d, size_t count)
{
	if( this->state != STATE_READ )
	{
		return 0;
	}

	size_t a = 0;
	int c = 0;
	byte *byte_data = (byte *) d;

	// Check if we can read count * elements with the remaining memory
	size_t bytes_remaining = EEPROM_24XX512_MAX_ADDR - this->read_addr;
	size_t element_count = min(count, bytes_remaining / sizeof(T));

	Wire.requestFrom(this->i2c_addr, sizeof(T) * element_count, false);

	// While there is enough bytes left to construct one element
	for( a = 0; a < element_count; a++ )
	{
		// Iterate over each byte in the element shift it into position and xor it to the zeroed out memory
		for( size_t b = 0; b < sizeof(T); b++ )
		{
			if( Wire.available())
			{
				byte_data[c] = Wire.read();
				c++;

				// Increment read address counter
				this->read_addr++;
			}
			else
			{
				// Not enough bytes available to read a full element?
				return a;
			}
		}
	}

	// Return number of elements read
	return a;
}

template<typename T>
int EEPROM_24XX512::writeArray(int address, const T *d, size_t count)
{
	if( this->state != STATE_RANDOM )
	{
		return 0;
	}

	this->beginWrite(address);
	int result = this->writeArray(d, count);
	this->endWrite();
	return result;
}

template<typename T>
int EEPROM_24XX512::writeArray(const T *d, size_t count)
{
	if( this->state != STATE_WRITE )
	{
		return 0;
	}

	// Make sure there is enough memory to read count * elements
	int bytes_available = EEPROM_24XX512_MAX_ADDR - this->write_addr;
	int element_count = min(count, bytes_available / sizeof(T));

	int c = 0;
	const byte *byte_data = (const byte *) d;

	// Increment through number of data to write
	for( int a = 0; a < element_count; a++ )
	{
		// Increment through number of bytes in element
		for( size_t b = 0; b < sizeof(T); b++ )
		{
			// Write a byte of the data and increment counter
			Wire.write(byte_data[c]);
			c++;
			this->write_count++;

			// If we have filled the write buffer (128 bytes), we have to send stop bit to flush then start again
			if( this->write_count == EEPROM_24XX512_WRITE_BUFFER_SIZE)
			{
				this->endWrite();
				this->beginWrite(this->write_addr + this->write_count);

				// this->beginWrite() resets write_count and updates this->write_addr
			}
		}
	}

	// Update our write address counter
	this->write_addr += this->write_count;

	return element_count;
}

template<typename T>
bool EEPROM_24XX512::read(int address, T *data)
{
	if( this->state != STATE_RANDOM )
	{
		return false;
	}

	this->beginRead(address);
	bool result = this->readArray(data, 1) == 1;
	this->endRead();
	return result;
}

template<typename T>
bool EEPROM_24XX512::read(T *data)
{
	return this->readArray(data, 1) == 1;
}

template<typename T>
bool EEPROM_24XX512::write(int address, const T data)
{
	if( this->state != STATE_RANDOM )
	{
		return false;
	}

	this->beginWrite(address);
	bool result = this->writeArray(&data, 1) == 1;
	this->endWrite();
	return result;
}

template<typename T>
bool EEPROM_24XX512::write(const T data)
{
	return this->writeArray(&data, 1) == 1;
}

#endif //__EEPROM_24XX512_H
