/* Part of EEPROM (24XX512) Arduino library
 * Copyright (C) 2017 Kane Wallmann
 * See LICENCE.txt for license (MIT)*/

#ifndef __EEPROM_24XX512_H
#define __EEPROM_24XX512_H

#include <Arduino.h>
#include <Wire.h>

#define EEPROM_24XX512_WRITE_BUFFER_SIZE     (128)        // Internal write buffer size per data sheet
#define EEPROM_24XX512_MAX_ADDR              (64000u)      // Max memory size per data sheet

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
	void beginWrite(unsigned int address);

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
	void beginRead(unsigned int address);

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
	unsigned int readArray(unsigned int address, T* data, unsigned int count);

	/**
	 * Read some data, must be called between beginRead() and endRead()
	 *
	 * @tparam T 		The type of data to read
	 * @param data 		Pointer to memory containing at least count * elements to read from
	 * @param count 	Number of elements to read
	 * @return 			Returns number of elements actually read, may be less than requested due to size restraint
	 */
	template<typename T>
	unsigned int readArray(T* data, unsigned int count);

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
	unsigned int writeArray(unsigned int address, const T* data, unsigned int count);

	/**
	 * Writes some data, must be called between beginWrite() and endWrite()
	 *
	 * @tparam T 		The type of data to write
	 * @param data 		Points to memory containing at least count * elements to write to
	 * @param count 	Number of elements to write
	 * @return 			Returns the number of elements actually written, may be less than requested due to size restraint
	 */
	template<typename T>
	unsigned int writeArray(const T* data, unsigned int count);

	/**
	 * Reads a single T from current address pointer, not to be called between beginRead() and endRead()
	 *
	 * @tparam T		The type of data to read
	 * @param address	The EEPROM address to read from
	 * @param data		The memory location to store the result
	 * @return			true on successful read, false if there wasn't enough memory to read the full type
	 */
	template<typename T>
	bool read(unsigned int address, T* data);

	/**
	 * Reads a single T from current address pointer, must be called between beginRead() and endRead()
	 *
	 * @tparam T		The type of data to read
	 * @param data		The memory location to store the result
	 * @return			true on successful read, false if there wasn't enough memory to read the full type
	 */
	template<typename T>
	bool read(T* data);

	/**
	 * Writes a single T to the current address point, must be called between beginWrite() and endWrite()
	 *
	 * @tparam T		The type of data to write
	 * @param address	The EEPROM address to write to
	 * @param data		The data to write
	 * @return			true on successful write, false if there wasn't enough space to write
	 */
	template<typename T>
	bool write(unsigned int address, const T data);

	/**
	 * Writes a single T to the current address point, must be called between beginWrite() and endWrite()
	 *
	 * @tparam T	The type of data to write
	 * @param data	The data to write
	 * @return		true on successful write, false if there wasn't enough space to write
	 */
	template<typename T>
	bool write(const T data);

	/**
	 * Writes bytes to the supplied address, not to be called between beginWrite() and endWrite()
	 *
	 * @param address 	The address to write to
	 * @param data 		Pointer to count * bytes to write
	 * @param count 	Number of bytes to write
	 * @return 			Number of bytes actually written, might be different from requested due to size restraints
	 */
	unsigned int writeBytes(unsigned int address, const byte* data, unsigned int count);

	/**
	 * Writes bytes to the current address pointer, must be called between beginWrite() and endWrite()
	 *
	 * @param data 		Pointer to count * bytes to write
	 * @param count 	Number of bytes to write
	 * @return 			Number of bytes actually written, might be different from requested due to size restraints
	 */
	unsigned int writeBytes(const byte* data, unsigned int count);

	/**
	 * Reads byte from the supplied address, not to be called between beginRead() and endRead()
	 *
	 * @param address 	Address to read from
	 * @param data 		Pointer to count * bytes to read the data into
	 * @param count 	Number of bytes to read
	 * @return 			Number of bytes actually read, might be different from requested due to size restraints
	 */
	unsigned int readBytes(unsigned int address, byte* data, unsigned int count);

	/**
	 * Reads byte from the current address pointer, must be called between beginRead() and endRead()
	 *
	 * @param data 		Pointer to count * bytes to read the data into
	 * @param count 	Number of bytes to read
	 * @return 			Number of bytes actually read, might be different from requested due to size restraints
	 */
	unsigned int readBytes(byte* data, unsigned int count);

	/**
	 * Attempts to perform a write and checks if device returns an ACK. This test process determines if the device is
	 * ready to receive new read/write commands following a write
	 *
	 * This method should not be called between beginWrite()/endWrite() or beginRead()/endRead()
	 *
	 * @return		True if the device is ready
	 */
	bool isReady();

private:
	int i2c_addr;
	unsigned int write_count;
	unsigned int write_addr;
	unsigned int read_addr;
	bool wp;
	int pin_wp;
	EepromState state;
};

template<typename T>
unsigned int EEPROM_24XX512::readArray(unsigned int address, T* d, unsigned int count)
{
	if( this->state != STATE_RANDOM )
	{
		return 0;
	}

	this->beginRead(address);
	unsigned int result = this->readArray(d, count);
	this->endRead();
	return result;
}

template<typename T>
unsigned int EEPROM_24XX512::readArray(T* d, unsigned int count)
{
	if( this->state != STATE_READ )
	{
		return 0;
	}

	unsigned int bytes_available = EEPROM_24XX512_MAX_ADDR - this->read_addr;
	unsigned int element_count = bytes_available / sizeof(T);
	element_count = min(count, element_count);
	unsigned int bytes_read = this->readBytes((byte*) d, element_count * sizeof(T));
	return bytes_read / sizeof(T);
}

template<typename T>
unsigned int EEPROM_24XX512::writeArray(unsigned int address, const T* d, unsigned int count)
{
	if( this->state != STATE_RANDOM )
	{
		return 0;
	}

	this->beginWrite(address);
	unsigned int result = this->writeArray(d, count);
	this->endWrite();
	return result;
}

template<typename T>
unsigned int EEPROM_24XX512::writeArray(const T* d, unsigned int count)
{
	if( this->state != STATE_WRITE )
	{
		return 0;
	}

	unsigned int bytes_available = EEPROM_24XX512_MAX_ADDR - this->write_addr;
	unsigned int element_count = bytes_available / sizeof(T);
	element_count = min(count, element_count);
	unsigned int bytes_read = this->writeBytes((byte*) d, element_count * sizeof(T));
	return bytes_read / sizeof(T);
}

template<typename T>
bool EEPROM_24XX512::read(unsigned int address, T* data)
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
bool EEPROM_24XX512::read(T* data)
{
	return this->readArray(data, 1) == 1;
}

template<typename T>
bool EEPROM_24XX512::write(unsigned int address, const T data)
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
