#include <Wire.h>
#include <EEPROM_24XX512.h>

// 0x05 is 0b101000 (7bit), the last three zeros are the state of A2, A1, and A0
EEPROM_24XX512 eeprom( 0x50 );

void setup()
{
	// Must call before using eeprom
	Wire.begin();

	// Random write command
	eeprom.write( 0, (long)0x1234567 );

	// Random read command
	long rd;
	eeprom.read(0,&rd);

	// Sequential write
	eeprom.beginWrite( 0 );
	eeprom.write( (int)10 );
	eeprom.write( (int)20 );
	eeprom.write( (int)30 );
	eeprom.endWrite();

	// Sequential read
	int a, b, c;
	eeprom.beginRead( 0 );
	eeprom.read( &a );
	eeprom.read( &b );
	eeprom.read( &c );
	eeprom.endRead();

	// Array write
	int data[5] = { 10, 20, 30, 40, 50 };
	eeprom.writeArray( 0, data, 5 );

	// Array read
	eeprom.readArray( 0, data, 5 );
}

void loop()
{

}
