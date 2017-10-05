EEPROM (24XX512)
================

![travis](https://travis-ci.org/kanewallmann/eeprom-24XX512-arduino.svg?branch=master)

An Arduino library to read/write data to/from Microchip 24XX512 EEPROMs via their i2c interface.

Yes, it is easy to use the Wire library directly. Or the EEPROM_I2C library. But this library adds some helper
functions and templates to make it even easier to do more complicated things such as reading and writing an array
of arbitrary data types. It also transparently handles the internal 128 byte write buffer of the 24XX512.

Datasheet available here: http://ww1.microchip.com/downloads/en/DeviceDoc/21754M.pdf

Wiring
==========

|24XX512|Arduino|
|:-----:|:-----:|
|SDA    |SDA    |
|SCL    |SCL    |

WP may be optionally used for write protect functionality. Connect to any digital pin and pass the pin number to the
constructor.

A2, A1, and A0 determine the last 3 bits of the i2c address. The first are 1010. Grounding all 3 pins results in an i2c
address of 0b1010000(7bit) or 0x50 in hex. This address must be supplied to the constructor.

Important Notes
============

This library uses the Wire library from Arduino. You must call Wire.begin() before attempting to use this library. You
may also select the i2c bus speed by calling Wire.setClock()

Usage
=======

View the examples folder for examples on usage.

License
========

See LICENCE.txt for license (MIT)
