/******************************************************************************
SCMD.cpp
SCMD Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/Serial_Controlled_Motor_Driver
https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
<arduino/development environment version>
<hardware version>
<etc>

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/
//Define USE_ALT_I2C to use the teesny 3 i2c_t3.h library, which allows I2C bus hang resolution
// THIS MUST BE DONE WITHIN THIS FILE.  Users wishing to do this will probably find themselves
// copying SCMD.cpp, SCMD.h, and SCMD_config.h to their project folder and excluding the arduino
// library form the IDE entirely.

//#define USE_ALT_I2C

//Use VERBOSE_SERIAL to add debug serial to an existing Serial object.
//Note:  Use of VERBOSE_SERIAL adds delays surround RW ops, and should not be used
//for functional testing.
//#define VERBOSE_SERIAL

//See _____ for additional topology notes.
#include <iostream>
#include <stdint.h>
#include <math.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values

#include <wiringPiI2C.h>

//****************************************************************************//
//
//  Constructor
//
//    Initalizes settings to default.
//
//****************************************************************************//
SCMD::SCMD( void )
{
	//Construct with these default settings if nothing is specified
	//Select default I2C address.
	settings.I2CAddress = 0x58;
}


//****************************************************************************//
//
//  Configuration section
//
//    This uses the stored SensorSettings to start the IMU.
//
//  Example usage:
//  //Configure for SPI mode using default CS pin of 10
//  mySensor.settings.commInterface = SPI_MODE;
//  mySensor.begin();
//
//****************************************************************************//
uint8_t SCMD::begin( void )
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	m_fd = wiringPiI2CSetup(settings.I2CAddress);
	std::cout << "Init result: "<< m_fd << '\n';

	//dummy read
	readRegister(SCMD_ID);

	return readRegister(SCMD_ID);
}

//check if enumeration is complete
bool SCMD::ready( void )
{
	uint8_t statusByte = readRegister(SCMD_STATUS_1);
	if(( statusByte & SCMD_ENUMERATION_BIT )&&(( statusByte != 0xFF )))//wait for ready flag and not 0xFF
	{
		return true;
	}
	else
	{
#ifdef VERBOSE_SERIAL
		std::cout << "-";
#endif
		return false;
	}

}

//check if SCMD is busy
bool SCMD::busy( void )
{
	uint8_t statusByte = readRegister(SCMD_STATUS_1);
	if( statusByte & (SCMD_BUSY_BIT | SCMD_REM_READ_BIT | SCMD_REM_WRITE_BIT))
	{
#ifdef VERBOSE_SERIAL
		std::cout << ".";
#endif
		return true;
	}
	else
	{
		return false;
	}

}
//Enable and disable functions.  Call after begin to enable the h-bridges
void SCMD::enable( void )
{
	writeRegister(SCMD_DRIVER_ENABLE, 0x01);
}

void SCMD::disable( void )
{
	writeRegister(SCMD_DRIVER_ENABLE, 0x00);
}

//Reset function
void SCMD::reset( void )
{
	// NOT IMPLEMENTED
}
//****************************************************************************//
//
//  Drive Section
//
//****************************************************************************//

//setDrive( ... )
//
//    Drive a motor at a level
//
//  uint8_t motorNum -- Motor number from 0 to 33
//  uint8_t direction -- 0 or 1 for forward and backward
//  uint8_t level -- 0 to 255 for drive strength
void SCMD::setDrive( uint8_t motorNum, uint8_t direction, uint8_t level )
{
	//convert to 7 bit
	level = level >> 1;
	int16_t driveValue; //use to build value to actually write to register

	//Make sure the motor number is valid
	if(motorNum < 34)
	{
		driveValue = (level * direction) + ((int8_t)level * ((int8_t)direction - 1)); //set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0; (level * direction);
		driveValue += 128;
		writeRegister(SCMD_MA_DRIVE + motorNum, driveValue);
	}
}

//inversionMode( ... )
//
//    Configure a motor's direction inversion
//
//  uint8_t motorNum -- Motor number from 0 to 33
//  uint8_t polarity -- 0 or 1 for default or inverted
void SCMD::inversionMode( uint8_t motorNum, uint8_t polarity )
{
	uint8_t regTemp;
	//Select target register
	if( motorNum < 2 )
	{
		//master
		if( motorNum == 0 ) writeRegister(SCMD_MOTOR_A_INVERT, polarity & 0x01);
		if( motorNum == 1 ) writeRegister(SCMD_MOTOR_B_INVERT, polarity & 0x01);
	}
	else
	{
		if( motorNum < 10 )
		{
			//register: SCMD_INV_2_9
			regTemp = SCMD_INV_2_9;
			motorNum -= 2;

		}
		else if( motorNum < 18 )
		{
			//register: SCMD_INV_10_17
			regTemp = SCMD_INV_10_17;
			motorNum -= 10;
		}
		else if( motorNum < 26 )
		{
			//register: SCMD_INV_18_25
			regTemp = SCMD_INV_18_25;
			motorNum -= 18;
		}
		else if( motorNum < 34 )
		{
			//register: SCMD_INV_26_33
			regTemp = SCMD_INV_26_33;
			motorNum -= 26;
		}
		else
		{
			//out of range
			return;
		}
		//convert motorNum to one-hot mask
		uint8_t data = readRegister( regTemp ) & ~( 1 << motorNum );
		writeRegister( regTemp, data | ((polarity & 0x01) << motorNum) );
	}

}

//bridgingMode( ... )
//
//    Configure a driver's bridging state
//
//  uint8_t driverNum -- Number of driver.  Master is 0, slave 1 is 1, etc.  0 to 16
//  uint8_t bridged -- 0 or 1 for forward and backward
void SCMD::bridgingMode( uint8_t driverNum, uint8_t bridged )
{
	uint8_t regTemp;
	//Select target register
	if( driverNum < 1 )
	{
		//master
		writeRegister(SCMD_BRIDGE, bridged & 0x01);
	}
	else
	{
		if( driverNum < 9 )
		{
			//register: SCMD_BRIDGE_SLV_L
			regTemp = SCMD_BRIDGE_SLV_L;
			driverNum -= 1;

		}
		else if( driverNum < 17 )
		{
			//register: SCMD_BRIDGE_SLV_H
			regTemp = SCMD_BRIDGE_SLV_H;
			driverNum -= 9;
		}
		else
		{
			//out of range
			return;
		}
		//convert driverNum to one-hot mask
		uint8_t data = readRegister( regTemp ) & ~( 1 << driverNum );
		writeRegister( regTemp, data | ((bridged & 0x01) << driverNum) );
	}

}

//****************************************************************************//
//
//  Diagnostics
//
//****************************************************************************//

//getDiagnostics( ... )
//
//    Get diagnostic information from the master
//
//  SCMDDiagnostics &diagObjectReference -- Object to contain returned data
void SCMD::getDiagnostics( SCMDDiagnostics &diagObjectReference )
{
	diagObjectReference.U_I2C_RD_ERR = readRegister( SCMD_U_I2C_RD_ERR );
	diagObjectReference.U_I2C_WR_ERR = readRegister( SCMD_U_I2C_WR_ERR );
	diagObjectReference.U_BUF_DUMPED = readRegister( SCMD_U_BUF_DUMPED );
	diagObjectReference.E_I2C_RD_ERR = readRegister( SCMD_E_I2C_RD_ERR );
	diagObjectReference.E_I2C_WR_ERR = readRegister( SCMD_E_I2C_WR_ERR );
	diagObjectReference.LOOP_TIME = readRegister( SCMD_LOOP_TIME );
	diagObjectReference.SLV_POLL_CNT = readRegister( SCMD_SLV_POLL_CNT );
	//Count slaves
	uint8_t topAddr = readRegister( SCMD_SLV_TOP_ADDR );
	if( (topAddr >= START_SLAVE_ADDR) && (topAddr < (START_SLAVE_ADDR + 16)))
	{
		//in valid range
		diagObjectReference.numberOfSlaves = topAddr - START_SLAVE_ADDR + 1;
	}
	diagObjectReference.MST_E_ERR = readRegister( SCMD_MST_E_ERR );
	diagObjectReference.MST_E_STATUS = readRegister( SCMD_MST_E_STATUS );
	diagObjectReference.FSAFE_FAULTS = readRegister( SCMD_FSAFE_FAULTS );
	diagObjectReference.REG_OOR_CNT = readRegister( SCMD_REG_OOR_CNT );
	diagObjectReference.REG_RO_WRITE_CNT = readRegister( SCMD_REG_RO_WRITE_CNT );

}

//getRemoteDiagnostics( ... )
//
//    Get diagnostic information from a slave
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
//  SCMDDiagnostics &diagObjectReference -- Object to contain returned data
void SCMD::getRemoteDiagnostics( uint8_t address, SCMDDiagnostics &diagObjectReference )
{
	diagObjectReference.numberOfSlaves = 0;
	diagObjectReference.U_I2C_RD_ERR = 0;
	diagObjectReference.U_I2C_WR_ERR = 0;
	diagObjectReference.U_BUF_DUMPED = 0;
	diagObjectReference.E_I2C_RD_ERR = readRemoteRegister( address, SCMD_E_I2C_RD_ERR );
	diagObjectReference.E_I2C_WR_ERR = readRemoteRegister( address, SCMD_E_I2C_WR_ERR );
	diagObjectReference.LOOP_TIME = readRemoteRegister( address, SCMD_LOOP_TIME );
	diagObjectReference.SLV_POLL_CNT = 0;
	diagObjectReference.MST_E_ERR = 0;
	diagObjectReference.MST_E_STATUS = 0;
	diagObjectReference.FSAFE_FAULTS = readRemoteRegister( address, SCMD_FSAFE_FAULTS );
	diagObjectReference.REG_OOR_CNT = readRemoteRegister( address, SCMD_REG_OOR_CNT );
	diagObjectReference.REG_RO_WRITE_CNT = readRemoteRegister( address, SCMD_REG_RO_WRITE_CNT );

}

//resetDiagnosticCounts( ... )
//
//    Reset the master's diagnostic counters
//
void SCMD::resetDiagnosticCounts( void )
{
	writeRegister( SCMD_U_I2C_RD_ERR, 0 );
	writeRegister( SCMD_U_I2C_WR_ERR, 0 );
	writeRegister( SCMD_U_BUF_DUMPED, 0 );
	writeRegister( SCMD_E_I2C_RD_ERR, 0 );
	writeRegister( SCMD_E_I2C_WR_ERR, 0 );
	//Clear uport time
	writeRegister( SCMD_LOOP_TIME, 0 );
	writeRegister( SCMD_MST_E_ERR, 0 );
	writeRegister( SCMD_FSAFE_FAULTS, 0 );
	writeRegister( SCMD_REG_OOR_CNT, 0 );
	writeRegister( SCMD_REG_RO_WRITE_CNT, 0 );

}

//resetRemoteDiagnosticCounts( ... )
//
//    Reset a slave's diagnostic counters
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
void SCMD::resetRemoteDiagnosticCounts( uint8_t address )
{
	writeRemoteRegister( address, SCMD_U_I2C_RD_ERR, 0 );
	writeRemoteRegister( address, SCMD_U_I2C_WR_ERR, 0 );
	writeRemoteRegister( address, SCMD_U_BUF_DUMPED, 0 );
	writeRemoteRegister( address, SCMD_E_I2C_RD_ERR, 0 );
	writeRemoteRegister( address, SCMD_E_I2C_WR_ERR, 0 );
	//Clear uport time
	writeRemoteRegister( address, SCMD_LOOP_TIME, 0 );
	writeRemoteRegister( address, SCMD_ID, 0 );
	writeRemoteRegister( address, SCMD_FSAFE_FAULTS, 0 );
	writeRemoteRegister( address, SCMD_REG_OOR_CNT, 0 );
	writeRemoteRegister( address, SCMD_REG_RO_WRITE_CNT, 0 );
}

//****************************************************************************//
//
//  Register Access Functions
//
//****************************************************************************//

//readRegister( ... )
//
//    Read data from the master
//
//  uint8_t offset -- Address of data to read.  Can be 0x00 to 0x7F
uint8_t SCMD::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;

	// read
	result = wiringPiI2CReadReg8(m_fd, offset);

	// TODO: May not receive all data

#ifdef VERBOSE_SERIAL
	std::cout << "~R";
	std::cout << offset;
	std::cout << ":";
	std::cout << result << '\n';
#endif
return result;
}

//writeRegister( ... )
//
//    Write data to the master
//
//  uint8_t offset -- Address of data to write.  Can be 0x00 to 0x7F
//  uint8_t dataToWrite -- Data to write.
void SCMD::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	int result = wiringPiI2CWriteReg8(m_fd, offset, dataToWrite);

#ifdef VERBOSE_SERIAL
	std::cout << "~W";
	std::cout << offset;
	std::cout << ":";
	std::cout << dataToWrite << '\n';
#endif
}

//readRegister( ... )
//
//    Read data from a slave.  Note that this waits 5ms for slave data to be aquired
//  before making the final read.
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
//  uint8_t offset -- Address of data to read.  Can be 0x00 to 0x7F
uint8_t SCMD::readRemoteRegister(uint8_t address, uint8_t offset)
{
	//while(busy());
	writeRegister(SCMD_REM_ADDR, address);
	writeRegister(SCMD_REM_OFFSET, offset);
	writeRegister(SCMD_REM_READ, 1);
	while(busy());
	uint8_t result = readRegister(SCMD_REM_DATA_RD);
#ifdef VERBOSE_SERIAL
	std::cout << "~R";
	std::cout << address;
	std::cout << ",";
	std::cout << offset;
	std::cout << ":";
	std::cout << result << '\n';
#endif
	return result;

}

//writeRegister( ... )
//
//    Write data from a slave
//
//  uint8_t address -- Address of slave to read.  Can be 0x50 to 0x5F for slave 1 to 16.
//  uint8_t offset -- Address of data to write.  Can be 0x00 to 0x7F
//  uint8_t dataToWrite -- Data to write.
void SCMD::writeRemoteRegister(uint8_t address, uint8_t offset, uint8_t dataToWrite)
{
	while(busy());
	writeRegister(SCMD_REM_ADDR, address);
	writeRegister(SCMD_REM_OFFSET, offset);
	writeRegister(SCMD_REM_DATA_WR, dataToWrite);
	writeRegister(SCMD_REM_WRITE, 1);
	while(busy());
#ifdef VERBOSE_SERIAL
	std::cout << "~W";
	std::cout << offset;
	std::cout << ":";
	std::cout << dataToWrite << '\n';
#endif
}
