/*
    FireTail UAV System
    Copyright (C) 2013-2014  Samuel Cowen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "/Users/gerardharte/Documents/Arduino/quadcopter/L3G.h"
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define L3G4200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
#define L3G4200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3GD20_ADDRESS_SA0_LOW    (0xD4 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)

// Public Methods //////////////////////////////////////////////////////////////

bool L3G::init()
{
 
    //address = L3G4200D_ADDRESS_SA0_HIGH;
    autoDetectAddress();
    //Serial.println(address);

}

// Turns on the L3G's gyro and places it in normal mode.
void L3G::enableDefault(void)
{
    writeReg(L3G_CTRL_REG1, 0b00001111); // Normal power mode, all axes enabled
    writeReg(L3G_CTRL_REG4, 0b00100000); // 2000 dps full scale
}

// Writes a gyro register
void L3G::writeReg(byte reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Reads a gyro register
byte L3G::readReg(byte reg)
{
    byte value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G::read()
{
    measure();
    apply_offset();
    
    g = (g * 0.07 * PI / 180.0);


  
}

/*
imu::Vector<3> L3G::read_gyro()
{
    measure();
    apply_offset();
    g = g * (0.07 * 3.14159265 / 180.0);

    return g;
}
*/



// Private Methods //////////////////////////////////////////////////////////////

void L3G::measure()
{

    Wire.beginTransmission(address);
    // assert the MSB of the address to get the gyro
    // to do slave-transmit subaddress updating.
    Wire.write(L3G_OUT_X_L | (1 << 7));
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)6);
    while (Wire.available() < 6);
    uint8_t xlg = Wire.read();  
    uint8_t xhg = Wire.read();
    uint8_t ylg = Wire.read();
    uint8_t yhg = Wire.read();
    uint8_t zlg = Wire.read();
    uint8_t zhg = Wire.read();

    // combine high and low bytes
    int16_t gx = (int16_t)(xhg << 8 | xlg);
    int16_t gy = (int16_t)(yhg << 8 | ylg);
    int16_t gz = (int16_t)(zhg << 8 | zlg);
    

	g.x() = gx;
	g.y() = gy;
	g.z() = gz;



}

void L3G::apply_offset()
{
    g = g - gyro_offset;
}

void L3G::measureOffsets()
{
    const int sampleCount = 100;
    for(int i = 0; i < sampleCount-1; i++)
    {
        measure();
        gyro_offset = gyro_offset + g;
        delay(20);
    }

    gyro_offset = gyro_offset / sampleCount;
}


bool L3G::autoDetectAddress(void)
{
    // try each possible address and stop if reading WHO_AM_I returns the expected response
    address = L3G4200D_ADDRESS_SA0_LOW;
    if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
    address = L3G4200D_ADDRESS_SA0_HIGH;
    if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
    address = L3GD20_ADDRESS_SA0_LOW;
    if (readReg(L3G_WHO_AM_I) == 0xD4) return true;
    address = L3GD20_ADDRESS_SA0_HIGH;
    if (readReg(L3G_WHO_AM_I) == 0xD4) return true;

    return false;
}
