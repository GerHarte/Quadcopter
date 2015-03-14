#ifndef I2CDEVICE_H
#define I2CDEVICE_H

#include "Arduino.h"
#include <Wire.h>


#define I2C_BUS Wire


class I2CDevice
{
public:
    I2CDevice();

    void set_address(uint8_t address);

    void write_reg(uint8_t reg, uint8_t value);
    uint8_t read_reg(uint8_t reg);

    uint8_t read_bytes(uint8_t reg, uint8_t* buf, uint8_t num);

private:
    uint8_t _addr;
};


#endif

