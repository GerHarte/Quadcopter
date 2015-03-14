#include "/Users/gerardharte/Documents/Arduino/quadcopter/I2CDevice.h"

I2CDevice::I2CDevice()
{

}


void I2CDevice::set_address(uint8_t address)
{
    _addr = address;
}

void I2CDevice::write_reg(uint8_t reg, uint8_t value)
{
    I2C_BUS.beginTransmission(_addr);
    I2C_BUS.write(reg);
    I2C_BUS.write(value);
    I2C_BUS.endTransmission();
}

uint8_t I2CDevice::read_reg(uint8_t reg)
{
    uint8_t value;

    I2C_BUS.beginTransmission(_addr);
    I2C_BUS.write(reg);
    I2C_BUS.endTransmission();
    I2C_BUS.requestFrom(_addr, (uint8_t)1);
    value = I2C_BUS.read();
    I2C_BUS.endTransmission();

    return value;
}

uint8_t I2CDevice::read_bytes(uint8_t reg, uint8_t* buf, uint8_t num)
{
    I2C_BUS.beginTransmission(_addr);
    I2C_BUS.write(reg);
    I2C_BUS.endTransmission();
    I2C_BUS.requestFrom(_addr, (uint8_t)num);

    for(int i = 0; i < num; i++)
        buf[i] = I2C_BUS.read();
}


