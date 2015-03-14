#include "/Users/gerardharte/Documents/Arduino/quadcopter/ADXL345.h"

void ADXL345::init()
{
    write_reg(ADXL345_POWER_CTL, 8);

    axis_gains.x() = 0.00376390;
    axis_gains.y() = 0.00376009;
    axis_gains.z() = 0.00349265;
}

imu::Vector<3> ADXL345::read_acc()
{
    imu::Vector<3> ret;
    uint8_t buf[6];
    read_bytes(ADXL345_DATAX0, buf, 6);

    int16_t x, y, z;

    x = (((buf[1]) << 8) | buf[0]);
    y = (((buf[3]) << 8) | buf[2]);
    z = (((buf[5]) << 8) | buf[4]);

    ret.x() = x * axis_gains.x();
    ret.y() = y * axis_gains.y();
    ret.z() = z * axis_gains.z();
    
    /*
    Serial.print("x: ");
    Serial.print(x);
    Serial.print("\t y: ");
    Serial.print(y);
    Serial.print("\t z: ");
    Serial.println(z);
    */
    
    return ret;
}


