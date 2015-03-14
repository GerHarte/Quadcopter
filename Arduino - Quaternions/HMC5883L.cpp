#include "HMC5883L.h"

void HMC5883L::init()
{
    write_reg(0x02, 0x00); //continuous mode

	cal[0] = 0; cal[1] = 18;
	cal[2] = 0; cal[3] = 386;
	cal[4] = 0; cal[5] = -436;
}


imu::Vector<3> HMC5883L::read_raw()
{
    uint8_t buffer[6];

    read_bytes(0x03, buffer, 6);
	imu::Vector<3> raw;

    raw.x() = ((int16_t)buffer[0] << 8) | buffer[1];
    raw.z() = ((int16_t)buffer[2] << 8) | buffer[3];
    raw.y() = ((int16_t)buffer[4] << 8) | buffer[5];
    return raw;
}

imu::Vector<3> HMC5883L::read_mag()
{
	imu::Vector<3> mag = read_raw();
	mag.x() -= (cal[0] + cal[1]) / 2;
        mag.z() -= (cal[2] + cal[3]) / 2;
        mag.y() -= (cal[4] + cal[5]) / 2;

	return mag;
}


