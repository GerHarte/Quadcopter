#ifndef UIMU_HMC5883L_h
#define UIMU_HMC5883L_h

#include "Arduino.h"
#include "I2CDevice.h"
#include "imumaths.h"


class HMC5883L : public I2CDevice
{
public:
    HMC5883L()
    {
        set_address(0x1E);
    }

    void init();
    imu::Vector<3> read_raw();
    imu::Vector<3> read_mag();

	int16_t cal[6];

private:
	
};


#endif
