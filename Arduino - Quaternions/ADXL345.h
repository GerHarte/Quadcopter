#ifndef UIMU_ADXL345
#define UIMU_ADXL345

#include "/Users/gerardharte/Documents/Arduino/quadcopter/I2CDevice.h"
#include "/Users/gerardharte/Documents/Arduino/quadcopter/imumaths.h"

#define ADXL345_ADDR_ALT_HIGH 0x1D
#define ADXL345_ADDR_ALT_LOW  0x53

#define ADXL345_POWER_CTL 0x2d
#define ADXL345_DATAX0 0x32

class ADXL345 : public I2CDevice
{
public:
    ADXL345(bool hl) : I2CDevice()
    {
        if(hl)
            set_address(ADXL345_ADDR_ALT_HIGH);
        else
            set_address(ADXL345_ADDR_ALT_LOW);
    }

    void init();

    imu::Vector<3> read_acc();
    imu::Vector<3> axis_gains;

private:


};


#endif

