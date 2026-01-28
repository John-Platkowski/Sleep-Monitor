#include "MAX30102Driver.h"

bool MAX30102Driver::init()
{
    if (!sensor.begin(Wire, I2C_SPEED_FAST)) 
    {
        return false;
    }
    
    sensor.setup();
    sensor.setPulseAmplitudeRed(0x1F);
    sensor.setPulseAmplitudeGreen(0);
    
    return true;
}

void MAX30102Driver::sleep()
{
    sensor.shutDown();
}

void MAX30102Driver::wake()
{
    sensor.wakeUp();
}

uint32_t MAX30102Driver::readIR()
{
    return sensor.getIR();
}