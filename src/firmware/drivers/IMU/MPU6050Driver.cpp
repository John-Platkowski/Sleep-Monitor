#include "MPU6050Driver.h"
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
bool MPU6050Driver::init()
{
    Wire.begin();
    Wire.setClock(100000);
    
    // Verify device is present by reading WHO_AM_I register (should return 0x68)
    uint8_t whoAmI = readRegister(0x75);
    if (whoAmI != 0x68) 
    {
        Serial.print("ERROR: MPU6050 WHO_AM_I mismatch. Expected 0x68, got 0x");
        Serial.println(whoAmI, HEX);
        return false;
    }
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    uint8_t error = Wire.endTransmission(true);
    if (error != 0)
    {
        Serial.print("ERROR: MPU6050 I2C write failed with error: ");
        Serial.println(error);
        return false;
    }
    
    Serial.println("MPU6050 initialized successfully");
    return true;
}

void MPU6050Driver::configureMotionInterrupt(uint8_t threshold, uint8_t duration)
{
    // Reset device to known state
    writeRegister(0x6B, 0x80); // PWR_MGMT_1 Device reset
    delay(100); // Wait for reset to complete
    writeRegister(0x6B, 0x00); // Wake up from reset
    
    // Configure sample rate 1kHz / (1 + 4) = 200Hz
    writeRegister(0x19, 0x04); // SMPLRT_DIV
    
    // Configure DLPF for motion detection Accel BW ~20Hz
    writeRegister(0x1A, 0x04); // CONFIG DLPF_CFG = 4
    
    // Configure accelerometer, +/- 2g range
    writeRegister(0x1C, 0x00); // ACCEL_CONFIG AFS_SEL = 0
    
    // Enable all accelerometer axes
    writeRegister(0x6C, 0x00); // PWR_MGMT_2 enable all axes
    
    // Set motion detection threshold threshold * 2mg per LSB
    writeRegister(0x1F, threshold); // MOT_THR
    
    // Set motion detection duration
    writeRegister(0x20, duration); // MOT_DUR
    
    // Configure motion detection decrement rate
    writeRegister(0x69, 0x15); // MOT_DETECT_CTRL
    
    // Enable motion detection interrupt
    writeRegister(0x38, 0x40); // INT_ENABLE MOT_EN = 1
    
    // Configure interrupt pin, Active LOW, push-pull, latched, clear on INT_STATUS read
    writeRegister(0x37, 0x20); // INT_PIN_CFG LATCH_INT_EN=1, INT_RD_CLEAR=0
}

void MPU6050Driver::clearInterrupt()
{
    // Reading INT_STATUS clears the interrupt when INT_RD_CLEAR is set
    readRegister(0x3A);
}

void MPU6050Driver::writeRegister(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

uint8_t MPU6050Driver::readRegister(uint8_t reg)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
    return Wire.read();
}

MPU6050Driver::Data MPU6050Driver::read()
{
    Data data = {0, 0, 0, 0, 0, 0};
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting register ACCEL_XOUT_H
    uint8_t error = Wire.endTransmission(false);
    if (error != 0)
    {
        Serial.print("ERROR: MPU6050 read setup failed: ");
        Serial.println(error);
        return data;
    }
    
    uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
    if (bytesReceived != 14) 
    {
        Serial.print("ERROR: MPU6050 expected 14 bytes, got ");
        Serial.println(bytesReceived);
        return data;
    }
    
    uint8_t buffer[14];
    for (int i = 0; i < 14; i++) 
    {
        buffer[i] = Wire.read();
    }
    
    data.ax = (int16_t)(buffer[0] << 8 | buffer[1]);   // acceleration x
    data.ay = (int16_t)(buffer[2] << 8 | buffer[3]);   // acceleration y
    data.az = (int16_t)(buffer[4] << 8 | buffer[5]);   // acceleration z
    // buffer[6], buffer[7] = temperature, discarded
    data.gx = (int16_t)(buffer[8] << 8 | buffer[9]);   // gyro x
    data.gy = (int16_t)(buffer[10] << 8 | buffer[11]); // gyro y
    data.gz = (int16_t)(buffer[12] << 8 | buffer[13]); // gyro z
    return data;
}

float MPU6050Driver::getAccelerationMagnitude(Data& data)
{
    float total_force = sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    return abs(total_force - 1.0f);
}

void MPU6050Driver::sleep()
{
    writeRegister(0x6B, 0b01000000); // PWR_MGMT_1 register, Sleep mode
}

void MPU6050Driver::wake()
{
    writeRegister(0x6B, 0b00000000); // PWR_MGMT_1 register, Wake mode
}