#include "MPU6050Driver.h"
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
// Weirdly enough, the MPU6050 we recieved does not conform to the 6050 datasheet; It matches the 6500 datasheet instead.
#include <math.h>

// Helper to keep track of which chip we actually have
static uint8_t _deviceId = 0x68; 

bool MPU6050Driver::init()
{
    Wire.begin(21, 22);
    Wire.setClock(400000);
    delay(100);
    
    // Check ID
    _deviceId = readRegister(0x75);
    
    // Accept 0x68 (MPU6050) or 0x70 (MPU6500/9250)
    if (_deviceId != 0x68 && _deviceId != 0x70) 
    {
        Serial.print("ERROR: Unknown MPU ID: 0x");
        Serial.println(_deviceId, HEX);
        return false;
    }
    
    // Wake up
    writeRegister(0x6B, 0x00); 
    // Clear any stuck interrupts from before the reboot
    readRegister(0x3A); // Read INT_STATUS
    Serial.println("MPU initialized successfully");
    return true;
}

void MPU6050Driver::configureMotionInterrupt(uint8_t threshold, uint8_t duration)
{
    writeRegister(0x1C, 0x00); // ACCEL_CONFIG: +/- 2g
    writeRegister(0x1B, 0x00); // GYRO_CONFIG: +/- 250dps
    writeRegister(0x19, 0x09); // Sample Rate 100Hz
    writeRegister(0x1A, 0x03); // DLPF ~40Hz bandwidth
    
    // Interrupt Pin Configuration (Active LOW, Push-Pull, Active LOW)
    // 0xA0 = 1010_0000
    writeRegister(0x37, 0xA0); 
    Serial.println(String(_deviceId));
        // Wake-on-Motion Threshold (1 LSB = 4mg)
        writeRegister(0x1F, threshold); 
        
        // ACCEL_INTEL_CTRL
        // 0xC0 = 1100_0000, Enable + Compare Mode
        writeRegister(0x69, 0xC0); 

        // INT_ENABLE 
        // Bit 6 is Wake on Motion
        writeRegister(0x38, 0x40); 
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

int16_t MPU6050Driver::readTempRaw()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x41);  // TEMP_OUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return (int16_t)(hi << 8 | lo);
}

void MPU6050Driver::addTemperatureSample(uint32_t nowMs)
{
    if (tempEpochCount > 0 && (nowMs - tempLastSampleMs) < TEMP_SAMPLE_INTERVAL_MS)
    {
        return;
    }
    if (tempEpochCount == 0)
    {
        tempEpochStartMs = nowMs;
    }
    int16_t raw = readTempRaw();
    float c = (raw / TEMP_SCALE) + TEMP_OFFSET_C;
    tempEpochSum += c;
    tempEpochCount++;
    tempLastSampleMs = nowMs;
}

float MPU6050Driver::getEpochTemperatureC(uint32_t nowMs, uint32_t epochDurationMs)
{
    if (tempEpochCount == 0)
    {
        return tempHasReported ? tempLastReportedC : 0.0f;
    }

    uint32_t elapsed = nowMs - tempEpochStartMs;
    if (elapsed < epochDurationMs)
    {
        return tempHasReported ? tempLastReportedC : (tempEpochSum / tempEpochCount);
    }
    float mean = tempEpochSum / (float)tempEpochCount;
    if (tempHasReported)
    {
        float delta = mean - tempLastReportedC;
        if (delta > TEMP_MAX_DELTA_C)
        {
            mean = tempLastReportedC + TEMP_MAX_DELTA_C;
        } else if (delta < -TEMP_MAX_DELTA_C) {
            mean = tempLastReportedC - TEMP_MAX_DELTA_C;
        }
    } else {
        tempHasReported = true;
    }

    tempLastReportedC = mean;
    tempEpochSum = 0.0f;
    tempEpochCount = 0;
    tempEpochStartMs = nowMs;
    return mean;
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
    // buffer[6], buffer[7] = TEMP_OUT
    data.gx = (int16_t)(buffer[8] << 8 | buffer[9]);   // gyro x
    data.gy = (int16_t)(buffer[10] << 8 | buffer[11]); // gyro y
    data.gz = (int16_t)(buffer[12] << 8 | buffer[13]); // gyro z
    return data;
}

float MPU6050Driver::getAccelerationMagnitude(Data& data)
{
    const float SCALE = 16384.0f; 
    
    float ax_g = data.ax / SCALE;
    float ay_g = data.ay / SCALE;
    float az_g = data.az / SCALE;

    float total_force = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
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