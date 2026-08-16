#include "MPU6050Driver.h"
#include <math.h>

bool MPU6050Driver::init()
{
    // SDA on 21, SCL on 22, at 400kHz fast mode. This owns bus setup for the whole system;
    // MAX30102Driver::init() reuses the bus opened here and must therefore run after it.
    Wire.begin(21, 22);
    Wire.setClock(400000);
    delay(100);

    uint8_t whoAmI = readRegister(0x75);
    if (whoAmI != MPU_WHOAMI_6500)
    {
        Serial.print("ERROR: expected MPU6500 (0x70), got: 0x");
        Serial.println(whoAmI, HEX);
        return false;
    }
    
    // PWR_MGMT_1: clear the sleep bit set at power-on, along with the cycle and temperature-disable
    // bits enterLowPowerMotion() sets. PWR_MGMT_2: all six axes out of standby, which the same call
    // puts the gyroscope into. Neither is necessarily at its power-on default.
    writeRegister(0x6B, 0x00);
    writeRegister(0x6C, 0x00);

    // Drain INT_STATUS before anyone attaches a handler; a latched interrupt survives a reboot.
    readRegister(0x3A);
    Serial.println("MPU initialized successfully");
    return true;
}

// The MPU6500 wake-on-motion block has no duration register, so motion is reported as soon as one
// sample clears the threshold.
void MPU6050Driver::configureMotionInterrupt(uint8_t threshold)
{
    writeRegister(0x1C, 0x00); // ACCEL_CONFIG: +/- 2g
    writeRegister(0x1B, 0x00); // GYRO_CONFIG: +/- 250dps
    writeRegister(0x19, 0x09); // Sample Rate 100Hz
    writeRegister(0x1A, 0x03); // DLPF ~40Hz bandwidth

    // ACCEL_CONFIG2: 460Hz bandwidth, the default. enterLowPowerMotion() narrows it to 184Hz and that
    // survives into the next boot.
    writeRegister(0x1D, 0x00);

    // Interrupt Pin Configuration (Active LOW, Push-Pull, Latch until read)
    // 0xA0 = 1010_0000
    writeRegister(0x37, 0xA0);

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
    // Repeated start rather than a stop, so no other master can take the bus between addressing the
    // register and reading it back.
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
    return Wire.read();
}

int16_t MPU6050Driver::readTempRaw()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x41);  // TEMP_OUT_H, with TEMP_OUT_L following at 0x42.
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    // Big-endian on the wire, and signed: the die reads below the reference temperature as negative.
    return (int16_t)(hi << 8 | lo);
}

void MPU6050Driver::addTemperatureSample(uint32_t nowMs)
{
    // getEpochTemperatureC() can reset the epoch from the BLE timer task between any two lines here,
    // so read the pacing state under the lock.
    portENTER_CRITICAL(&tempMux);
    bool due = (tempEpochCount == 0) || ((nowMs - tempLastSampleMs) >= TEMP_SAMPLE_INTERVAL_MS);
    portEXIT_CRITICAL(&tempMux);

    if (!due)
    {
        return;
    }

    // I2C takes milliseconds; never hold the spinlock across it.
    int16_t raw = readTempRaw();
    float c = (raw / TEMP_SCALE) + TEMP_OFFSET_C;

    portENTER_CRITICAL(&tempMux);
    if (tempEpochCount == 0)
    {
        tempEpochStartMs = nowMs;
    }
    tempEpochSum += c;
    tempEpochCount++;
    tempLastSampleMs = nowMs;
    portEXIT_CRITICAL(&tempMux);
}

float MPU6050Driver::getEpochTemperatureC(uint32_t nowMs, uint32_t epochDurationMs)
{
    // Runs on the BLE timer task while the sampling task is accumulating, so the read-average-reset
    // sequence has to be atomic as a whole. No I2C in here.
    portENTER_CRITICAL(&tempMux);

    if (tempEpochCount == 0)
    {
        float held = tempHasReported ? tempLastReportedC : 0.0f;
        portEXIT_CRITICAL(&tempMux);
        return held;
    }

    uint32_t elapsed = nowMs - tempEpochStartMs;
    if (elapsed < epochDurationMs)
    {
        float partial = tempHasReported ? tempLastReportedC : (tempEpochSum / tempEpochCount);
        portEXIT_CRITICAL(&tempMux);
        return partial;
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

    portEXIT_CRITICAL(&tempMux);
    return mean;
}

MPU6050Driver::Data MPU6050Driver::read()
{
    // Accelerometer, temperature, and gyroscope occupy 14 consecutive registers from 0x3B, so one
    // burst read gets all six axes from a single instant.
    Data data = {0, 0, 0, 0, 0, 0};

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H, first of the 14.
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
    
    // Each axis arrives as a big-endian signed pair. Bytes 6 and 7 are TEMP_OUT, which
    // addTemperatureSample() reads separately on its own slower schedule.
    data.ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    data.ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    data.az = (int16_t)(buffer[4] << 8 | buffer[5]);
    data.gx = (int16_t)(buffer[8] << 8 | buffer[9]);
    data.gy = (int16_t)(buffer[10] << 8 | buffer[11]);
    data.gz = (int16_t)(buffer[12] << 8 | buffer[13]);
    return data;
}

float MPU6050Driver::getAccelerationMagnitude(Data& data)
{
    // Counts per g at the +/-2g range set in configureMotionInterrupt().
    const float SCALE = 16384.0f;

    float ax_g = data.ax / SCALE;
    float ay_g = data.ay / SCALE;
    float az_g = data.az / SCALE;

    // The vector magnitude includes gravity, which reads 1g whatever direction the sensor faces.
    // Subtracting 1 makes the result orientation-independent: any stationary pose reads near zero, and
    // only real movement registers.
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

void MPU6050Driver::enterLowPowerMotion(uint8_t threshold, uint8_t odrCode)
{
    // ACCEL_CONFIG2: 184Hz bandwidth. Motion is detected by differencing consecutive samples, so the
    // threshold sees their noise as well as their signal.
    writeRegister(0x1D, 0x01);

    writeRegister(0x1F, threshold); // WOM_THR, in 4mg counts.
    writeRegister(0x1E, odrCode); // LP_ACCEL_ODR: how often the cycle counter takes a sample.

    writeRegister(0x69, 0xC0); // MOT_DETECT_CTRL: intelligence enabled, compare against previous.
    writeRegister(0x38, 0x40); // INT_ENABLE: bit 6, wake on motion.

    writeRegister(0x6C, 0x07); // PWR_MGMT_2: gyroscope to standby.

    // PWR_MGMT_1: cycle and temperature-disable set, sleep left clear. Cycle turns continuous sampling
    // into one burst per interval; clearing sleep keeps the accelerometer, and the interrupt, alive.
    writeRegister(0x6B, 0x28);
}