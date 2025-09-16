#include <Wire.h>

const int MPU_addr = 0x68;

// Filtering buffers
const int MEDIAN_SIZE = 7;
const int AVG_SIZE = 4;
float medianBuffer[MEDIAN_SIZE];
int medianIndex = 0;
float avgBuffer[AVG_SIZE];
int avgIndex = 0;

// Epoch aggregation
unsigned long lastEpoch = 0;
float motionSum = 0;
int sampleCount = 0;
const unsigned long EPOCH_MS = 60000;

float filterMotion(float newValue)
{
  // Median filter
  medianBuffer[medianIndex] = newValue;
  medianIndex = (medianIndex + 1) % MEDIAN_SIZE;
  
  float sorted[MEDIAN_SIZE];
  for (int i = 0; i < MEDIAN_SIZE; i++) sorted[i] = medianBuffer[i];
  
  // Simple bubble sort for median calculation
  for (int i = 0; i < MEDIAN_SIZE - 1; i++)
  {
    for (int j = i + 1; j < MEDIAN_SIZE; j++)
    {
      if (sorted[j] < sorted[i])
      {
        float tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }
  float medianVal = sorted[MEDIAN_SIZE / 2];
  
  // Moving Average filter
  avgBuffer[avgIndex] = medianVal;
  avgIndex = (avgIndex + 1) % AVG_SIZE;
  
  float sum = 0;
  for (int i = 0; i < AVG_SIZE; i++) sum += avgBuffer[i];
  return sum / AVG_SIZE;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  
  Wire.begin(21, 22);
  Wire.setClock(100000);
  
  // Wake up the MPU-6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 initialized - starting motion detection");
  delay(100);
}

void loop()
{
  // Read accelerometer data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // request 6 registers for accelerometer only
  
  if (Wire.available() >= 6) 
  {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    
    // Convert raw accel to g units (same scale as your original code)
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    
    // Calculate motion as total change since last sample
    static float prev_ax = 0, prev_ay = 0, prev_az = 0;
    static bool firstReading = true;
    
    float motion = 0;
    if (!firstReading) 
    {
      motion = fabs(ax_g - prev_ax) + fabs(ay_g - prev_ay) + fabs(az_g - prev_az);
    } else {
      firstReading = false;
    }
    
    prev_ax = ax_g;
    prev_ay = ay_g;
    prev_az = az_g;
    
    // Apply filtering
    float filteredMotion = filterMotion(motion);
    
    // Accumulate motion data
    motionSum += filteredMotion;
    sampleCount++;
    
    // Output results once per epoch (every 60 seconds)
    unsigned long now = millis();
    if (now - lastEpoch >= EPOCH_MS)
    {
      float motionPerEpoch = (sampleCount > 0) ? (motionSum / sampleCount) : 0;
      
      // Output: timestamp (seconds), average motion score for this epoch
      Serial.print(now / 1000);
      Serial.print(",");
      Serial.println(motionPerEpoch, 6);
      
      // Reset for next epoch
      motionSum = 0;
      sampleCount = 0;
      lastEpoch = now;
    }
  }
  
  delay(100); // 10 Hz sampling rate
}