#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "heartRate.h"

// BLE Configuration
#define SERVICE_UUID "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
#define CHARACTERISTIC_UUID "a16cba2a-8165-4039-96c6-06e922eb6551"

// I2C Configuration
const uint8_t I2C_SDA_PIN = 21;
const uint8_t I2C_SCL_PIN = 22;
const uint32_t I2C_CLOCK_SPEED = 100000;

// MPU6050 Registers
const uint8_t MPU_ADDR = 0x68;
const uint8_t MPU_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU_INT_PIN_CFG = 0x37;
const uint8_t MPU_INT_ENABLE = 0x38;
const uint8_t MPU_INT_STATUS = 0x3A;
const uint8_t MPU_MOT_THR = 0x1F;
const uint8_t MPU_MOT_DUR = 0x20;

// MPU6050 Configuration Values
const uint8_t MPU_INT_PIN = 25;
const uint8_t MPU_MOTION_THRESHOLD = 20;    // 1-255, lower = more sensitive
const uint8_t MPU_MOTION_DURATION = 1;      // Duration in ms before triggering
const uint8_t MPU_INT_CFG_VALUE = 0x10;     // Active high, push-pull, clear on read
const uint8_t MPU_INT_ENABLE_MOTION = 0x40; // Enable motion detection interrupt
const float MPU_ACCEL_SCALE = 16384.0f;     // LSB/g for +/- 2g range

// MAX30102 Configuration
const uint8_t HR_LED_AMPLITUDE = 0x1F;
const int32_t HR_FINGER_THRESHOLD = 50000;     // IR value below this = no finger

// BPM Filtering Thresholds
const float BPM_MIN_VALID = 30.0f;
const float BPM_MAX_VALID = 200.0f;
const float BPM_MAX_JUMP = 30.0f;

// BLE Global Variables
BLEServer *pServer = nullptr;
BLEService *pService = nullptr;
BLECharacteristic *pCharacteristic = nullptr;

// Heart Rate Monitor
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
int32_t rates[RATE_SIZE];
int32_t rateSpot = 0;
unsigned long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// Heart Rate IR filtering
const int HR_MEDIAN_SIZE = 7;
const int HR_AVG_SIZE = 4;
int32_t hrMedianBuffer[HR_MEDIAN_SIZE];
int hrMedianIndex = 0;
int32_t hrAvgBuffer[HR_AVG_SIZE];
int hrAvgIndex = 0;

// Heart Rate BPM stabilizer
const int BPM_AVG_SIZE = 5;
float bpmBuffer[BPM_AVG_SIZE];
int bpmIndex = 0;
bool bpmStabilized = false;



// Motion filtering buffers
const int MOTION_MEDIAN_SIZE = 7;
const int MOTION_AVG_SIZE = 4;
float motionMedianBuffer[MOTION_MEDIAN_SIZE];
int motionMedianIndex = 0;
float motionAvgBuffer[MOTION_AVG_SIZE];
int motionAvgIndex = 0;

// Motion epoch aggregation
unsigned long lastMotionEpoch = 0;
float motionSum = 0;
int motionSampleCount = 0;
const unsigned long MOTION_EPOCH_MS = 60000; // 1 minute
float currentMotionScore = 0;

// Timing variables
unsigned long lastBLEUpdate = 0;
unsigned long lastMotionRead = 0;
const unsigned long BLE_UPDATE_INTERVAL = 4000; // 4 seconds
const unsigned long MOTION_READ_INTERVAL = 200; // 200ms
volatile bool motionDetected = false;

// Heart Rate IR Filter Function
long filterIR(long newValue) 
{
  // Median filter
  hrMedianBuffer[hrMedianIndex] = newValue;
  hrMedianIndex = (hrMedianIndex + 1) % HR_MEDIAN_SIZE;
  
  long sorted[HR_MEDIAN_SIZE];
  for (int i = 0; i < HR_MEDIAN_SIZE; i++) sorted[i] = hrMedianBuffer[i];
  
  for (int i = 0; i < HR_MEDIAN_SIZE - 1; i++) 
  {
    for (int j = i + 1; j < HR_MEDIAN_SIZE; j++) 
    {
      if (sorted[j] < sorted[i]) {
        long tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }
  long medianVal = sorted[HR_MEDIAN_SIZE / 2];
  
  // Moving Average
  hrAvgBuffer[hrAvgIndex] = medianVal;
  hrAvgIndex = (hrAvgIndex + 1) % HR_AVG_SIZE;
  
  long sum = 0;
  for (int i = 0; i < HR_AVG_SIZE; i++) sum += hrAvgBuffer[i];
  return sum / HR_AVG_SIZE;
}

// BPM Filter Function
float filterBPM(float bpm) 
{
  static float avg = 0;
  
  // Reject impossible values
  if (bpm < BPM_MIN_VALID || bpm > BPM_MAX_VALID) return avg;
  
  // Reject sudden jumps
  if (bpmStabilized && abs(bpm - avg) > BPM_MAX_JUMP) return avg;
  
  // Add to buffer
  bpmBuffer[bpmIndex] = bpm;
  bpmIndex = (bpmIndex + 1) % BPM_AVG_SIZE;
  
  float sum = 0;
  for (int i = 0; i < BPM_AVG_SIZE; i++) sum += bpmBuffer[i];
  avg = sum / BPM_AVG_SIZE;
  
  if (!bpmStabilized && bpmIndex == 0) bpmStabilized = true;
  return avg;
}

// Motion Filter Function
float filterMotion(float newValue) 
{
  // Median filter
  motionMedianBuffer[motionMedianIndex] = newValue;
  motionMedianIndex = (motionMedianIndex + 1) % MOTION_MEDIAN_SIZE;
  
  float sorted[MOTION_MEDIAN_SIZE];
  for (int i = 0; i < MOTION_MEDIAN_SIZE; i++) sorted[i] = motionMedianBuffer[i];
  
  // Simple bubble sort for median calculation
  for (int i = 0; i < MOTION_MEDIAN_SIZE - 1; i++) 
  {
    for (int j = i + 1; j < MOTION_MEDIAN_SIZE; j++) 
    {
      if (sorted[j] < sorted[i]) 
      {
        float tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }
  float medianVal = sorted[MOTION_MEDIAN_SIZE / 2];
  
  // Moving Average filter
  motionAvgBuffer[motionAvgIndex] = medianVal;
  motionAvgIndex = (motionAvgIndex + 1) % MOTION_AVG_SIZE;
  
  float sum = 0;
  for (int i = 0; i < MOTION_AVG_SIZE; i++) sum += motionAvgBuffer[i];
  return sum / MOTION_AVG_SIZE;
}


void IRAM_ATTR motionISR()
{
  motionDetected = true;
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("Initializing integrated BLE health monitor...");
  
  // Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_SPEED);
  
  // Initialize Heart Rate Sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) 
  {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  Serial.println("Heart rate sensor initialized.");
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(HR_LED_AMPLITUDE);
  particleSensor.setPulseAmplitudeGreen(0);
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0);    // wake up the MPU-6050
  Wire.endTransmission(true);
  
  // Configure MPU6050 motion detection
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_MOT_THR);
  Wire.write(MPU_MOTION_THRESHOLD);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_MOT_DUR);
  Wire.write(MPU_MOTION_DURATION);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_INT_PIN_CFG);
  Wire.write(MPU_INT_CFG_VALUE);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_INT_ENABLE);
  Wire.write(MPU_INT_ENABLE_MOTION);
  Wire.endTransmission(true);
  
  // Attach hardware interrupt
  pinMode(MPU_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), motionISR, RISING);
  
  Serial.println("MPU6050 initialized with motion interrupt on GPIO25.");
  
  // Initialize BLE
  BLEDevice::init("HealthMonitor_ESP32");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  
  // Create characteristic with Notify + Read + Write
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  BLE2902 *cccd = new BLE2902();
  cccd->setNotifications(true);
  pCharacteristic->addDescriptor(cccd);
  pCharacteristic->setValue("Health Monitor Ready");
  
  // Start BLE service and advertising
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE advertising started. Place finger on heart rate sensor.");
  Serial.println("System ready!");
  
  delay(100);
}

void loop() 
{
  unsigned long currentTime = millis();
  
  // Handle motion interrupt
  if (motionDetected)
  {
    motionDetected = false;
    
    // Clear interrupt by reading INT_STATUS register
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_INT_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 1, true);
    Wire.read();
    
    Serial.println("Motion interrupt triggered!");
  }
  
  // Read Heart Rate (continuous)
  long irValue = filterIR(particleSensor.getIR());
  if (checkForBeat(irValue) == true) 
  {
    long delta = currentTime - lastBeat;
    lastBeat = currentTime;
    
    float bpm = 60 / (delta / 1000.0);
    beatsPerMinute = filterBPM(bpm);
    
    if (beatsPerMinute > 0) 
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  
  // Read Motion Data (10Hz)
  if (currentTime - lastMotionRead >= MOTION_READ_INTERVAL) 
  {
    lastMotionRead = currentTime;
    
    // Read accelerometer data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    
    if (Wire.available() >= 6) 
    {
      int16_t ax = Wire.read() << 8 | Wire.read();
      int16_t ay = Wire.read() << 8 | Wire.read();
      int16_t az = Wire.read() << 8 | Wire.read();
      
      // Convert to g units
      float ax_g = ax / MPU_ACCEL_SCALE;
      float ay_g = ay / MPU_ACCEL_SCALE;
      float az_g = az / MPU_ACCEL_SCALE;
      
      // Calculate motion
      static float prev_ax = 0, prev_ay = 0, prev_az = 0;
      static bool firstMotionReading = true;
      float motion = 0;
      
      if (!firstMotionReading) 
      {
        motion = fabs(ax_g - prev_ax) + fabs(ay_g - prev_ay) + fabs(az_g - prev_az);
      } else {
        firstMotionReading = false;
      }
      
      prev_ax = ax_g;
      prev_ay = ay_g;
      prev_az = az_g;
      
      // Apply filtering
      float filteredMotion = filterMotion(motion);
      
      // Accumulate for epoch
      motionSum += filteredMotion;
      motionSampleCount++;
      
      // Check if epoch completed
      if (currentTime - lastMotionEpoch >= MOTION_EPOCH_MS) 
      {
        currentMotionScore = (motionSampleCount > 0) ? (motionSum / motionSampleCount) : 0;
        
        // Reset for next epoch
        motionSum = 0;
        motionSampleCount = 0;
        lastMotionEpoch = currentTime;
        
        Serial.print("Motion Epoch Complete - Score: ");
        Serial.println(currentMotionScore, 6);
      }
    }
  }
  
  // Update BLE
  if (currentTime - lastBLEUpdate >= BLE_UPDATE_INTERVAL) 
  {
    lastBLEUpdate = currentTime;
    
    if (pCharacteristic) 
    {
      // Create message with all sensor data
      String msg = "IR=" + String(irValue) + ", BPM=" + String(beatsPerMinute, 1) + ", Avg BPM=" + String(beatAvg) + ", Motion=" + String(currentMotionScore, 4);
      
      // Add finger detection
      if (irValue < HR_FINGER_THRESHOLD) 
      {
        msg += ", Status=No finger detected";
      } else {
        msg += ", Status=Active";
      }
      
      // Update BLE characteristic and notify
      pCharacteristic->setValue(msg.c_str());
      pCharacteristic->notify();
      
      // Also output to Serial
      Serial.println("BLE Update: " + msg);
    }
  }
}
