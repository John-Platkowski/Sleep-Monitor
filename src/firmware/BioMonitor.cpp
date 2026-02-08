#include "BioMonitor.h"
#include "Matrix.h"
// https://www.freertos.org/Documentation/00-Overview

#define MPU_INT_PIN 25

// PPG sampling period - single source of truth for timing
// 40ms = 25Hz sampling rate (MAX30102 default is 50 samples/sec)
static constexpr uint32_t SAMPLE_PERIOD_MS = 20;
static constexpr float dt = SAMPLE_PERIOD_MS / 1000.0f;  // Convert to seconds for Kalman

BioMonitor* globalMonitor = nullptr;

// State transition matrix F: predicts next state from current
// [HR_new]     [1  dt  ] [HR]        HR_new = HR + dt * velocity
// [vel_new]  = [0  decay] [vel]      vel_new = decay * vel (mean-reverting velocity)
// Velocity decay prevents accumulation and pulls toward rest state
static constexpr float VELOCITY_DECAY = 0.96f; // Approx 0.9-0.98
const Matrix<float, 2, 2> BioMonitor::F = {
    1.0f, dt,
    0.0f, VELOCITY_DECAY
};

// Maximum allowable HR velocity (BPM per second)
// Physiologically, HR rarely changes faster than ~5 BPM/s even during exercise
static constexpr float MAX_HR_VELOCITY = 5.0f;

// Measurement matrix H: maps state to measurement space
// We only measure HR directly: z = [1 0] * [HR, vel]^T
const Matrix<float, 1, 2> BioMonitor::H = {
    1.0f, 0.0f
};

// Base process noise covariance Q: uncertainty in the motion model
// Higher values = less trust in prediction, more responsive to measurements
const Matrix<float, 2, 2> BioMonitor::Q = {
    0.014f, 0.0f,
    0.0f,  0.014f
};

// Adaptive Q parameters
static constexpr float Q_VEL_BASE = 0.01f;
static constexpr float Q_VEL_HIGH = 0.1f;
static constexpr float INNOVATION_THRESH = 6.0f;
static constexpr float ADAPT_RATE = 0.1f;


static constexpr float MOTION_Q_GAIN = 4.2f;

// Measurement noise covariance R: uncertainty in HR sensor readings
// Higher values = less trust in measurements, smoother output
static constexpr float R_BASE = 3.8f;
const Matrix<float, 1, 1> BioMonitor::R = { R_BASE };

// Identity matrix
const Matrix<float, 2, 2> BioMonitor::I = Matrix<float, 2, 2>::identity();

BioMonitor::BioMonitor() : motionDetected(false), lastAccelMag(0.0f)
{
    globalMonitor = this;
    
    // Initialize state estimate: assume resting HR of 70, no velocity
    x(0, 0) = 70.0f;  // Initial HR estimate
    x(1, 0) = 0.0f;   // Initial velocity (no change)
    
    // Initialize covariance with high uncertainty; true state is not known
    P(0, 0) = 100.0f;  // HR variance
    P(0, 1) = 0.0f;
    P(1, 0) = 0.0f;
    P(1, 1) = 10.0f;   // Velocity variance
    
    // Initialize adaptive Q to base values
    Q_adaptive(0, 0) = Q(0, 0);
    Q_adaptive(0, 1) = 0.0f;
    Q_adaptive(1, 0) = 0.0f;
    Q_adaptive(1, 1) = Q_VEL_BASE;
}

void BioMonitor::begin()
{
    // Initialize sensors (IMU init handles Wire.begin())
    if (!imu.init()) 
    {
        Serial.println("ERROR: MPU6050 initialization failed");
    }
    if (!ppg.init()) 
    {
        Serial.println("ERROR: MAX30102 initialization failed");
    }

    // Configure IMU motion detection interrupt
    // threshold: ~40mg (n * 2mg/LSB), duration: 10ms
    imu.configureMotionInterrupt(10, 10);

    // Configure GPIO interrupt for MPU motion detection
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    imu.clearInterrupt();
    delay(10);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), isrTrampoline, FALLING);

    // Function, Name, Stack Depth, *pvParameters, Priority, Handle
    xTaskCreate(taskTrampoline, "BioTask", 4096, this, 1, &taskHandle);

    // Initialize BLE and start periodic notifications
    ble.init();
    ble.startPeriodicNotify(BLE_NOTIFY_PERIOD_MS, bleNotifyCallback, this);
}

void BioMonitor::taskTrampoline(void *pvParameters)
{
    // Cast void* back to BioMonitor*
    BioMonitor* instance = static_cast<BioMonitor*>(pvParameters);
    instance -> runLoop();
    vTaskDelete(NULL);
}

void BioMonitor::isrTrampoline()
{
    // Jump to instance method
    if (globalMonitor) globalMonitor -> handleISR();
}

void BioMonitor::handleISR()
{
    motionDetected = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BioMonitor::runLoop()
{
    const TickType_t samplePeriod = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Maintain the 50Hz heartbeat of the system
        vTaskDelayUntil(&lastWakeTime, samplePeriod);
        
        float accelMag = readAccelIfMotion();
        float bpm = ppg.processSample();
        imu.addTemperatureSample(millis());

        // This advances the state by dt, keeping sync with real time.
        predictKalman(accelMag);

        // This corrects the predicted state.
        if (bpm > 0.0f)
        {
            updateKalman(bpm);
        }
    } 
    // TODO: Power management should be handled by a separate task/state machine
    // that coordinates sensor sleep, BLE sleep, and ESP32 light sleep modes
}

float BioMonitor::readAccelIfMotion()
{
    // Only read IMU when motion interrupt has fired
    if (motionDetected)
    {
        motionDetected = false;
        
        // Read acceleration and compute magnitude
        MPU6050Driver::Data data = imu.read();
        lastAccelMag = imu.getAccelerationMagnitude(data);
        if (lastAccelMag > currentAccelMagUI)
        {
            currentAccelMagUI = lastAccelMag;
        }
        
        // Clear the hardware interrupt latch
        imu.clearInterrupt();
        
        return lastAccelMag;
    }
    
    // Decay accel magnitude toward zero when no new motion detected
    // Smooth transition back to rest state for Q scaling in predictKalman()
    lastAccelMag *= 0.95f;
    return lastAccelMag;
}

String BioMonitor::bleNotifyCallback(void* context)
{
    BioMonitor* monitor = static_cast<BioMonitor*>(context);
    float hr = monitor->getFilteredHR();
    float motion = monitor->getMotionScore();
    float tempC = monitor->getEpochTemperatureC();
    static char buffer[80];
    snprintf(buffer, sizeof(buffer), "HR=%.1f, Motion=%.2f, Temp=%.1f", hr, motion, tempC);
    return String(buffer);
}

void BioMonitor::predictKalman(float accelMag)
{
    // Predict state: x = F * x
    x = F * x;
    
    // Clamp velocity to physiologically reasonable range
    if (x(1, 0) > MAX_HR_VELOCITY) x(1, 0) = MAX_HR_VELOCITY;
    if (x(1, 0) < -MAX_HR_VELOCITY) x(1, 0) = -MAX_HR_VELOCITY;
    
    // Scale process noise by motion magnitude
    float motionFactor = 1.0f + accelMag * MOTION_Q_GAIN;
    Q_scratch(0, 0) = Q_adaptive(0, 0) * motionFactor;
    Q_scratch(0, 1) = 0.0f;
    Q_scratch(1, 0) = 0.0f;
    Q_scratch(1, 1) = Q_adaptive(1, 1) * motionFactor;
    
    // Predict covariance: P = F * P * F^T + Q_scratch
    P = F * P * F.transpose() + Q_scratch;
}

void BioMonitor::updateKalman(float measurement)
{
    // Innovation (measurement residual): y = z - H * x
    y(0, 0) = measurement - (H * x)(0, 0);
    
    // Adapt process noise based on innovation magnitude
    adaptProcessNoise(y(0, 0));
    
    // Innovation covariance: S = H * P * H^T + R
    S = H * P * H.transpose() + R;
    
    // Kalman gain: K = P * H^T * S^(-1)
    K = P * H.transpose() * S.inverse();
    
    // Update state: x = x + K * y
    x = x + K * y;
    
    // Clamp velocity after measurement update as well
    if (x(1, 0) > MAX_HR_VELOCITY) x(1, 0) = MAX_HR_VELOCITY;
    if (x(1, 0) < -MAX_HR_VELOCITY) x(1, 0) = -MAX_HR_VELOCITY;
    
    // Update covariance: P = (I - K * H) * P
    P = (I - K * H) * P;
}

// TODO: Replace innovation-based adaptation with sleep-stage-dependent Q values once sleep staging is implemented. 
// Early sleep stages (N1/N2) have higher HRV and would benefit from higher Q_VEL, while deep sleep (N3) and REM have more stable patterns.
void BioMonitor::adaptProcessNoise(float innovation)
{
    // Target Q velocity variance based on innovation magnitude
    float targetQVel = (std::abs(innovation) > INNOVATION_THRESH) ? Q_VEL_HIGH : Q_VEL_BASE;
    
    // Exponential smoothing to avoid abrupt Q changes
    // Q_new = Q_old + alpha * (target - Q_old)
    Q_adaptive(1, 1) += ADAPT_RATE * (targetQVel - Q_adaptive(1, 1));
}

float BioMonitor::getFilteredHR() const
{
    return x(0, 0);  // Return current HR true state
}

float BioMonitor::getMotionScore() const
{
    return lastAccelMag;
}

float BioMonitor::getEpochTemperatureC()
{
    return imu.getEpochTemperatureC(millis(), BLE_NOTIFY_PERIOD_MS);
}