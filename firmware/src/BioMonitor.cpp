#include "BioMonitor.h"
#include "Matrix.h"
// https://www.freertos.org/Documentation/00-Overview

#define MPU_INT_PIN 25

// PPG sampling period - single source of truth for timing
// 40ms = 25Hz sampling rate (MAX30102 default is 50 samples/sec)
static constexpr uint32_t SAMPLE_PERIOD_MS = 40;
static constexpr float dt = SAMPLE_PERIOD_MS / 1000.0f;  // Convert to seconds for Kalman

BioMonitor* globalMonitor = nullptr;

// State transition matrix F: predicts next state from current
// [HR_new]     [1  dt] [HR]        HR_new = HR + dt * velocity
// [vel_new]  = [0   1] [vel]       vel_new = vel (constant velocity model)
const Matrix<float, 2, 2> BioMonitor::F = {
    1.0f, dt,
    0.0f, 1.0f
};

// Measurement matrix H: maps state to measurement space
// We only measure HR directly: z = [1 0] * [HR, vel]^T
const Matrix<float, 1, 2> BioMonitor::H = {
    1.0f, 0.0f
};

// Process noise covariance Q: uncertainty in our motion model
// Higher values = less trust in prediction, more responsive to measurements
const Matrix<float, 2, 2> BioMonitor::Q = {
    0.01f, 0.0f,
    0.0f,  0.01f
};

// Measurement noise covariance R: uncertainty in HR sensor readings
// Higher values = less trust in measurements, smoother output
const Matrix<float, 1, 1> BioMonitor::R = {
    4.0f  // Typical PPG sensor variance ~2-5 BPM^2
};

BioMonitor::BioMonitor()
{
    globalMonitor = this;
    
    // Initialize state estimate: assume resting HR of 70, no velocity
    x(0, 0) = 70.0f;  // Initial HR estimate
    x(1, 0) = 0.0f;   // Initial velocity (no change)
    
    // Initialize covariance with high uncertainty (we don't know the true state yet)
    P(0, 0) = 100.0f;  // HR variance
    P(0, 1) = 0.0f;
    P(1, 0) = 0.0f;
    P(1, 1) = 10.0f;   // Velocity variance
}

void BioMonitor::begin()
{
    Wire.begin();

    // Initialize sensors
    if (!ppg.init()) 
    {
        Serial.println("ERROR: MAX30102 initialization failed");
    }
    if (!imu.init()) 
    {
        Serial.println("ERROR: MPU6050 initialization failed");
    }

    // Configure Interrupt
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
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
    // MPU6050 motion interrupt - can be used for motion artifact detection
    // For now, just acknowledge the interrupt
    // TODO: Set motion flag to increase Kalman R during motion
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BioMonitor::runLoop()
{
    // Use shared SAMPLE_PERIOD_MS for consistent timing with Kalman dt
    const TickType_t samplePeriod = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Poll PPG sensor at consistent intervals
        vTaskDelayUntil(&lastWakeTime, samplePeriod);
        float bpm = ppg.processSample();
        if (bpm > 0.0f)
        {
            predictKalman();
            updateKalman(bpm);
        }
    }
    
    // TODO: Power management should be handled by a separate task/state machine
    // that coordinates sensor sleep, BLE sleep, and ESP32 light sleep modes
}

String BioMonitor::bleNotifyCallback(void* context)
{
    BioMonitor* monitor = static_cast<BioMonitor*>(context);
    float hr = monitor->getFilteredHR();
    return "HR=" + String(hr, 1);
}

// TODO: Fuse PPG with IMU data strictly for HR; motion data is not affected by HR, but motion may affect the PPG signal and thus HR.
// This can be done by dynamically increasing R when motion is detected, and returning to normal when motion is not detected.
void BioMonitor::predictKalman()
{
    // Predict state: x = F * x
    x = F * x;
    
    // Predict covariance: P = F * P * F^T + Q
    P = F * P * F.transpose() + Q;
}

void BioMonitor::updateKalman(float measurement)
{
    // Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
    // For 1D measurement, (H * P * H^T + R) is scalar, so we can simplify
    Matrix<float, 1, 1> S = H * P * H.transpose() + R;  // Innovation covariance
    float S_inv = 1.0f / S(0, 0);  // Scalar inverse (avoid matrix inverse for 1x1)
    
    Matrix<float, 2, 1> K;  // Kalman gain
    Matrix<float, 2, 1> PHt = P * H.transpose();
    K(0, 0) = PHt(0, 0) * S_inv;
    K(1, 0) = PHt(1, 0) * S_inv;
    
    // Innovation (measurement residual): y = z - H * x
    float y = measurement - (H * x)(0, 0);
    
    // Update state: x = x + K * y
    x(0, 0) = x(0, 0) + K(0, 0) * y;
    x(1, 0) = x(1, 0) + K(1, 0) * y;
    
    // Update covariance: P = (I - K * H) * P
    // Expanded: P = P - K * H * P
    Matrix<float, 2, 2> KH;
    KH(0, 0) = K(0, 0) * H(0, 0);  KH(0, 1) = K(0, 0) * H(0, 1);
    KH(1, 0) = K(1, 0) * H(0, 0);  KH(1, 1) = K(1, 0) * H(0, 1);
    
    Matrix<float, 2, 2> I = Matrix<float, 2, 2>::identity();
    P = (I - KH) * P;
}

float BioMonitor::getFilteredHR() const
{
    return x(0, 0);  // Return current HR estimate
}