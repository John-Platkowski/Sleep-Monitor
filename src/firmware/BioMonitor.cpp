// Implementation of the acquisition task and the heart rate Kalman filter.
//
// Constants marked "Tuned" were arrived at by running the filter against live signal and adjusting
// until the output tracked without oscillating. None have been validated against a reference heart
// rate monitor, so treat them as working defaults rather than characterized values.
//
// FreeRTOS reference: https://www.freertos.org/Documentation/00-Overview

#include "BioMonitor.h"
#include "Matrix.h"

// GPIO wired to the IMU's INT pin.
#define MPU_INT_PIN 25

// Sampling period for the whole pipeline, and the single source of truth for timing. 20ms is 50Hz;
// MAX30102Driver::init() configures the PPG to emit at the same rate, so the sensor never produces
// samples this loop cannot consume.
static constexpr uint32_t SAMPLE_PERIOD_MS = 20;

// Sample period in seconds, as the Kalman model works in BPM per second.
static constexpr float dt = SAMPLE_PERIOD_MS / 1000.0f;

// Set by the constructor so the static ISR trampoline can reach the instance. Single-instance
// assumption: a second BioMonitor would steal the interrupt.
BioMonitor* globalMonitor = nullptr;

// Fraction of the rate-of-change carried over between samples. Below 1.0 the velocity decays on its
// own, so an unmeasured filter drifts back toward a steady rate instead of extrapolating a trend
// forever. Tuned; useful range is roughly 0.9 to 0.98.
static constexpr float VELOCITY_DECAY = 0.96f;

// State transition matrix F, one sample of the constant-velocity model:
//
//   [HR_new ]   [1  dt   ] [HR ]     HR_new  = HR + dt * velocity
//   [vel_new] = [0  decay] [vel]     vel_new = decay * velocity
const Matrix<float, 2, 2> BioMonitor::F = {
    1.0f, dt,
    0.0f, VELOCITY_DECAY
};

// Ceiling on the rate of change, in BPM per second. Heart rate rarely moves faster than this even
// under exertion, so anything beyond it is filter divergence rather than physiology.
static constexpr float MAX_HR_VELOCITY = 5.0f;

// Measurement matrix H. Only heart rate is observed, never velocity directly: z = [1 0] * [HR, vel]^T.
const Matrix<float, 1, 2> BioMonitor::H = {
    1.0f, 0.0f
};

// Baseline process noise Q, the model's admitted error per sample. Larger values make the filter
// trust incoming measurements more and its own prediction less. Tuned.
const Matrix<float, 2, 2> BioMonitor::Q = {
    0.014f, 0.0f,
    0.0f,  0.014f
};

// Velocity process noise, switched between these two by adaptProcessNoise(). The filter sits at BASE
// while measurements agree with prediction and climbs toward HIGH when they stop agreeing, which is
// how it recovers from a genuine change in rate instead of dismissing it as noise. Tuned.
static constexpr float Q_VEL_BASE = 0.01f;
static constexpr float Q_VEL_HIGH = 0.1f;

// Innovation size in BPM above which a measurement is treated as a real change rather than noise.
// Tuned.
static constexpr float INNOVATION_THRESH = 6.0f;

// Smoothing factor for moving between the two Q values, per update.
static constexpr float ADAPT_RATE = 0.1f;

// How strongly motion inflates process noise: each g of movement multiplies Q by roughly this much.
// This is the mechanism that makes the filter distrust the optical signal while the wearer moves.
// Tuned.
static constexpr float MOTION_Q_GAIN = 4.2f;

// Measurement noise R, the assumed variance of a beat-to-beat reading. Larger values smooth the
// output at the cost of lag. Tuned.
static constexpr float R_BASE = 3.8f;
const Matrix<float, 1, 1> BioMonitor::R = { R_BASE };

const Matrix<float, 2, 2> BioMonitor::I = Matrix<float, 2, 2>::identity();

BioMonitor::BioMonitor() : motionDetected(false), lastAccelMag(0.0f), motionPeak(0.0f)
{
    globalMonitor = this;

    // Start from a plausible resting rate rather than zero, so the first measurements correct a
    // reasonable guess instead of dragging the estimate up from nothing.
    x(0, 0) = 70.0f;  // BPM.
    x(1, 0) = 0.0f;   // BPM/s, assumed steady.

    // Open with high uncertainty, so the filter leans on the first real measurements. P shrinks on
    // its own as they arrive.
    P(0, 0) = 100.0f;  // Heart rate variance.
    P(0, 1) = 0.0f;
    P(1, 0) = 0.0f;
    P(1, 1) = 10.0f;   // Velocity variance.

    Q_adaptive(0, 0) = Q(0, 0);
    Q_adaptive(0, 1) = 0.0f;
    Q_adaptive(1, 0) = 0.0f;
    Q_adaptive(1, 1) = Q_VEL_BASE;
}

void BioMonitor::begin()
{
    // The IMU driver opens the shared I2C bus and must go first. Both failures are
    // logged and tolerated, since a partly working monitor is more useful than none.
    if (!imu.init())
    {
        Serial.println("ERROR: MPU6050 initialization failed");
    }
    if (!ppg.init())
    {
        Serial.println("ERROR: MAX30102 initialization failed");
    }

    // 10 counts * 4mg = roughly 40mg, low enough to catch a wearer rolling over but above the
    // sensor's noise floor at rest.
    imu.configureMotionInterrupt(10);

    // The INT pin idles high and pulses low, hence the pull-up and FALLING edge. Clear any latched
    // interrupt before attaching, or a stale assertion fires the handler immediately and never re-arms.
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    imu.clearInterrupt();
    delay(10);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), isrTrampoline, FALLING);

    // Priority 1 leaves the filter below the timer service task, so BLE notifications are not delayed
    // by a sample in progress.
    // (function, name, stack depth in words, parameter, priority, handle)
    xTaskCreate(taskTrampoline, "BioTask", 4096, this, 1, &taskHandle);

    ble.init();
    ble.startPeriodicNotify(BLE_NOTIFY_PERIOD_MS, bleNotifyCallback, this);
}

void BioMonitor::taskTrampoline(void *pvParameters)
{
    BioMonitor* instance = static_cast<BioMonitor*>(pvParameters);
    instance -> runLoop();
    //vTaskDelete(NULL);
}

// IRAM_ATTR: this can fire while the flash cache is disabled, during SPI flash writes or OTA, and a
// handler living in flash would fault there.
void IRAM_ATTR BioMonitor::isrTrampoline()
{
    if (globalMonitor) globalMonitor -> handleISR();
}

void IRAM_ATTR BioMonitor::handleISR()
{
    // Only raises a flag, so there is no woken task to yield to. The sampling task picks it up on its
    // next tick.
    motionDetected = true;
}

void BioMonitor::runLoop()
{
    const TickType_t samplePeriod = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // The wake time is derived from the last wake, so the loop maintains 50hz and doesn't rely on body time
        vTaskDelayUntil(&lastWakeTime, samplePeriod);

        float accelMag = readAccelIfMotion();
        float bpm = ppg.processSample();
        imu.addTemperatureSample(millis());

        // Predict every tick, so the estimate stays aligned with real time through stretches where no beat is detected.
        predictKalman(accelMag);

        // Correct only when there is something to correct with. A negative return means no beat
        // on this sample, which is the common case at 50hz: beats arrive roughly once every 50 ticks.
        if (bpm > 0.0f)
        {
            updateKalman(bpm);
        }
    }
    // TODO: Move power management into its own task or state machine to coordinate sensor sleep, BLE
    // sleep, and ESP32 light sleep. The driver sleep() and wake() methods exist but nothing calls
    // them, so the device currently runs at full power indefinitely.
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

        // Hold the peak for the BLE report, since lastAccelMag decays roughly 92% across a 1hz notify period.
        portENTER_CRITICAL(&motionMux);
        if (lastAccelMag > motionPeak)
        {
            motionPeak = lastAccelMag;
        }
        portEXIT_CRITICAL(&motionMux);

        // Clear the hardware interrupt latch.
        imu.clearInterrupt();

        return lastAccelMag;
    }

    // Ease toward zero rather than dropping to it, so the filter's trust in the sensor returns
    // gradually as the wearer settles.
    lastAccelMag *= 0.95f;
    return lastAccelMag;
}

String BioMonitor::bleNotifyCallback(void* context)
{
    BioMonitor* monitor = static_cast<BioMonitor*>(context);
    float hr = monitor->getFilteredHR();
    float motion = monitor->getMotionScore();
    float tempC = monitor->getEpochTemperatureC();

    // Static to keep the buffer off the timer task's modest stack. Safe only because the timer service
    // task is the sole caller and never reenters.
    static char buffer[80];
    snprintf(buffer, sizeof(buffer), "HR=%.1f, Motion=%.2f, Temp=%.1f", hr, motion, tempC);
    return String(buffer);
}

void BioMonitor::predictKalman(float accelMag)
{
    // Project the state forward one sample: x = F * x.
    x = F * x;

    if (x(1, 0) > MAX_HR_VELOCITY) x(1, 0) = MAX_HR_VELOCITY;
    if (x(1, 0) < -MAX_HR_VELOCITY) x(1, 0) = -MAX_HR_VELOCITY;

    // Inflate process noise in proportion to movement: a larger Q widens P, which shrinks the gain
    // computed in updateKalman(). At rest the factor is 1.0 and the filter behaves conventionally.
    float motionFactor = 1.0f + accelMag * MOTION_Q_GAIN;
    Q_scratch(0, 0) = Q_adaptive(0, 0) * motionFactor;
    Q_scratch(0, 1) = 0.0f;
    Q_scratch(1, 0) = 0.0f;
    Q_scratch(1, 1) = Q_adaptive(1, 1) * motionFactor;

    // Project the covariance: P = F * P * F^T + Q.
    P = F * P * F.transpose() + Q_scratch;
}

void BioMonitor::updateKalman(float measurement)
{
    // Innovation, the surprise in this measurement: y = z - H * x.
    y(0, 0) = measurement - (H * x)(0, 0);

    adaptProcessNoise(y(0, 0));

    // Innovation covariance: S = H * P * H^T + R.
    S = H * P * H.transpose() + R;

    // Kalman gain: K = P * H^T * S^-1, the ratio of the filter's own uncertainty to the total.
    K = P * H.transpose() * S.inverse();

    // Correct the state in proportion to that gain: x = x + K * y.
    x = x + K * y;

    // Clamp again, since a large innovation can push velocity past the limit even when the prediction
    // was inside it.
    if (x(1, 0) > MAX_HR_VELOCITY) x(1, 0) = MAX_HR_VELOCITY;
    if (x(1, 0) < -MAX_HR_VELOCITY) x(1, 0) = -MAX_HR_VELOCITY;

    // Shrink the covariance to reflect what was just learned: P = (I - K * H) * P.
    P = (I - K * H) * P;
}

// Widens process noise when measurements stop matching prediction. A large innovation means the model
// is not describing what is happening, whether from a real change in rate or a corrupted reading, and
// in both cases holding the model tightly would keep the estimate stale.
//
// TODO: Replace innovation-based adaptation with sleep-stage-dependent Q once sleep staging exists.
// N1 and N2 carry higher heart rate variability and would suit a higher Q_VEL, while N3 and REM are
// more stable.
void BioMonitor::adaptProcessNoise(float innovation)
{
    float targetQVel = (std::abs(innovation) > INNOVATION_THRESH) ? Q_VEL_HIGH : Q_VEL_BASE;

    // Ease toward the target rather than stepping to it, so one anomalous beat cannot flip the filter
    // into its loose regime.
    Q_adaptive(1, 1) += ADAPT_RATE * (targetQVel - Q_adaptive(1, 1));
}

float BioMonitor::getFilteredHR() const
{
    return x(0, 0);  // Return current HR true state
}

float BioMonitor::getMotionScore()
{
    portENTER_CRITICAL(&motionMux);
    float peak = motionPeak;
    motionPeak = 0.0f;
    portEXIT_CRITICAL(&motionMux);
    return peak;
}

float BioMonitor::getEpochTemperatureC()
{
    return imu.getEpochTemperatureC(millis(), BLE_NOTIFY_PERIOD_MS);
}