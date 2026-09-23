// Implementation of the acquisition task and the heart rate Kalman filter.
//
// FreeRTOS reference: https://www.freertos.org/Documentation/00-Overview

#include "BioMonitor.h"
#include "Matrix.h"

#include <esp_sleep.h>
#include <driver/rtc_io.h>

// GPIO wired to the IMU's INT pin. Also the deep sleep wake source, so it has to be an RTC-capable
// pin; an ordinary GPIO is powered down with the digital core.
#define MPU_INT_PIN 25

// Sampling period for the whole pipeline, and the single source of truth for timing. 20ms is 50Hz;
// MAX30102Driver::init() configures the PPG to emit at the same rate, so the sensor never produces
// samples this loop cannot consume.
static constexpr uint32_t SAMPLE_PERIOD_MS = 20;

// Sample period in seconds, as the Kalman model works in BPM per second.
static constexpr float dt = SAMPLE_PERIOD_MS / 1000.0f;

// Reported in place of a heart rate whenever the device is not on a wearer.
static constexpr float NO_READING_HR = -1.0f;

// Wake-on-motion thresholds, in WOM_THR counts of 4mg each. 40mg while worn catches a sleeper rolling
// over; 160mg while dormant sits above what furniture transmits and below what handling produces.
static constexpr uint8_t WORN_WOM_THRESHOLD = 10;
static constexpr uint8_t DORMANT_WOM_THRESHOLD = 40;

// Accelerometer rate while dormant, as an LP_ACCEL_ODR code. 5 is 7.81Hz: 128ms of wake latency for
// about 23uA.
static constexpr uint8_t DORMANT_LP_ODR = 5;

// Bounds dormancy in case the IMU stops raising interrupts.
static constexpr uint64_t DORMANT_BACKSTOP_US = 3600ULL * 1000000ULL;

// Set by the constructor so the static ISR trampoline can reach the instance. Single-instance
// assumption.
BioMonitor* globalMonitor = nullptr;

// Fraction of the rate-of-change carried over between samples. Below 1.0 the velocity decays on its
// own, so an unmeasured filter drifts back toward a steady rate. Tuned; useful range is roughly 0.9
// to 0.98.
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
// while measurements agree with prediction and climbs toward HIGH when they stop agreeing. Tuned.
static constexpr float Q_VEL_BASE = 0.01f;
static constexpr float Q_VEL_HIGH = 0.1f;

// Innovation size in BPM above which a measurement is treated as a real change rather than noise.
// Tuned.
static constexpr float INNOVATION_THRESH = 6.0f;

// Smoothing factor for moving between the two Q values, per update.
static constexpr float ADAPT_RATE = 0.1f;

// How strongly motion inflates process noise: each g of movement multiplies Q by roughly this much.
// Tuned.
static constexpr float MOTION_Q_GAIN = 4.2f;

// Measurement noise R, the assumed variance of a beat-to-beat reading. Larger values smooth the
// output at the cost of lag. Tuned.
static constexpr float R_BASE = 3.8f;
const Matrix<float, 1, 1> BioMonitor::R = { R_BASE };

const Matrix<float, 2, 2> BioMonitor::I = Matrix<float, 2, 2>::identity();

BioMonitor::BioMonitor() : motionDetected(false), lastAccelMag(0.0f), motionPeak(0.0f),
                           ppgAwake(false), imuOk(false)
{
    globalMonitor = this;

    resetFilter();
}

// Returns the filter to the state a session starts in. Called on every entry to MONITORING, not only
// at construction.
void BioMonitor::resetFilter()
{
    // Start from a plausible resting rate.
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
    // Dormancy ends in a reset, so this is the only record of what the device was doing beforehand
    // and it has to be read before anything else clears it.
    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();

    // The IMU driver opens the shared I2C bus and must go first. Both failures are logged and
    // tolerated.
    imuOk = imu.init();
    if (!imuOk)
    {
        Serial.println("ERROR: MPU6050 initialization failed");
    }
    if (!ppg.init())
    {
        Serial.println("ERROR: MAX30102 initialization failed");
    }

    // init() leaves the PPG running.
    ppgAwake = true;

    imu.configureMotionInterrupt(WORN_WOM_THRESHOLD);

    // The INT pin idles high and pulses low, hence the pull-up and FALLING edge. Clear any latched
    // interrupt before attaching.
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    imu.clearInterrupt();
    delay(10);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), isrTrampoline, FALLING);

    // Before the task that reads it starts, and after imuOk is known.
    power.begin(millis(), wakeCause == ESP_SLEEP_WAKEUP_TIMER, imuOk);

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

// IRAM_ATTR: this can fire while the flash cache is disabled, during SPI flash writes or OTA.
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

        // Taken once per tick rather than by each reader, so the accelerometer read and the power
        // state machine cannot disagree about whether this tick saw movement.
        bool motionEvent = motionDetected;
        motionDetected = false;

        float accelMag = readAccelIfMotion(motionEvent);
        imu.addTemperatureSample(millis());

        // A shut down PPG keeps answering on I2C while producing nothing, and processSample() costs
        // the library's 250ms FIFO timeout per call.
        float bpm = NO_READING_HR;
        bool fingerPresent = false;
        if (ppgAwake)
        {
            bpm = ppg.processSample();
            fingerPresent = ppg.fingerPresent();
        }

        PowerManager::State previous = power.state();

        // Only worth advancing while there is a wearer to advance it about.
        if (previous == PowerManager::STATE_MONITORING)
        {
            // Predict every tick, so the estimate stays aligned with real time through stretches where no beat is detected.
            predictKalman(accelMag);

            // Correct only when there is something to correct with. A negative return means no beat
            // on this sample, which is the common case at 50hz: beats arrive roughly once every 50 ticks.
            if (bpm > 0.0f)
            {
                updateKalman(bpm);
            }
        }

        applyPowerState(previous, power.update(millis(), fingerPresent, motionEvent));
    }
}

float BioMonitor::readAccelIfMotion(bool motionEvent)
{
    // Only read IMU when motion interrupt has fired
    if (motionEvent)
    {
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

    // Ease toward zero, so the filter's trust in the sensor returns gradually as the wearer settles.
    lastAccelMag *= 0.95f;
    return lastAccelMag;
}

void BioMonitor::applyPowerState(PowerManager::State previous, PowerManager::State next)
{
    if (next != previous)
    {
        if (next == PowerManager::STATE_MONITORING)
        {
            resetFilter();
        }
        else if (next == PowerManager::STATE_DORMANT)
        {
            // Returns only if it decided not to sleep after all, having touched nothing and put the
            // state machine back into IDLE, so the PPG handling below still sees a current answer.
            enterDormant();
        }
    }

    // The PPG is the one part whose power changes within a state as well as between them.
    bool wantPpgAwake = power.ppgShouldBeAwake();
    if (wantPpgAwake != ppgAwake)
    {
        if (wantPpgAwake)
        {
            ppg.wake();
        }
        else
        {
            ppg.sleep();
        }
        ppgAwake = wantPpgAwake;
    }
}

// Shuts the device down and enters deep sleep, from which the only exit is a reset. Deep sleep rather
// than light sleep, and the ordering below, are both explained in docs/power-management.md.
void BioMonitor::enterDormant()
{
    // Before touching anything, so changing our mind is free. ext0 wakes on a level, not an edge, and
    // the INT pin latches low until INT_STATUS is read; clearing the latch and finding it low anyway
    // means the device is moving right now.
    imu.clearInterrupt();
    delay(10);
    if (digitalRead(MPU_INT_PIN) == LOW)
    {
        power.cancelDormant(millis());
        return;
    }

    Serial.println("Going dormant: no wearer and no movement");
    Serial.flush();

    // The sensors keep their own supply through an ESP32 deep sleep, so what is left running here
    // sets the floor for the whole device rather than the chip's own current.
    ppg.sleep();
    ppgAwake = false;

    // Released, not slept: deep sleep stops the radio's clocks underneath the controller.
    ble.shutdown();

    // Stays awake because it is the wake source, at roughly 23uA rather than 450uA.
    imu.enterLowPowerMotion(DORMANT_WOM_THRESHOLD, DORMANT_LP_ODR);

    // Clear the latch once more, since reconfiguring the accelerometer can leave a comparison pending.
    detachInterrupt(digitalPinToInterrupt(MPU_INT_PIN));
    imu.clearInterrupt();

    esp_sleep_enable_ext0_wakeup((gpio_num_t)MPU_INT_PIN, 0);

    // Insurance against an IMU that stops driving this push-pull pin; begin()'s pull-up is a
    // digital-domain one and does not survive deep sleep.
    rtc_gpio_pullup_en((gpio_num_t)MPU_INT_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)MPU_INT_PIN);

    esp_sleep_enable_timer_wakeup(DORMANT_BACKSTOP_US);

    // Does not return; the next thing this program does is setup().
    esp_deep_sleep_start();
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
// is not describing what is happening, whether from a real change in rate or a corrupted reading.
void BioMonitor::adaptProcessNoise(float innovation)
{
    float targetQVel = (std::abs(innovation) > INNOVATION_THRESH) ? Q_VEL_HIGH : Q_VEL_BASE;

    // Ease toward the target rather than stepping to it, so one anomalous beat cannot flip the filter
    // into its loose regime.
    Q_adaptive(1, 1) += ADAPT_RATE * (targetQVel - Q_adaptive(1, 1));
}

float BioMonitor::getFilteredHR() const
{
    // Outside MONITORING the filter is not advancing and x holds whatever the last wearer left in it.
    if (power.state() != PowerManager::STATE_MONITORING)
    {
        return NO_READING_HR;
    }

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