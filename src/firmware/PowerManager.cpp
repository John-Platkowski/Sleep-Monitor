// Implementation of the power state machine. Every constant here is derived rather than picked; the
// derivations are in docs/power-management.md.

#include "PowerManager.h"

// Time below the PPG's finger threshold before the device counts as off the wearer.
static constexpr uint32_t NO_FINGER_TIMEOUT_MS = 30000;

// Stillness required, already off the wearer, before powering down.
static constexpr uint32_t STILL_TIMEOUT_MS = 300000;

// IDLE's finger probe: five samples every two seconds, a 5% duty cycle on the PPG. Counted in samples
// rather than milliseconds because the read blocks until the freshly woken sensor produces one.
static constexpr uint32_t PROBE_WINDOW_SAMPLES = 5;
static constexpr uint32_t PROBE_INTERVAL_MS = 2000;

void PowerManager::begin(uint32_t nowMs, bool wokeOnTimer, bool motionWakeAvailable)
{
    currentState = STATE_IDLE;
    dormantAllowed = motionWakeAvailable;

    lastFingerMs = nowMs;
    lastMotionMs = nowMs;

    // Open a probe window immediately rather than waiting out an interval: the caller's init() leaves
    // the PPG running.
    probing = true;
    probeSamples = 0;
    probePhaseStartMs = nowMs;

    if (wokeOnTimer)
    {
        // Start the stillness window already spent, so one empty probe sends the device back down.
        // Unsigned, so wrapping below zero here still compares correctly.
        lastMotionMs = nowMs - STILL_TIMEOUT_MS;
    }
}

PowerManager::State PowerManager::update(uint32_t nowMs, bool fingerPresent, bool motionEvent)
{
    if (motionEvent)
    {
        lastMotionMs = nowMs;
    }
    if (fingerPresent)
    {
        lastFingerMs = nowMs;
    }

    switch (currentState)
    {
    case STATE_MONITORING:
        if (nowMs - lastFingerMs >= NO_FINGER_TIMEOUT_MS)
        {
            currentState = STATE_IDLE;

            // Enter on the unpowered half of the cycle, so the PPG shuts down now.
            probing = false;
            probeSamples = 0;
            probePhaseStartMs = nowMs;
        }
        break;

    case STATE_IDLE:
        if (probing)
        {
            if (fingerPresent)
            {
                // The sensor is already powered and sampling, so monitoring resumes with no warm-up.
                currentState = STATE_MONITORING;
                probing = false;
                break;
            }

            // Reached only once the PPG is powered, since the tick that opens the window takes the
            // branch below and the caller powers the sensor up after this returns. Every count is
            // therefore a sample actually read.
            if (++probeSamples >= PROBE_WINDOW_SAMPLES)
            {
                probing = false;
                probePhaseStartMs = nowMs;
            }
        }
        else if (nowMs - probePhaseStartMs >= PROBE_INTERVAL_MS)
        {
            probing = true;
            probeSamples = 0;
        }

        // Reaching IDLE already required NO_FINGER_TIMEOUT_MS without a finger, so stillness is all
        // that is left to test. Never mid-probe, which is the one chance to notice a finger returning.
        if (dormantAllowed && !probing && (nowMs - lastMotionMs >= STILL_TIMEOUT_MS))
        {
            currentState = STATE_DORMANT;
        }
        break;

    case STATE_DORMANT:
        // Terminal; the reset that ends deep sleep brings the machine back through begin().
        break;
    }

    return currentState;
}

bool PowerManager::ppgShouldBeAwake() const
{
    switch (currentState)
    {
    case STATE_MONITORING:
        return true;
    case STATE_IDLE:
        return probing;
    default:
        return false;
    }
}

void PowerManager::cancelDormant(uint32_t nowMs)
{
    currentState = STATE_IDLE;

    // The caller only refuses a power-down because it found the device moving.
    lastMotionMs = nowMs;

    probing = false;
    probeSamples = 0;
    probePhaseStartMs = nowMs;
}
