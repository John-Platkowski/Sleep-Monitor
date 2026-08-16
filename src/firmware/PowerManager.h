// Decides how much of the device should be powered, from whether it is on a wearer and whether it is
// being moved.

#ifndef POWERMANAGER_H
#define POWERMANAGER_H

#include <Arduino.h>

// The power policy for the whole monitor: MONITORING on a wearer, IDLE off one but probing for a
// finger, DORMANT off one and untouched. DORMANT is terminal, since the caller leaves it through deep
// sleep and comes back through begin().
//
// Holds no hardware and touches no bus: the sampling task owns I2C by convention (see BioMonitor.h),
// so it asks this what to do on each tick and does it itself. Confine an instance to that task, apart
// from state(), which reads a single word.
//
// The states and every constant behind them are in docs/power-management.md.
class PowerManager
{
public:
    enum State
    {
        STATE_MONITORING,
        STATE_IDLE,
        STATE_DORMANT
    };

    // Seeds the machine for the session starting. Call once, before the sampling task runs.
    // wokeOnTimer shortens the first stillness window to a single probe; motionWakeAvailable false
    // disables DORMANT outright, which an IMU that failed to initialize needs.
    void begin(uint32_t nowMs, bool wokeOnTimer, bool motionWakeAvailable);

    // Advances one sample tick and returns the state to be in. Compare against state() read before
    // the call to spot a transition. Pass fingerPresent false unless the PPG was powered, which is
    // what ppgShouldBeAwake() reported last tick.
    State update(uint32_t nowMs, bool fingerPresent, bool motionEvent);

    // One word, so the BLE timer task may call this while the sampling task runs.
    State state() const { return currentState; }

    // Changes within a state as well as between them, since IDLE duty-cycles the PPG.
    bool ppgShouldBeAwake() const;

    // Withdraws a STATE_DORMANT decision the caller could not carry out. A device found moving on the
    // way into deep sleep is not one that should sleep.
    void cancelDormant(uint32_t nowMs);

private:
    State currentState;

    // Compared against millis() by unsigned subtraction, which survives the 49-day wrap and lets
    // begin() seed an already-expired window by subtracting its length.
    uint32_t lastFingerMs;
    uint32_t lastMotionMs;

    // Which half of IDLE's probe cycle is in progress. The powered window is counted in samples read,
    // the gap between windows against the clock.
    bool probing;
    uint32_t probeSamples;
    uint32_t probePhaseStartMs;

    bool dormantAllowed;
};

#endif
