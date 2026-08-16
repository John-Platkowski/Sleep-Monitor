// Arduino entry point for the sleep monitor firmware.
//
// All real work happens in FreeRTOS tasks that BioMonitor::begin() creates, so loop() has nothing to
// do. See BioMonitor.h for the task layout.

#include "BioMonitor.h"

// Global, so it outlives setup() and lands in static storage rather than on the Arduino task's stack.
BioMonitor monitor;

void setup()
{
    // Owns serial setup for the whole program; nothing else may call begin().
    Serial.begin(115200);
    monitor.begin();
}

void loop()
{
    // The sampling task and the BLE timer carry the workload. Sleeping keeps this task off the CPU.
    delay(1000);
}