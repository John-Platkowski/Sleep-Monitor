#include "BioMonitor.h"

BioMonitor monitor;

void setup()
{
    Serial.begin(115200);
    monitor.begin();
}

void loop()
{
    delay(1000);
}