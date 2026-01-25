#include "BioMonitor.h"
//https://www.freertos.org/Documentation/00-Overview

#define MPU_INT_PIN 25

BioMonitor* globalMonitor = nullptr; 

BioMonitor::BioMonitor()
{
    globalMonitor = this;
    // Initialize Matrix values
}

void BioMonitor::begin()
{
    Wire.begin();
    sensorQueue = xQueueCreate(10, sizeof(float));

    // Configure Interrupt
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), isrTrampoline, FALLING);

    // Function, Name, Stack Depth, *pvParameters, Priority, Handle
    xTaskCreate(taskTrampoline, "BioTask", 4096, this, 1, &taskHandle);

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
    vTaskDelete(NULL);
}

void BioMonitor::handleISR()
{
    float val = 60.0; // Cook the data for now; should be sensor.read()
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(sensorQueue, &rawVal, &xHigherPriorityTaskWoken);
    
    // If the kalman task is waiting, switch immediately
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BioMonitor::runLoop()
{
    float measurement;

    while (true)
    {
        // Sleep until the ISR sends data to the queue
        if (xQueueReceive(sensorQueue, &measurement, portMAX_DELAY) == pdTRUE)
        {
            updateKalman(measurement);
        }
    }
}

void BioMonitor::updateKalman()
{
    
}

void BioMonitor::updateBLE()
{

}