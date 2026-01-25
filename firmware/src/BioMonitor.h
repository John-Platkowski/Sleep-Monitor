#ifndef BIOMONITOR_H
#define BIOMONITOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Matrix.h"
#include <Wire.h>

class BioMonitor {
public:
    BioMonitor();
    void begin();

private:
    TaskHandle_t taskHandle;
    QueueHandle_t sensorQueue;

    static void taskTrampoline(void *pvParameters);
    static void isrTrampoline();

    void runLoop();

    // Kalman State: [HeartRate, Velocity]
    Matrix<float, 2, 1> x;
    Matrix<float, 2, 2> P;

    void updateKalman(float measurement);
    void updateBLE();
};
