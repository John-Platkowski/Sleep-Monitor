#ifndef BIOMONITOR_H
#define BIOMONITOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Matrix.h"
#include <Wire.h>
#include "MAX30102Driver.h"
#include "MPU6050Driver.h"

class BioMonitor {
public:
    BioMonitor();
    void begin();

private:
    TaskHandle_t taskHandle;
    QueueHandle_t sensorQueue;

    static void taskTrampoline(void *pvParameters);
    static void isrTrampoline();
    void handleISR();

    void runLoop();

    // Kalman State: [HeartRate, Velocity]
    Matrix<float, 2, 1> x;
    Matrix<float, 2, 2> P;

    void updateKalman(float measurement);
    void updateBLE();

    MAX30102Driver ppg;
    MPU6050Driver imu;
};

#endif