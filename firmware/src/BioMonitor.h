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

class BioMonitor 
{
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

    // Kalman Filter for Heart Rate
    // State: [HR, HR_velocity]^T - tracks heart rate and its rate of change
    Matrix<float, 2, 1> x;      // State estimate
    Matrix<float, 2, 2> P;      // Estimate covariance
    
    // System model (constant, defined in cpp)
    static const Matrix<float, 2, 2> F;  // State transition
    static const Matrix<float, 1, 2> H;  // Measurement matrix
    static const Matrix<float, 2, 2> Q;  // Process noise covariance
    static const Matrix<float, 1, 1> R;  // Measurement noise covariance

    void predictKalman();
    void updateKalman(float measurement);
    float getFilteredHR() const;
    
    void updateBLE();

    MAX30102Driver ppg;
    MPU6050Driver imu;
};

#endif