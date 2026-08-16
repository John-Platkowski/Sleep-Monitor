// BLE peripheral exposing sensor readings as a notifying GATT characteristic.

#pragma once
#ifndef BLEDRIVER_H
#define BLEDRIVER_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>   // required for CCCD descriptor
#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

// Randomly generated 128-bit UUIDs for a vendor-specific service. These are not the adopted Bluetooth
// SIG heart rate UUIDs, so generic heart rate clients will not recognize this device; the Python
// dashboard matches these values directly.
#define SERVICE_UUID        "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
#define CHARACTERISTIC_UUID "a16cba2a-8165-4039-96c6-06e922eb6551"

// Supplies the payload for each periodic notification.
//
// Invoked on the FreeRTOS timer service task, so implementations must be safe to call from a task
// other than the one producing the data, and must not block: stalling here delays every other
// software timer in the system.
using BLENotifyCallback = String (*)(void* context);

// Owns the BLE server, its single service, and the notification timer.
//
// Thread safety: none. init() and the notify controls are expected to run from the setup path, while
// the timer callback runs on the timer service task.
class BLEDriver
{
public:
    // Brings up the BLE stack, publishes the service, and starts advertising.
    //
    // Always returns true; the underlying stack calls do not report failure.
    bool init();

    // Stops advertising and notifications, and resumes them.
    //
    // Not used by the power manager, which keeps the device connectable for as long as it runs at all
    // and calls shutdown() when it stops.
    void sleep();
    void wake();

    // Releases the BLE stack outright: notification timer, advertising, the GATT server, and the
    // controller with them.
    //
    // Terminal, where sleep() is not: the server, service and characteristic go with the stack, so
    // only a reset can rebuild them. Meant for the moment before deep sleep.
    void shutdown();

    // Pushes one value to any subscribed client immediately.
    void notify(const String& data);

    // Starts calling callback every periodMs and notifying with what it returns, replacing any timer
    // already running.
    //
    // context is passed back to the callback untouched, and must outlive the timer.
    void startPeriodicNotify(uint32_t periodMs, BLENotifyCallback callback, void* context = nullptr);

    // Stops and destroys the notification timer. Safe to call when none is running.
    void stopPeriodicNotify();

    // Changes the interval of a running timer. Does nothing if none is running.
    void setNotifyPeriod(uint32_t periodMs);

private:
    // Owned by the BLE stack, which frees them; this class does not.
    BLEServer *pServer = nullptr;
    BLEService *pService = nullptr;
    BLECharacteristic *pCharacteristic = nullptr;

    TimerHandle_t notifyTimer = nullptr;
    BLENotifyCallback notifyCallback = nullptr;
    void* callbackContext = nullptr;

    // Timer entry point. Recovers the instance from the timer ID, since FreeRTOS timers carry one
    // void* of user data.
    static void timerCallback(TimerHandle_t xTimer);
};


#endif
