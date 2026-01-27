#pragma once
#ifndef BLEDRIVER_H
#define BLEDRIVER_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>   // required for CCCD descriptor
#include <Arduino.h>
#include <Wire.h>


#define SERVICE_UUID        "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
#define CHARACTERISTIC_UUID "a16cba2a-8165-4039-96c6-06e922eb6551"

class BLEDriver
{
public:
    bool init()
    {
        return true;
    }
    void sleep();
    void wake();
    void notify(const String& data);


private:
    BLEServer *pServer = nullptr;
    BLEService *pService = nullptr;
    BLECharacteristic *pCharacteristic = nullptr;



};





#endif
