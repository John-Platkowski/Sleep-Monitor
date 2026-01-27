#include "BLEDriver.h"

bool BLEDriver::init()
{
    // Set up serial communication
    Serial.begin(115200);
    // Create the BLE Device
    BLEDevice::init("MyESP32");

    // Create the server and service
    pServer = BLEDevice::createServer();
    pService = pServer->createService(SERVICE_UUID);

    // Create characteristic with Notify + Read
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    // Enable notifications for the CCCD of the characteristic
    BLE2902 *cccd = new BLE2902();
    cccd->setNotifications(true);
    pCharacteristic->addDescriptor(cccd);

    pCharacteristic->setValue("Hello from ESP32");

    // Start service and advertising
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("BLE advertising started...");
    return true;
}

void BLEDriver::sleep()
{
    pServer->stopAdvertising();
    pService->stop();
    pCharacteristic->setValue("Sleeping...");
    pCharacteristic->notify();
    Serial.println("BLE sleeping...");
}

void BLEDriver::wake()
{
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising started...");
}

void BLEDriver::notify(const String& data)
{
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
    Serial.println("BLE notified: " + data);
}