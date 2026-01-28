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
    stopPeriodicNotify();
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

void BLEDriver::startPeriodicNotify(uint32_t periodMs, BLENotifyCallback callback, void* context)
{
    notifyCallback = callback;
    callbackContext = context;

    if (notifyTimer != nullptr) 
    {
        xTimerStop(notifyTimer, 0);
        xTimerDelete(notifyTimer, 0);
    }

    notifyTimer = xTimerCreate(
        "BLE_Notify",       // Timer name
        pdMS_TO_TICKS(periodMs),    // Period in ticks
        pdTRUE,     // Auto-reload (repeating)
        this,   // Timer ID (pass 'this' pointer)
        timerCallback   // Callback function
    );

    if (notifyTimer != nullptr) 
    {
        xTimerStart(notifyTimer, 0);
        Serial.println("BLE periodic notify started (" + String(periodMs) + "ms)");
    }
}

void BLEDriver::stopPeriodicNotify()
{
    if (notifyTimer != nullptr) 
    {
        xTimerStop(notifyTimer, 0);
        xTimerDelete(notifyTimer, 0);
        notifyTimer = nullptr;
        Serial.println("BLE periodic notify stopped");
    }
}

void BLEDriver::setNotifyPeriod(uint32_t periodMs)
{
    if (notifyTimer != nullptr) 
    {
        xTimerChangePeriod(notifyTimer, pdMS_TO_TICKS(periodMs), 0);
        Serial.println("BLE notify period changed to " + String(periodMs) + "ms");
    }
}

void BLEDriver::timerCallback(TimerHandle_t xTimer)
{
    // Retrieve the BLEDriver instance from the timer ID
    BLEDriver* driver = static_cast<BLEDriver*>(pvTimerGetTimerID(xTimer));
    
    if (driver && driver->notifyCallback && driver->pCharacteristic) 
    {
        String data = driver->notifyCallback(driver->callbackContext);
        driver->pCharacteristic->setValue(data.c_str());
        driver->pCharacteristic->notify();
        Serial.println("BLE notified: " + data);
    }
}