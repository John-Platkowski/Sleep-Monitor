#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>   // <<-- required for CCCD descriptor

#define SERVICE_UUID        "66b53535-8ebb-4a24-bad7-ed67ebb935a2"
#define CHARACTERISTIC_UUID "a16cba2a-8165-4039-96c6-06e922eb6551"

BLEServer *pServer = nullptr;
BLEService *pService = nullptr;
BLECharacteristic *pCharacteristic = nullptr;

void setup() 
{
  Serial.begin(115200);
  BLEDevice::init("MyESP32");

  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);

  // Create characteristic with Notify + Read
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

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
}

void loop() 
{
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis < 1000) return; // notify once/sec
  lastMillis = millis();

  if (!pCharacteristic) return;

  static int counter = 0;
  counter++;

  String msg = "IR=" + String(counter * 1000) +
               ", BPM=" + String(60 + (counter % 10)) +
               ", Avg BPM=" + String(65);

  // update value and notify subscribed clients
  pCharacteristic->setValue(msg.c_str());
  pCharacteristic->notify();

  Serial.println("Notified: " + msg);
}
