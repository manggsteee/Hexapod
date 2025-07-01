#include "ble_controller.hpp"

BLEController::CommandCallback::CommandCallback(BLEController* parent) : parent(parent) {}

void BLEController::CommandCallback::onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (!value.empty() && parent->commandHandler) {
        parent->commandHandler(value[0]);  // gọi callback xử lý lệnh ký tự đầu tiên
    }
}

void BLEController::init(const std::string& deviceName) {
    BLEDevice::init(deviceName);
    printf("📡 BLE MAC address: %s\n", BLEDevice::getAddress().toString().c_str());
    pServer = BLEDevice::createServer();

    BLEService *pService = pServer->createService("12345678-1234-1234-1234-123456789abc");

    pCharacteristic = pService->createCharacteristic(
        "abcd1234-abcd-1234-abcd-123456789abc",
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->setCallbacks(new CommandCallback(this));
    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    printf("🔵 Starting BLE advertising...\n");
    pAdvertising->start();
    printf("🔵 BLE advertising started successfully!\n");
}

void BLEController::loop() {
    // Không cần polling, vì BLE dùng callback.
}

void BLEController::sendMessage(const std::string& message) {
    if (pCharacteristic) {
        pCharacteristic->setValue(message);
        pCharacteristic->notify();
    }
}

void BLEController::setCommandHandler(void (*handler)(char)) {
    commandHandler = handler;
}
