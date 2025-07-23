#include "ble_controller.hpp"
#include <esp32-hal.h>
#include <algorithm>

namespace hexapod {

BLEController::CommandCallback::CommandCallback(BLEController *parent) : parent(parent) {}

void BLEController::ServerCallbacks::onConnect(BLEServer *pServer)
{
    printf("🔵 Client connected!\n");
}

void BLEController::ServerCallbacks::onDisconnect(BLEServer *pServer)
{
    printf("🔵 Client disconnected!\n");

    // Luôn khởi động lại quảng cáo ngay sau khi mất kết nối
    delay(100); // Chờ một chút trước khi restart advertising
    BLEDevice::startAdvertising();
    parent->_lastConnectionAttempt = millis();
    printf("🔵 Restarted advertising. Ready for connections...\n");
}


void BLEController::CommandCallback::onWrite(BLECharacteristic *pCharacteristic)
{
    std::string value = pCharacteristic->getValue();
    if (!value.empty())
    {
        // Kiểm tra xem dữ liệu có chứa dấu phẩy không (định dạng x,y)
        size_t commaPos = value.find(',');
        
        if (commaPos != std::string::npos && commaPos > 0 && commaPos < value.length() - 1)
        {
            // Có thể là dữ liệu joystick x,y
            try
            {
                std::string xStr = value.substr(0, commaPos);
                std::string yStr = value.substr(commaPos + 1);
                
                // Đọc giá trị raw
                float rawX = atof(xStr.c_str());
                float rawY = atof(yStr.c_str());
                
                // Thêm deadzone tại nguồn để giảm nhiễu
                // const float DEADZONE = 0.1f;
                // if (abs(rawX) < DEADZONE) rawX = 0.0f;
                // if (abs(rawY) < DEADZONE) rawY = 0.0f;
                
                // // Giới hạn giá trị vào khoảng [-1, 1]
                // // parent->_joystickX = std::max(-1.0f, std::min(1.0f, rawX));
                // // parent->_joystickY = std::max(-1.0f, std::min(1.0f, rawY));
                
                // // Áp dụng lọc cho joystick để tăng độ mượt
                // const float SMOOTHING = 0.3f; // 0.0 = không lọc, 1.0 = lọc hoàn toàn
                // parent->_joystickX = parent->_joystickX * (1-SMOOTHING) + rawX * SMOOTHING;
                // parent->_joystickY = parent->_joystickY * (1-SMOOTHING) + rawY * SMOOTHING;

                parent->_joystickX = rawX;
                parent->_joystickY = rawY;

                parent->_hasNewJoystickData = true;
                
                // Debug
                printf("Joystick data: X=%.2f, Y=%.2f\n", parent->_joystickX, parent->_joystickY);
            }
            catch (...)
            {
                printf("Error parsing joystick data\n");
            }
        }
        else if (value.length() == 1)
        {
            // Xử lý lệnh đơn ký tự
            parent->_lastCommand = value[0];
            parent->_hasNewCommand = true;
            
            // Gọi callback xử lý nếu có
            if (parent->commandHandler)
            {
                parent->commandHandler(value[0]);
            }
        }
        else
        {
            // Không phải dữ liệu hợp lệ
            printf("Invalid data format: %s\n", value.c_str());
        }
    }
}

void BLEController::init(const std::string &deviceName)
{
    BLEDevice::init(deviceName);
    printf("📡 BLE MAC address: %s\n", BLEDevice::getAddress().toString().c_str());
    pServer = BLEDevice::createServer();

    pServer->setCallbacks(new ServerCallbacks(this)); // Thêm dòng này

    BLEService *pService = pServer->createService("12345678-1234-1234-1234-123456789abc");

    pCharacteristic = pService->createCharacteristic(
        "abcd1234-abcd-1234-abcd-123456789abc",
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );

    pCharacteristic->setCallbacks(new CommandCallback(this));
    pCharacteristic->addDescriptor(new BLE2902());

    BLEDevice::setMTU(512); // Thêm dòng này

    // Tăng giá trị SUPERVISION_TIMEOUT
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Giúp iPhone phát hiện thiết bị
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->addServiceUUID("12345678-1234-1234-1234-123456789abc");
    BLEAdvertisementData advData;
    advData.setFlags(0x06); // General Discoverable + BR/EDR Not Supported
    advData.setName(deviceName);
    pAdvertising->setAdvertisementData(advData);
    printf("🔵 Starting BLE advertising...\n");
    pAdvertising->start();
    printf("🔵 BLE advertising started successfully!\n");
    _autoConnectEnabled = true;
    _lastConnectionAttempt = millis();
}

void BLEController::loop()
{
     if (_autoConnectEnabled && !isConnected()) {
        unsigned long currentTime = millis();

        // Thử kết nối lại mỗi 10 giây
        if (currentTime - _lastConnectionAttempt > 10000) {
            printf("🔵 No connection. Restarting advertising...\n");
            BLEDevice::startAdvertising();
            _lastConnectionAttempt = currentTime;
        }
    }

    static unsigned long lastKeepAlive = 0;
    if (isConnected() && millis() - lastKeepAlive > 2000) {
        // Gửi gói keep-alive mỗi 2 giây
        sendMessage("PING");
        lastKeepAlive = millis();
    }
}

void BLEController::sendMessage(const std::string &message)
{
    if (pCharacteristic)
    {
        pCharacteristic->setValue(message);
        pCharacteristic->notify();
    }
}


void BLEController::setCommandHandler(std::function<void(char)> handler)
{
    commandHandler = handler;
}

// Trong file ble_controller.cpp
bool BLEController::hasCommand()
{
    return _hasNewCommand;
}

char BLEController::getCommand()
{
    if (_hasNewCommand)
    {
        _hasNewCommand = false;
        return _lastCommand;
    }
    return 0;
}

bool BLEController::isConnected() const
{
    return pServer != nullptr && pServer->getConnectedCount() > 0;
}

bool BLEController::hasJoystickData()
{
    return _hasNewJoystickData;
}

float BLEController::getJoystickX()
{
    _hasNewJoystickData = false; // Reset flag sau khi đọc
    return _joystickX;
}

float BLEController::getJoystickY()
{
    return _joystickY;
}

void BLEController::enableAutoConnect(bool enable) {
    _autoConnectEnabled = enable;
    printf("🔵 Auto-connect %s\n", enable ? "enabled" : "disabled");
    
    if (enable && !isConnected()) {
        // Nếu bật auto-connect và chưa kết nối, bắt đầu quảng cáo ngay
        BLEDevice::startAdvertising();
        _lastConnectionAttempt = millis();
    }
}

} // namespace hexapod
