#ifndef BLE_CONTROLLER_HPP
#define BLE_CONTROLLER_HPP

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class BLEController
{
public:
    void init(const std::string &deviceName);
    void loop(); // xử lý dữ liệu từ BLE

    void setCommandHandler(void (*handler)(char)); // callback khi nhận lệnh

    void sendMessage(const std::string &message);

    // Trong file ble_controller.hpp
    bool hasCommand();
    char getCommand();
    bool isConnected() const;

    bool hasJoystickData();
    float getJoystickX();
    float getJoystickY();

    void enableAutoConnect(bool enable = false);

private:
    BLEServer *pServer = nullptr;
    BLECharacteristic *pCharacteristic = nullptr;
    std::string rxValue;

    void (*commandHandler)(char) = nullptr;

    bool _hasNewCommand = false;
    char _lastCommand = 0;

    bool _hasNewJoystickData = false;
    float _joystickX = 0.0f;
    float _joystickY = 0.0f;

    bool _autoConnectEnabled = false;
    unsigned long _lastConnectionAttempt = 0;
    std::string _deviceName;

    class CommandCallback : public BLECharacteristicCallbacks
    {
    public:
        CommandCallback(BLEController *parent);
        void onWrite(BLECharacteristic *pCharacteristic) override;

    private:
        BLEController *parent;
    };

    class ServerCallbacks : public BLEServerCallbacks
    {
    public:
        ServerCallbacks(BLEController *parent) : parent(parent) {}

        void onConnect(BLEServer *pServer);

        void onDisconnect(BLEServer *pServer);

    private:
        BLEController *parent;
    };
};

#endif // BLE_CONTROLLER_HPP
