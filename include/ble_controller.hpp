#ifndef BLE_CONTROLLER_HPP
#define BLE_CONTROLLER_HPP

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class BLEController {
public:
    void init(const std::string& deviceName);
    void loop();  // xử lý dữ liệu từ BLE

    void setCommandHandler(void (*handler)(char)); // callback khi nhận lệnh

    void sendMessage(const std::string& message);

private:
    BLEServer* pServer = nullptr;
    BLECharacteristic* pCharacteristic = nullptr;
    std::string rxValue;

    void (*commandHandler)(char) = nullptr;

    class CommandCallback : public BLECharacteristicCallbacks {
    public:
        CommandCallback(BLEController* parent);
        void onWrite(BLECharacteristic* pCharacteristic) override;

    private:
        BLEController* parent;
    };
};

#endif // BLE_CONTROLLER_HPP
