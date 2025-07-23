#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <functional>
#include <string>

namespace hexapod {

/**
 * BLE controller for remote control of the hexapod
 */
class BLEController {
public:
    /**
     * Initialize BLE server with a device name
     * 
     * @param deviceName Name to advertise the device as
     */
    void init(const std::string &deviceName);
    
    /**
     * Main loop function, process BLE data
     * Should be called regularly in the main loop
     */
    void loop();
    
    /**
     * Set command handler callback
     * 
     * @param handler Function to call when a command is received
     */
    void setCommandHandler(std::function<void(char)> handler);
    
    /**
     * Send a message to connected clients
     * 
     * @param message Message to send
     */
    void sendMessage(const std::string &message);
    
    /**
     * Check if a new command is available
     * 
     * @return True if a new command is available
     */
    bool hasCommand();
    
    /**
     * Get the last received command
     * 
     * @return Last command character
     */
    char getCommand();
    
    /**
     * Check if a BLE client is connected
     * 
     * @return True if connected
     */
    bool isConnected() const;
    
    /**
     * Check if new joystick data is available
     * 
     * @return True if new joystick data is available
     */
    bool hasJoystickData();
    
    /**
     * Get joystick X position (-1.0 to 1.0)
     * 
     * @return X position
     */
    float getJoystickX();
    
    /**
     * Get joystick Y position (-1.0 to 1.0)
     * 
     * @return Y position
     */
    float getJoystickY();
    
    /**
     * Enable/disable auto-reconnection
     * 
     * @param enable Whether to enable auto-reconnection
     */
    void enableAutoConnect(bool enable = true);

private:
    BLEServer *pServer = nullptr;
    BLECharacteristic *pCharacteristic = nullptr;
    std::string rxValue;
    
    std::function<void(char)> commandHandler = nullptr;
    
    bool _hasNewCommand = false;
    char _lastCommand = 0;
    
    bool _hasNewJoystickData = false;
    float _joystickX = 0.0f;
    float _joystickY = 0.0f;
    
    bool _autoConnectEnabled = false;
    unsigned long _lastConnectionAttempt = 0;
    std::string _deviceName;
    
    // BLE callbacks
    class CommandCallback : public BLECharacteristicCallbacks {
    public:
        CommandCallback(BLEController *parent);
        void onWrite(BLECharacteristic *pCharacteristic) override;
        
    private:
        BLEController *parent;
    };
    
    class ServerCallbacks : public BLEServerCallbacks {
    public:
        ServerCallbacks(BLEController *parent) : parent(parent) {}
        
        void onConnect(BLEServer *pServer) override;
        void onDisconnect(BLEServer *pServer) override;
        
    private:
        BLEController *parent;
    };
};

} // namespace hexapod
