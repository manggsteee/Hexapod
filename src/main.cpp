#include <Arduino.h>
#include "hexapod.hpp"
#include "gait_controller.hpp"
#include "ble_controller.hpp"
#include "config.hpp"
#include "types.hpp"
#include "utils.hpp"

// Khai báo đối tượng chính
hexapod::Hexapod robot;
hexapod::GaitController gaitController;
hexapod::BLEController bleController;

// Các biến trạng thái và thời gian
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL_MS = 20; // 50Hz update rate

// Xử lý lệnh BLE và Serial
void handleBLECommand(char command) {
    Serial.print("Received command: ");
    Serial.println(command);
    
    switch (command) {
        case 'F': // Forward
            gaitController.setMode(hexapod::GaitController::Mode::WALK);
            gaitController.setDirection(hexapod::GaitController::Direction::FORWARD);
            break;
            
        case 'B': // Backward
            gaitController.setMode(hexapod::GaitController::Mode::WALK);
            gaitController.setDirection(hexapod::GaitController::Direction::BACKWARD);
            break;
            
        case 'L': // Left
            gaitController.setMode(hexapod::GaitController::Mode::WALK);
            gaitController.setDirection(hexapod::GaitController::Direction::LEFT);
            break;
            
        case 'R': // Right
            gaitController.setMode(hexapod::GaitController::Mode::WALK);
            gaitController.setDirection(hexapod::GaitController::Direction::RIGHT);
            break;
            
        case 'S': // Stop
            gaitController.setMode(hexapod::GaitController::Mode::STAND);
            break;
            
        case 'H': // Home
            robot.goHome();
            break;
            
        case 'T': // Tripod gait
            gaitController.setGaitType(hexapod::GaitController::GaitType::TRIPOD);
            break;
            
        case 'W': // Wave gait
            gaitController.setGaitType(hexapod::GaitController::GaitType::WAVE);
            break;
            
        default:
            // Không xử lý các lệnh khác
            break;
    }
}

void setup() {
    // Khởi tạo Serial với tốc độ cao để debug
    Serial.begin(115200);
    delay(500); // Đợi Serial ổn định
    
    Serial.println("\n\n=== Hexapod Robot Starting ===");
    
    // Khởi tạo servo controller (được gọi từ constructor của Hexapod)
    
    // Kết nối gait controller với hexapod
    gaitController.attachHexapod(&robot);
    
    // Khởi tạo BLE
    bleController.init("Hexapod_BLE");
    bleController.setCommandHandler(handleBLECommand);
    Serial.println("Bluetooth initialized and ready for connections");
    
    // Khởi tạo robot ở vị trí home
    Serial.println("Moving robot to home position...");
    robot.goHome();
    
    Serial.println("=== Initialization Complete ===");
    Serial.println("Ready for commands. Use BLE app or Serial monitor to control.");
    Serial.println("Commands: F=Forward, B=Backward, L=Left, R=Right");
    Serial.println("          S=Stop, H=Home, T=Tripod gait, W=Wave gait");
}

void loop() {
    // Xử lý BLE
    bleController.loop();
    
    // Cập nhật chuyển động theo chu kỳ
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
        float deltaTime = (currentTime - lastUpdateTime) / 1000.0f; // Chuyển sang giây
        lastUpdateTime = currentTime;
        
        // Xử lý joystick nếu có dữ liệu mới
        if (bleController.hasJoystickData()) {
            float joyX = bleController.getJoystickX();
            float joyY = bleController.getJoystickY();
            
            // Chỉ di chuyển khi joystick đủ xa vị trí trung tâm
            const float deadzone = 0.2f;
            if (abs(joyX) > deadzone || abs(joyY) > deadzone) {
                gaitController.setMode(hexapod::GaitController::Mode::WALK);
                
                // Xác định hướng di chuyển chính dựa trên joystick
                if (abs(joyX) > abs(joyY)) {
                    // Di chuyển sang trái/phải
                    if (joyX > 0) {
                        gaitController.setDirection(hexapod::GaitController::Direction::RIGHT);
                    } else {
                        gaitController.setDirection(hexapod::GaitController::Direction::LEFT);
                    }
                } else {
                    // Di chuyển tiến/lùi
                    if (joyY > 0) {
                        gaitController.setDirection(hexapod::GaitController::Direction::FORWARD);
                    } else {
                        gaitController.setDirection(hexapod::GaitController::Direction::BACKWARD);
                    }
                }
                
                // Điều chỉnh tốc độ dựa trên mức độ nghiêng joystick
                float magnitude = sqrt(joyX * joyX + joyY * joyY);
                magnitude = min(magnitude, 1.0f); // Giới hạn tối đa
                gaitController.setSpeed(magnitude);
            } else {
                // Dừng lại khi joystick ở vị trí trung tâm
                gaitController.setMode(hexapod::GaitController::Mode::STAND);
            }
        }
        
        // Cập nhật gait controller
        gaitController.update(deltaTime);
    }
    
    // Nhận lệnh từ Serial nếu có (để debug)
    if (Serial.available()) {
        char command = Serial.read();
        handleBLECommand(command);
    }
}
