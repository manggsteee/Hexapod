#include "hexapod.hpp"
#include "servo_controller.hpp"
#include "config.hpp"
#include <Arduino.h> // Cần thiết cho hàm delay()
#include "leg.hpp" // Thêm dòng này nếu ServoChannels được định nghĩa trong leg.hpp

namespace hexapod {

// Cấu trúc servo cho các chân
const ServoChannels LEG_SERVO_CHANNELS[6] = {
    {0, 2, 1, 0},  // Leg 0: PCA=0, Coxa=2, Femur=1, Tibia=0
    {0, 6, 5, 4},  // Leg 1: PCA=0, Coxa=6, Femur=5, Tibia=4
    {0, 10, 9, 8}, // Leg 2: PCA=0, Coxa=10, Femur=9, Tibia=8
    {1, 2, 1, 0},  // Leg 3: PCA=1, Coxa=2, Femur=1, Tibia=0
    {1, 6, 5, 4},  // Leg 4: PCA=1, Coxa=6, Femur=5, Tibia=4
    {1, 10, 9, 8}  // Leg 5: PCA=1, Coxa=10, Femur=9, Tibia=8
};

// Vị trí khởi tạo của các chân
const Point3D HOME_POSITIONS[6] = {
    {198.5f, 0.0f, -50.1f},  // Leg 0 (phải-trước)
    {198.5f, 0.0f, -50.1f},  // Leg 1 (phải-giữa) 
    {198.5f, 0.0f, -50.1f},  // Leg 2 (phải-sau)
    {198.5f, 0.0f, -50.1f},   // Leg 3 (trái-trước)
    {198.5f, 0.0f, -50.1f},   // Leg 4 (trái-giữa)
    {198.5f, 0.0f, -50.1f}    // Leg 5 (trái-sau)
};

Hexapod::Hexapod() {
    // Khởi tạo ServoController
    ServoController::initialize();
    
    // Khởi tạo các chân
    setupLegs();
}

// void Hexapod::goHome() {
//     Serial.println("Going to home position...");
    
//     // Góc home đã được điều chỉnh từ manual calibration
//     const float homeAngles[6][3] = {
//         {126.00f, 159.00f, 90.00f}, // Leg 0 - [Coxa, Femur, Tibia]
//         {99.00f, 163.00f, 79.00f},  // Leg 1
//         {85.00f, 159.00f, 80.00f},  // Leg 2
//         {80.00f, 69.00f, 97.00f},   // Leg 3
//         {100.00f, 19.00f, 78.00f},  // Leg 4
//         {96.00f, 34.00f, 106.00f}   // Leg 5
//     };
    
//     // Áp dụng góc home cho tất cả các chân
//     for (int leg = 0; leg < 6; leg++) {
//         int pca_id = LEG_SERVO_CHANNELS[leg][0];
        
//         // Coxa
//         int coxa_channel = LEG_SERVO_CHANNELS[leg][3]; // Channel 2
//         servo_controller::ServoController::setAngle(pca_id, coxa_channel, homeAngles[leg][0]);
//         delay(50);
        
//         // Femur  
//         int femur_channel = LEG_SERVO_CHANNELS[leg][2]; // Channel 1
//         servo_controller::ServoController::setAngle(pca_id, femur_channel, homeAngles[leg][1]);
//         delay(50);
        
//         // Tibia
//         int tibia_channel = LEG_SERVO_CHANNELS[leg][1]; // Channel 0
//         servo_controller::ServoController::setAngle(pca_id, tibia_channel, homeAngles[leg][2]);
//         delay(50);
        
//         Serial.print("Leg ");
//         Serial.print(leg);
//         Serial.println(" home position set");
//     }
    
//     Serial.println("Home position completed!");
// }

void Hexapod::setupLegs() {
    // --- Phần khởi tạo chân ---
    legs.reserve(config::NUM_LEGS);
    for (int i = 0; i < config::NUM_LEGS; ++i) {
        const auto& channels = hexapod::LEG_SERVO_CHANNELS[i];
        bool is_right = (i < 3); // Legs 0, 1, 2 are right legs; 3, 4, 5 are left
        legs.emplace_back(
            i,
            channels.pca_id,
            channels.coxa,
            channels.femur,
            channels.tibia,
            is_right
        );
    }

    // --- ĐƯA ROBOT VỀ VỊ TRÍ HOME KHI KHỞI ĐỘNG ---
    Serial.println("Moving to home position...");
    this->goHome();
    delay(1000); // Chờ 1 giây để các servo di chuyển xong
    Serial.println("Homing complete.");
}

bool Hexapod::setTargetPositions(const std::array<Point3D, config::NUM_LEGS>& positions) {
    bool all_reachable = true;
    for (size_t i = 0; i < legs.size(); ++i) {
        legs[i].setTargetPosition(positions[i]);
    }
    return all_reachable;
}

void Hexapod::update() {
    for (auto& leg : legs) {
        leg.update();
    }
}

void Hexapod::goHome() {
    Serial.println("Going to home position...");
    
    // Áp dụng vị trí home cho tất cả các chân
    for (int i = 0; i < config::NUM_LEGS; i++) {
        if (i < legs.size()) {
            const auto& home = HOME_POSITIONS[i];
            Serial.print("Setting home position for leg ");
            Serial.print(i);
            Serial.print(": x=");
            Serial.print(home.x);
            Serial.print(", y=");
            Serial.print(home.y);
            Serial.print(", z=");
            Serial.println(home.z);
            
            legs[i].setTargetPosition(home);
            legs[i].update();
            
            delay(100); // Thời gian để servo di chuyển
        }
    }
    
    Serial.println("Home position completed!");
}

Leg& Hexapod::getLeg(int leg_id) {
    if (leg_id < 0 || leg_id >= legs.size()) {
        // Nếu leg_id không hợp lệ, trả về chân đầu tiên
        return legs[0];
    }
    return legs[leg_id];
}

} // namespace hexapod
