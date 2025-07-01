#include "hexapod.hpp"
#include "servo_controller.hpp"
#include <Arduino.h> // Cần thiết cho hàm delay()

// Cấu trúc LEG_SERVO_CHANNELS giữ nguyên như bạn đã định nghĩa
const int LEG_SERVO_CHANNELS[Kinematics::NUM_LEGS][4] = {
    // --- Các chân trên Mạch #1 (PCA_ID = 0) ---
    {0, 0, 1, 2},    // Chân 0 (phải-trước)
    {0, 4, 5, 6},    // Chân 1 (phải-giữa)
    {0, 8, 9, 10},    // Chân 2 (phải-sau)

    // --- Các chân trên Mạch #2 (PCA_ID = 1) ---
    {1, 0, 1, 2},    // Chân 0 (phải-trước)
    {1, 4, 5, 6},    // Chân 1 (phải-giữa)
    {1, 8, 9, 10},    // Chân 2 (phải-sau)

};

Hexapod::Hexapod() {}

void Hexapod::goHome() {
    Serial.println("Going to home position...");
    
    // Góc home đã được điều chỉnh từ manual calibration
    const float homeAngles[6][3] = {
        {126.00f, 159.00f, 90.00f}, // Leg 0 - [Coxa, Femur, Tibia]
        {99.00f, 163.00f, 79.00f},  // Leg 1
        {85.00f, 159.00f, 80.00f},  // Leg 2
        {80.00f, 69.00f, 97.00f},   // Leg 3
        {100.00f, 19.00f, 78.00f},  // Leg 4
        {96.00f, 34.00f, 106.00f}   // Leg 5
    };
    
    // Áp dụng góc home cho tất cả các chân
    for (int leg = 0; leg < 6; leg++) {
        int pca_id = LEG_SERVO_CHANNELS[leg][0];
        
        // Coxa
        int coxa_channel = LEG_SERVO_CHANNELS[leg][3]; // Channel 2
        ServoController::setAngle(pca_id, coxa_channel, homeAngles[leg][0]);
        delay(50);
        
        // Femur  
        int femur_channel = LEG_SERVO_CHANNELS[leg][2]; // Channel 1
        ServoController::setAngle(pca_id, femur_channel, homeAngles[leg][1]);
        delay(50);
        
        // Tibia
        int tibia_channel = LEG_SERVO_CHANNELS[leg][1]; // Channel 0
        ServoController::setAngle(pca_id, tibia_channel, homeAngles[leg][2]);
        delay(50);
        
        Serial.print("Leg ");
        Serial.print(leg);
        Serial.println(" home position set");
    }
    
    Serial.println("Home position completed!");
}

void Hexapod::setupLegs() {
    // --- Phần khởi tạo chân giữ nguyên ---
    legs.reserve(Kinematics::NUM_LEGS);
    for (int i = 0; i < Kinematics::NUM_LEGS; ++i) {
        int pca_id     = LEG_SERVO_CHANNELS[i][0];
        int tibia_ch = LEG_SERVO_CHANNELS[i][1];  // Channel 0 = Tibia
        int femur_ch = LEG_SERVO_CHANNELS[i][2];  // Channel 1 = Femur  
        int coxa_ch = LEG_SERVO_CHANNELS[i][3];   // Channel 2 = Coxa

        legs.emplace_back(
            i,
            pca_id,
            coxa_ch,
            femur_ch,
            tibia_ch
        );
    }

    // --- PHẦN THÊM MỚI: ĐƯA ROBOT VỀ VỊ TRÍ HOME KHI KHỞI ĐỘNG ---
    Serial.println("Moving to home position...");
    this->goHome();
    delay(1000); // Chờ 1 giây để các servo di chuyển xong
    Serial.println("Homing complete.");
}

void Hexapod::setTargetPositions(const std::array<Vec3, Kinematics::NUM_LEGS>& positions) {
    for (size_t i = 0; i < legs.size(); ++i) {
        legs[i].setTargetPosition(positions[i].x, positions[i].y, positions[i].z);
    }
}

void Hexapod::update() {
    for (auto& leg : legs) {
        leg.update();
    }
}
