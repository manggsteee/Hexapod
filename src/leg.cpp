#include "leg.hpp"
#include "servo_controller.hpp"
#include <Arduino.h>
#include <cmath>
#include <cstdio>

// Các thông số vật lý của robot (đảm bảo khớp với robot của bạn)
constexpr float COXA_LENGTH = 65.0f;
constexpr float FEMUR_LENGTH = 65.0f;
constexpr float TIBIA_LENGTH = 95.0f;

// Các góc offset để robot đứng thẳng (bạn có thể tinh chỉnh sau)
constexpr float COXA_OFFSET = 90.0f;
constexpr float FEMUR_OFFSET = 90.0f;
constexpr float TIBIA_OFFSET = 90.0f;

float current_coxa = 0.0f;

const float HOME_CORRECTION[6][3] = {
    {36.0f, 69.0f, 0.0f},   // Leg 0: +36°, +69°, +0°
    {9.0f, 73.0f, -11.0f},  // Leg 1: +9°, +73°, -11°
    {-5.0f, 69.0f, -10.0f}, // Leg 2: -5°, +69°, -10°
    {-10.0f, -21.0f, 7.0f}, // Leg 3: -10°, -21°, +7°
    {10.0f, -71.0f, -12.0f},// Leg 4: +10°, -71°, -12°
    {6.0f, -56.0f, 16.0f}   // Leg 5: +6°, -56°, +16°
};


Leg::Leg(int leg_id, int pca_id, int coxa_channel, int femur_channel, int tibia_channel)
    : leg_id(leg_id),
      pca_id(pca_id),
      coxa_channel(coxa_channel),
      femur_channel(femur_channel),
      tibia_channel(tibia_channel),
      target_x(0), target_y(0), target_z(0),
      coxa_angle(0), femur_angle(0), tibia_angle(0)
{}

void Leg::setTargetPosition(float x, float y, float z) {
    target_x = x;
    target_y = y;
    target_z = z;
    computeIK();
}

void Leg::computeIK() {
    printf("IK Input - Leg %d: x=%.2f, y=%.2f, z=%.2f\n", leg_id, target_x, target_y, target_z);
    
    // === BƯỚC 1: COXA ANGLE ===
    coxa_angle = atan2(target_y, target_x) * 180.0f / M_PI;
    
    // === BƯỚC 2: TÍNH KHOẢNG CÁCH TRONG MẶT PHẲNG 2D ===
    float horizontal_dist = sqrt(target_x * target_x + target_y * target_y) - COXA_LENGTH;
    float vertical_dist = target_z;

    if (horizontal_dist < 5.0f) {
        horizontal_dist = 5.0f; // Chỉ giữ minimum distance nhỏ
    }

    float dist = sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);
    
    printf("Intermediate: horizontal_dist=%.2f, vertical_dist=%.2f, dist=%.2f\n", 
           horizontal_dist, vertical_dist, dist);
    
    // === BỎ HẾT KIỂM TRA TẦM VỚI ===
    // Không check max_reach, min_reach nữa
    
    // === TÍNH TOÁN IK TỰ DO ===
    float a1 = atan2(-vertical_dist, horizontal_dist);

    float cos_a2 = (FEMUR_LENGTH * FEMUR_LENGTH + dist * dist - TIBIA_LENGTH * TIBIA_LENGTH) /
                   (2 * FEMUR_LENGTH * dist);
    cos_a2 = std::max(-1.0f, std::min(1.0f, cos_a2)); // Chỉ clamp toán học

    float a2 = acos(cos_a2);
    femur_angle = (a1 + a2) * 180.0f / M_PI;
    
    float cos_tibia = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - dist * dist) /
                      (2 * FEMUR_LENGTH * TIBIA_LENGTH);
    cos_tibia = std::max(-1.0f, std::min(1.0f, cos_tibia)); // Chỉ clamp toán học

    float phi3 = acos(cos_tibia);
    tibia_angle = -(M_PI - phi3) * 180.0f / M_PI;
    
    printf("Final angles - Coxa: %.2f°, Femur: %.2f°, Tibia: %.2f°\n", 
           coxa_angle, femur_angle, tibia_angle);
    
    // === BỎ HẾT WARNING ANGLE CHECKS ===
    // Không warning nữa, để IK tự do
}

void Leg::update() {
    // --- CẢI TIẾN LOGIC CHIỀU QUAY SERVO ---
    // Mặc định cho các chân bên phải (0, 1, 2)
    int coxa_dir = 1;
    int femur_dir = 1;
    int tibia_dir = -1; // Tibia hầu như luôn quay ngược

    // Các chân bên trái (3, 4, 5) thường có chiều quay ngược lại
    if (leg_id >= 3) {
        coxa_dir = -1;
        femur_dir = -1;
        // tibia_dir giữ nguyên
    }
    // Gợi ý: Nếu chân 3 vẫn đi sai, hãy thử đổi dấu của coxa_dir hoặc femur_dir ở đây.

    // Áp dụng offset và chiều quay
    float coxa_pwm = COXA_OFFSET + HOME_CORRECTION[leg_id][0] + (coxa_dir * coxa_angle);
    float femur_pwm = FEMUR_OFFSET + HOME_CORRECTION[leg_id][1] + (femur_dir * femur_angle);
    float tibia_pwm = TIBIA_OFFSET + HOME_CORRECTION[leg_id][2] + (tibia_dir * tibia_angle);
static float current_coxa_angles[6] = {COXA_OFFSET, COXA_OFFSET, COXA_OFFSET, 
                                         COXA_OFFSET, COXA_OFFSET, COXA_OFFSET};
    
    // Xác định bước di chuyển (có thể điều chỉnh số này để thay đổi tốc độ)
    float step_size = 1.0f; // Số độ mỗi lần di chuyển
    
    // Tính góc mới dựa trên bước di chuyển
    if (abs(current_coxa_angles[leg_id] - coxa_pwm) > step_size) {
        // Nếu góc mới lớn hơn góc hiện tại
        if (current_coxa_angles[leg_id] < coxa_pwm) {
            current_coxa_angles[leg_id] += step_size;
        } 
        // Nếu góc mới nhỏ hơn góc hiện tại
        else {
            current_coxa_angles[leg_id] -= step_size;
        }
    } else {
        // Nếu sự khác biệt nhỏ hơn bước di chuyển, đặt đúng vị trí
        current_coxa_angles[leg_id] = coxa_pwm;
    }
    
    // Áp dụng góc coxa mới đã được làm chậm
    ServoController::setAngle(pca_id, coxa_channel, current_coxa_angles[leg_id]);
    // Giới hạn góc trong khoảng an toàn
    femur_pwm = std::max(0.0f, std::min(180.0f, femur_pwm));
    tibia_pwm = std::max(0.0f, std::min(180.0f, tibia_pwm));
    
    // int counter = 10;
    // Serial.println("Current Coxa angle: " + String(current_coxa));
    // Serial.println("Target Coxa angle: " + String(coxa_pwm));
    // if (coxa_pwm < current_coxa){
    //     counter = -counter; // Chỉ cần tăng góc coxa
    // }
    // else{
    //     counter = counter; // Chỉ cần giảm góc coxa
    // }
    // // Gửi lệnh đến servo controller
    // for (int i = int(current_coxa); i <= int(coxa_pwm); i += counter) {
    //     Serial.println("Setting Coxa angle: " + String(i));
    //     ServoController::setAngle(pca_id, coxa_channel, i);
    //     delay(100);
    // }
    // current_coxa = coxa_pwm;
    ServoController::setAngle(pca_id, coxa_channel, coxa_pwm);
    ServoController::setAngle(pca_id, femur_channel, femur_pwm);
    ServoController::setAngle(pca_id, tibia_channel, tibia_pwm);
}

std::array<float, 3> Leg::getServoAngles() const {
    return {coxa_angle, femur_angle, tibia_angle};
}
