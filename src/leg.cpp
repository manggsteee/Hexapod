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
    {0.0f, 8.0f, -2.0f},     // Leg 0: 0°, +8°, -2°
    {-2.0f, 3.0f, 3.0f},   // Leg 1: -2°, +3°, +3°
    {-5.0f, -4.0f, 12.0f},   // Leg 2: -5°, -4°, +12°
    {8.0f, -10.0f, 7.0f},   // Leg 3: 8°, -10°, +7°
    {-6.0f, 3.0f, -10.0f},   // Leg 4: -6°, +3°, -10°
    {-0.02f, 5.22f, -5.20f}    // Leg 5: -0.02°, +5.22°, -5.20°
};

Leg::Leg(int leg_id, int pca_id, int coxa_channel, int femur_channel, int tibia_channel)
    : leg_id(leg_id),
      pca_id(pca_id),
      coxa_channel(coxa_channel),
      femur_channel(femur_channel),
      tibia_channel(tibia_channel),
      target_x(0), target_y(0), target_z(0),
      coxa_angle(0), femur_angle(0), tibia_angle(0)
{
}

void Leg::setTargetPosition(float x, float y, float z, bool elbow_up)
{
    // Lưu vị trí mục tiêu gốc
    target_x = x;
    target_y = y;
    target_z = z;

    // Tính toán IK với các giá trị target_x, target_y, target_z
    computeIK(elbow_up);

    // Chuyển đổi từ góc IK sang góc servo
    int coxa_dir = (leg_id < 3) ? 1 : -1;
    int femur_dir = (leg_id < 3) ? 1 : -1;
    int tibia_dir = (leg_id < 3) ? 1 : -1;

    // Áp dụng offset và home correction
    coxa_servo_angle = COXA_OFFSET + HOME_CORRECTION[leg_id][0] + (coxa_dir * coxa_angle);
    femur_servo_angle = FEMUR_OFFSET + HOME_CORRECTION[leg_id][1] + (femur_dir * femur_angle);
    tibia_servo_angle = TIBIA_OFFSET + HOME_CORRECTION[leg_id][2] + (tibia_dir * tibia_angle);

    // Giới hạn trong khoảng hợp lệ
    coxa_servo_angle = std::max(0.0f, std::min(180.0f, coxa_servo_angle));
    femur_servo_angle = std::max(0.0f, std::min(180.0f, femur_servo_angle));
    tibia_servo_angle = std::max(0.0f, std::min(180.0f, tibia_servo_angle));

    Serial.print("Final angles - Coxa: ");
    Serial.print(coxa_servo_angle);
    Serial.print("°, Femur: ");
    Serial.print(femur_servo_angle);
    Serial.print("°, Tibia: ");
    Serial.print(tibia_servo_angle);
    Serial.println("°");
}

void Leg::computeIK(bool elbow_up)
{
    Serial.print("IK Input - Leg ");
    Serial.print(leg_id);
    Serial.print(": x=");
    Serial.print(target_x);
    Serial.print(", y=");
    Serial.print(target_y);
    Serial.print(", z=");
    Serial.println(target_z);

    // === KIỂM TRA INPUT HỢP LỆ ===
    if (isnan(target_x) || isnan(target_y) || isnan(target_z))
    {
        Serial.println("ERROR: Invalid input coordinates");
        return;
    }

    // === BƯỚC 1: COXA ANGLE ===
    coxa_angle = atan2(target_y, target_x) * (180.0f / M_PI);

    // === BƯỚC 2: TÍNH KHOẢNG CÁCH TRONG MẶT PHẲNG 2D ===
    float horizontal_dist = sqrt(target_x * target_x + target_y * target_y) - COXA_LENGTH;
    float vertical_dist = target_z;
    float dist = sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);

    Serial.print("Intermediate: horizontal_dist=");
    Serial.print(horizontal_dist);
    Serial.print(", vertical_dist=");
    Serial.print(vertical_dist);
    Serial.print(", dist=");
    Serial.println(dist);

    // === KIỂM TRA ĐIỀU KIỆN TỒN TẠI NGHIỆM ===
    float max_reach = FEMUR_LENGTH + TIBIA_LENGTH;
    if (dist > max_reach)
    {
        dist = max_reach - 1.0f; // Giới hạn khoảng cách tối đa
        Serial.println("WARNING: Target out of reach, adjusting distance");
    }
    

    // === TÍNH TOÁN IK ===
    float a1 = atan2(-vertical_dist, horizontal_dist);

    float cos_a2 = (FEMUR_LENGTH * FEMUR_LENGTH + dist * dist - TIBIA_LENGTH * TIBIA_LENGTH) /
                   (2 * FEMUR_LENGTH * dist);

    // Kiểm tra validity trước khi clamp
    if (cos_a2 < -1.00f || cos_a2 > 1.00f)
    {
        Serial.print("WARNING: cos_a2 out of range: ");
        Serial.println(cos_a2);
    }
    cos_a2 = std::max(-1.0f, std::min(1.0f, cos_a2));

    bool effective_elbow_up;
    if (leg_id < 3) { // Chân phải (0, 1, 2)
        effective_elbow_up = !elbow_up; // Đảo ngược elbow_up
    } else { // Chân trái (3, 4, 5)
        effective_elbow_up = elbow_up;
    }

    float a2;
    if (elbow_up) {
        a2 = -acos(cos_a2); // Luôn chọn góc âm cho elbow up
    } else {
        a2 = acos(cos_a2);  // Luôn chọn góc dương cho elbow down
    }
    femur_angle = (a1 + a2) * 180.0f / M_PI;

    float cos_tibia = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - dist * dist) /
                      (2 * FEMUR_LENGTH * TIBIA_LENGTH);

    
    cos_tibia = std::max(-1.0f, std::min(1.0f, cos_tibia));
    float tibia_angle_rad = acos(cos_tibia);
    // Thay đổi phương pháp tính tibia_angle để phù hợp với elbow up
    if (a2 < 0) { // elbow up
        tibia_angle = (180.0f - tibia_angle_rad * 180.0f / M_PI); // Giá trị dương, góc hẹp
    } else { // elbow down
        tibia_angle = -(180.0f - tibia_angle_rad * 180.0f / M_PI); // Giá trị âm, góc rộng
    }
    
    
    // Hiệu chỉnh cuối cùng nếu tibia_angle vẫn là 0
    if (abs(tibia_angle) < 0.01f) {
        // Tibia đối nghịch với femur để giữ chân thẳng
        tibia_angle = -femur_angle;
        Serial.print("Corrected tibia angle: ");
        Serial.println(tibia_angle);
    }
    
    // Giới hạn góc tibia trong phạm vi hợp lý
    tibia_angle = std::max(-90.0f, std::min(90.0f, tibia_angle));

    // === KIỂM TRA KẾT QUẢ HỢP LỆ ===
    if (isnan(femur_angle) || isnan(tibia_angle))
    {
        Serial.println("ERROR: NaN result in IK calculation");
        femur_angle = 0.0f;
        tibia_angle = 0.0f;
    }
}

void Leg::update()
{
    // Chỉ cần áp dụng góc servo đã tính
    ServoController::setAngle(pca_id, coxa_channel, coxa_servo_angle);
    ServoController::setAngle(pca_id, femur_channel, femur_servo_angle);
    ServoController::setAngle(pca_id, tibia_channel, tibia_servo_angle);
}

std::array<float, 3> Leg::getServoAngles() const
{
    return {coxa_servo_angle, femur_servo_angle, tibia_servo_angle};
}

void Leg::setServoAngles(bool isSetHome, float coxa, float femur, float tibia)
{
    // Đặt góc servo trực tiếp, bỏ qua IK
    coxa_servo_angle = coxa;
    femur_servo_angle = femur;
    tibia_servo_angle = tibia;

    // Giới hạn trong khoảng hợp lệ
    coxa_servo_angle = std::max(0.0f, std::min(180.0f, coxa_servo_angle));
    femur_servo_angle = std::max(0.0f, std::min(180.0f, femur_servo_angle));
    tibia_servo_angle = std::max(0.0f, std::min(180.0f, tibia_servo_angle));

    // Chuyển đổi ngược từ góc servo sang góc IK (nếu cần)
    int coxa_dir = (leg_id < 3) ? 1 : -1;
    int femur_dir = (leg_id < 3) ? 1 : -1;
    int tibia_dir = -1;

    coxa_angle = (coxa_servo_angle - COXA_OFFSET - (isSetHome ? HOME_CORRECTION[leg_id][0] : 0)) / coxa_dir;
    femur_angle = (femur_servo_angle - FEMUR_OFFSET - (isSetHome ? HOME_CORRECTION[leg_id][1] : 0)) / femur_dir;
    tibia_angle = (tibia_servo_angle - TIBIA_OFFSET - (isSetHome ? HOME_CORRECTION[leg_id][2] : 0)) / tibia_dir;

}