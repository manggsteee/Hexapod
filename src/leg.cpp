#include "leg.hpp"
#include "servo_controller.hpp"
#include "utils.hpp"
#include <Arduino.h>

namespace hexapod {

// Sử dụng HOME_CORRECTION từ config

Leg::Leg(int leg_id, int pca_id, int coxa_channel, int femur_channel, int tibia_channel, bool is_right)
    : leg_id(leg_id),
      pca_id(pca_id),
      coxa_channel(coxa_channel),
      femur_channel(femur_channel),
      tibia_channel(tibia_channel),
      is_right(is_right),
      target_position{0, 0, 0},
      current_angles{90, 90, 90},
      servo_angles{90, 90, 90}
{
}

bool Leg::setTargetPosition(const Point3D& position, bool elbow_up) {
    // Lưu vị trí mục tiêu
    target_position = position;
    
    // Tính toán IK
    ServoAngles ikAngles;
    bool success = kinematics::computeIK(position, ikAngles, elbow_up);
    
    if (!success) {
        Serial.print("IK failed for leg ");
        Serial.println(leg_id);
        return false;
    }
    
    // Lưu góc IK hiện tại
    current_angles = ikAngles;
    
    // Chuyển đổi từ góc IK sang góc servo
    servo_angles = kinematics::ikToServoAngles(ikAngles, is_right);
    
    // Áp dụng hiệu chỉnh riêng cho mỗi chân
    servo_angles.coxa += config::HOME_CORRECTION[leg_id][0];
    servo_angles.femur += config::HOME_CORRECTION[leg_id][1];
    servo_angles.tibia += config::HOME_CORRECTION[leg_id][2];
    
    // Giới hạn trong khoảng hợp lệ
    servo_angles.coxa = utils::limit(servo_angles.coxa, 0.0f, 180.0f);
    servo_angles.femur = utils::limit(servo_angles.femur, 0.0f, 180.0f);
    servo_angles.tibia = utils::limit(servo_angles.tibia, 0.0f, 180.0f);
    
    Serial.print("Final angles - Leg ");
    Serial.print(leg_id);
    Serial.print(" - Coxa: ");
    Serial.print(servo_angles.coxa);
    Serial.print("°, Femur: ");
    Serial.print(servo_angles.femur);
    Serial.print("°, Tibia: ");
    Serial.print(servo_angles.tibia);
    Serial.println("°");
    
    return true;
}

void Leg::update() {
    // Gửi góc servo tới bộ điều khiển servo
    ServoController::setAngle(pca_id, coxa_channel, servo_angles.coxa);
    ServoController::setAngle(pca_id, femur_channel, servo_angles.femur);
    ServoController::setAngle(pca_id, tibia_channel, servo_angles.tibia);
}

ServoAngles Leg::getServoAngles() const {
    return servo_angles;
}

void Leg::setServoAngles(bool isSetHome, const ServoAngles& servoAngles) {
    // Đặt góc servo trực tiếp, bỏ qua IK
    servo_angles = servoAngles;
    
    // Giới hạn trong khoảng hợp lệ
    servo_angles.coxa = utils::limit(servo_angles.coxa, 0.0f, 180.0f);
    servo_angles.femur = utils::limit(servo_angles.femur, 0.0f, 180.0f);
    servo_angles.tibia = utils::limit(servo_angles.tibia, 0.0f, 180.0f);
    
    // Tính góc IK tương ứng nếu không thiết lập home
    if (!isSetHome) {
        current_angles = kinematics::servoToIkAngles(servo_angles, is_right);
        
        // Trừ đi hiệu chỉnh riêng cho mỗi chân
        current_angles.coxa -= config::HOME_CORRECTION[leg_id][0] / (is_right ? 1 : -1);
        current_angles.femur -= config::HOME_CORRECTION[leg_id][1] / (is_right ? 1 : -1);
        current_angles.tibia -= config::HOME_CORRECTION[leg_id][2] / (is_right ? 1 : -1);
    }
}

bool Leg::isIKValid() const {
    // Kiểm tra góc femur và tibia có trong khoảng hợp lệ không
    return (current_angles.femur >= -90.0f && current_angles.femur <= 180.0f) &&
           (current_angles.tibia >= -90.0f && current_angles.tibia <= 90.0f);
}

} // namespace hexapod