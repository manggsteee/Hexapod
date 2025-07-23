#include "kinematics.hpp"
#include "utils.hpp"
#include <cmath>
#include <Arduino.h>

namespace hexapod {
namespace kinematics {

/**
 * Compute forward kinematics from servo angles for a specific leg
 * Used for debug purposes
 */
void computeLegFK(int legId, float coxaAngle, float femurAngle, float tibiaAngle, 
              float& x, float& y, float& z) 
{
    // Xác định hướng servo dựa vào ID chân
    int coxa_dir = (legId < 3) ? 1 : -1;
    int femur_dir = (legId < 3) ? 1 : -1;
    int tibia_dir = (legId < 3) ? 1 : -1;  // Hướng tibia luôn ngược

    // Sử dụng HOME_CORRECTION từ config
    float coxa_correction = config::HOME_CORRECTION[legId][0];
    float femur_correction = config::HOME_CORRECTION[legId][1];
    float tibia_correction = config::HOME_CORRECTION[legId][2];

    float coxa_angle = (coxaAngle - config::COXA_OFFSET - coxa_correction) / coxa_dir;
    float femur_angle = (femurAngle - config::FEMUR_OFFSET - femur_correction) / femur_dir;
    float tibia_angle = (tibiaAngle - config::TIBIA_OFFSET - tibia_correction) / tibia_dir;

    // Chuyển đổi sang radians
    float coxa_rad = utils::degToRad(coxa_angle);
    float femur_rad = utils::degToRad(femur_angle);
    
    // Tính điểm cuối của coxa
    float coxa_x = config::COXA_LENGTH * cos(coxa_rad);
    float coxa_y = config::COXA_LENGTH * sin(coxa_rad);
    
    // Tính chiều dài phần femur trên mặt phẳng ngang và chiều cao
    float femur_proj = config::FEMUR_LENGTH * cos(femur_rad);
    float femur_height = config::FEMUR_LENGTH * sin(femur_rad);
    
    // Tính góc tổng của tibia so với mặt phẳng ngang
    float tibia_angle_total = femur_angle + tibia_angle;
    float tibia_rad_total = utils::degToRad(tibia_angle_total);
    
    // Tính chiều dài phần tibia trên mặt phẳng ngang và chiều cao
    float tibia_proj = config::TIBIA_LENGTH * cos(tibia_rad_total);
    float tibia_height = config::TIBIA_LENGTH * sin(tibia_rad_total);
    
    // Tính tọa độ cuối cùng của đầu chân
    x = coxa_x + femur_proj * cos(coxa_rad) + tibia_proj * cos(coxa_rad);
    y = coxa_y + femur_proj * sin(coxa_rad) + tibia_proj * sin(coxa_rad);
    z = -(femur_height + tibia_height); // Đảo dấu z vì trục z hướng xuống trong hệ tọa độ robot
}

/**
 * Compute forward kinematics and print the result
 * Used for debug purposes
 */
void computeLegFKArray(int legId, float coxaAngle, float femurAngle, float tibiaAngle) 
{
    float x, y, z;
    computeLegFK(legId, coxaAngle, femurAngle, tibiaAngle, x, y, z);
    Serial.print("FK for Leg ");
    Serial.print(legId);
    Serial.print(": (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(")");
}

ServoAngles ikToServoAngles(const ServoAngles& angles, bool isRight) {
    ServoAngles servoAngles;
    
    // Hệ số hướng dựa trên chân trái hay phải
    int coxa_dir = isRight ? 1 : -1;
    int femur_dir = isRight ? 1 : -1;
    int tibia_dir = isRight ? 1 : -1;
    
    // Áp dụng offset chung
    servoAngles.coxa = config::COXA_OFFSET + (coxa_dir * angles.coxa);
    servoAngles.femur = config::FEMUR_OFFSET + (femur_dir * angles.femur);
    servoAngles.tibia = config::TIBIA_OFFSET + (tibia_dir * angles.tibia);
    
    // Giới hạn trong khoảng hợp lệ
    servoAngles.coxa = utils::limit(servoAngles.coxa, 0.0f, 180.0f);
    servoAngles.femur = utils::limit(servoAngles.femur, 0.0f, 180.0f);
    servoAngles.tibia = utils::limit(servoAngles.tibia, 0.0f, 180.0f);
    
    return servoAngles;
}

ServoAngles servoToIkAngles(const ServoAngles& servoAngles, bool isRight) {
    ServoAngles ikAngles;
    
    // Hệ số hướng dựa trên chân trái hay phải
    int coxa_dir = isRight ? 1 : -1;
    int femur_dir = isRight ? 1 : -1;
    int tibia_dir = isRight ? 1 : -1;
    
    // Chuyển đổi từ góc servo sang góc IK
    ikAngles.coxa = (servoAngles.coxa - config::COXA_OFFSET) / coxa_dir;
    ikAngles.femur = (servoAngles.femur - config::FEMUR_OFFSET) / femur_dir;
    ikAngles.tibia = (servoAngles.tibia - config::TIBIA_OFFSET) / tibia_dir;
    
    return ikAngles;
}

bool computeIK(const Point3D& target, ServoAngles& result, bool elbow_up) {
    // Kiểm tra đầu vào
    if (isnan(target.x) || isnan(target.y) || isnan(target.z)) {
        Serial.println("IK Error: Invalid input coordinates");
        return false;
    }
    
    // Bước 1: Tính góc coxa dựa trên vị trí xy
    result.coxa = atan2(target.y, target.x) * (180.0f / M_PI);
    
    // Bước 2: Tính toán khoảng cách trong mặt phẳng 2D
    float horizontal_dist = sqrt(target.x * target.x + target.y * target.y) - config::COXA_LENGTH;
    float vertical_dist = target.z;
    float dist = sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);
    
    // Kiểm tra điều kiện tồn tại nghiệm
    float max_reach = config::FEMUR_LENGTH + config::TIBIA_LENGTH;
    if (dist > max_reach) {
        dist = max_reach - 1.0f; // Giới hạn khoảng cách tối đa
        Serial.println("IK Warning: Target out of reach, adjusting distance");
    }
    
    // Bước 3: Tính toán IK với phương pháp hình học
    float a1 = atan2(-vertical_dist, horizontal_dist);
    
    float cos_a2 = (config::FEMUR_LENGTH * config::FEMUR_LENGTH + dist * dist - config::TIBIA_LENGTH * config::TIBIA_LENGTH) /
                  (2 * config::FEMUR_LENGTH * dist);
    
    // Kiểm tra giới hạn
    if (cos_a2 < -1.0f || cos_a2 > 1.0f) {
        Serial.print("IK Warning: cos_a2 out of range: ");
        Serial.println(cos_a2);
        cos_a2 = utils::limit(cos_a2, -1.0f, 1.0f);
    }
    
    float a2;
    if (elbow_up) {
        a2 = -acos(cos_a2); // Góc âm cho elbow up
    } else {
        a2 = acos(cos_a2);  // Góc dương cho elbow down
    }
    
    result.femur = (a1 + a2) * 180.0f / M_PI;
    
    // Tính góc tibia
    float cos_tibia = (config::FEMUR_LENGTH * config::FEMUR_LENGTH + config::TIBIA_LENGTH * config::TIBIA_LENGTH - dist * dist) /
                     (2 * config::FEMUR_LENGTH * config::TIBIA_LENGTH);
    
    cos_tibia = utils::limit(cos_tibia, -1.0f, 1.0f);
    float tibia_angle_rad = acos(cos_tibia);
    
    // Điều chỉnh góc tibia dựa trên cấu hình elbow
    if (a2 < 0) { // elbow up
        result.tibia = (180.0f - tibia_angle_rad * 180.0f / M_PI);
    } else { // elbow down
        result.tibia = -(180.0f - tibia_angle_rad * 180.0f / M_PI);
    }
    
    // Giới hạn góc tibia
    result.tibia = utils::limit(result.tibia, -90.0f, 90.0f);
    
    // Kiểm tra kết quả
    if (isnan(result.femur) || isnan(result.tibia)) {
        Serial.println("IK Error: NaN result in calculation");
        return false;
    }
    
    return true;
}

Point3D computeFK(const ServoAngles& angles) {
    Point3D result;
    
    // Chuyển đổi góc từ độ sang radian
    float coxa_rad = utils::degToRad(angles.coxa);
    float femur_rad = utils::degToRad(angles.femur);
    float tibia_rad = utils::degToRad(angles.tibia);
    
    // Tính vị trí coxa
    float coxa_x = config::COXA_LENGTH * cos(coxa_rad);
    float coxa_y = config::COXA_LENGTH * sin(coxa_rad);
    float coxa_z = 0;
    
    // Tính vị trí femur
    float femur_x = config::FEMUR_LENGTH * cos(femur_rad) * cos(coxa_rad);
    float femur_y = config::FEMUR_LENGTH * cos(femur_rad) * sin(coxa_rad);
    float femur_z = config::FEMUR_LENGTH * sin(femur_rad);
    
    // Tính vị trí tibia (end effector)
    float tibia_angle_rad = tibia_rad + femur_rad;
    float tibia_x = config::TIBIA_LENGTH * cos(tibia_angle_rad) * cos(coxa_rad);
    float tibia_y = config::TIBIA_LENGTH * cos(tibia_angle_rad) * sin(coxa_rad);
    float tibia_z = config::TIBIA_LENGTH * sin(tibia_angle_rad);
    
    // Tổng hợp kết quả
    result.x = coxa_x + femur_x + tibia_x;
    result.y = coxa_y + femur_y + tibia_y;
    result.z = coxa_z + femur_z + tibia_z;
    
    return result;
}

} // namespace kinematics
} // namespace hexapod
