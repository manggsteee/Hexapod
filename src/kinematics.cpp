#include "kinematics.hpp"
#include <cmath>
#include <Arduino.h>

void Kinematics::computeFK(int legId, float coxaAngle, float femurAngle, float tibiaAngle, float& x, float& y, float& z)
{
    // Thông số vật lý của robot
    const float COXA_LENGTH = 65.0f;
    const float FEMUR_LENGTH = 65.0f;
    const float TIBIA_LENGTH = 95.0f;
    // Các offset chuẩn
    const float COXA_OFFSET = 90.0f;
    const float FEMUR_OFFSET = 90.0f;
    const float TIBIA_OFFSET = 90.0f;
    const float HOME_CORRECTION[6][3] = {
        {0.0f, 8.0f, -2.0f},
        {-2.0f, 3.0f, 3.0f},
        {-5.0f, -4.0f, 12.0f},
        {8.0f, -10.0f, 7.0f},
        {-6.0f, 3.0f, -10.0f},
        {-0.02f, 5.22f, -5.20f}
    };
    int coxa_dir = (legId < 3) ? 1 : -1;
    int femur_dir = (legId < 3) ? 1 : -1;
    int tibia_dir = (legId < 3) ? 1 : -1;
    float coxa_correction = HOME_CORRECTION[legId][0];
    float femur_correction = HOME_CORRECTION[legId][1];
    float tibia_correction = HOME_CORRECTION[legId][2];
    float coxa_angle = (coxaAngle - COXA_OFFSET - coxa_correction) / coxa_dir;
    float femur_angle = (femurAngle - FEMUR_OFFSET - femur_correction) / femur_dir;
    float tibia_angle = (tibiaAngle - TIBIA_OFFSET - tibia_correction) / tibia_dir;
    float coxa_rad = coxa_angle * M_PI / 180.0f;
    float femur_rad = femur_angle * M_PI / 180.0f;
    float coxa_x = COXA_LENGTH * cos(coxa_rad);
    float coxa_y = COXA_LENGTH * sin(coxa_rad);
    float femur_proj = FEMUR_LENGTH * cos(femur_rad);
    float femur_height = FEMUR_LENGTH * sin(femur_rad);
    float tibia_angle_total = femur_angle + tibia_angle;
    float tibia_rad_total = tibia_angle_total * M_PI / 180.0f;
    float tibia_proj = TIBIA_LENGTH * cos(tibia_rad_total);
    float tibia_height = TIBIA_LENGTH * sin(tibia_rad_total);
    x = coxa_x + femur_proj * cos(coxa_rad) + tibia_proj * cos(coxa_rad);
    y = coxa_y + femur_proj * sin(coxa_rad) + tibia_proj * sin(coxa_rad);
    z = -(femur_height + tibia_height);
}

void Kinematics::computeFKArray(int legId, float coxaAngle, float femurAngle, float tibiaAngle)
{
    float x, y, z;
    computeFK(legId, coxaAngle, femurAngle, tibiaAngle, x, y, z);
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

bool Kinematics::computeIK(int leg_id, float x, float y, float z, float& coxa_angle, float& femur_angle, float& tibia_angle, bool elbow_up)
{
    constexpr float COXA_LENGTH = 65.0f;
    constexpr float FEMUR_LENGTH = 65.0f;
    constexpr float TIBIA_LENGTH = 95.0f;
    if (isnan(x) || isnan(y) || isnan(z)) return false;
    coxa_angle = atan2(y, x) * (180.0f / M_PI);
    float horizontal_dist = sqrt(x * x + y * y) - COXA_LENGTH;
    float vertical_dist = z;
    float dist = sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);
    float max_reach = FEMUR_LENGTH + TIBIA_LENGTH;
    if (dist > max_reach) return false;
    float a1 = atan2(-vertical_dist, horizontal_dist);
    float cos_a2 = (FEMUR_LENGTH * FEMUR_LENGTH + dist * dist - TIBIA_LENGTH * TIBIA_LENGTH) / (2 * FEMUR_LENGTH * dist);
    if (cos_a2 < -1.0f || cos_a2 > 1.0f) return false;
    cos_a2 = std::max(-1.0f, std::min(1.0f, cos_a2));
    float a2 = elbow_up ? -acos(cos_a2) : acos(cos_a2);
    femur_angle = (a1 + a2) * 180.0f / M_PI;
    float cos_tibia = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - dist * dist) / (2 * FEMUR_LENGTH * TIBIA_LENGTH);
    cos_tibia = std::max(-1.0f, std::min(1.0f, cos_tibia));
    float tibia_angle_rad = acos(cos_tibia);
    tibia_angle = (elbow_up ? -1 : 1) * tibia_angle_rad * 180.0f / M_PI;
    return true;
}
