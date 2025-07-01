#pragma once

#include <array>

class Leg {
public:
    // Cập nhật hàm khởi tạo để nhận thêm pca_id (nếu bạn vẫn dùng cấu trúc này)
    Leg(int leg_id, int pca_id, int coxa_channel, int femur_channel, int tibia_channel);

    // XÓA BỎ "delete", THAY BẰNG "default" ĐỂ CHO PHÉP SAO CHÉP
    Leg() = default;
    Leg(const Leg&) = default;
    Leg& operator=(const Leg&) = default;

    // Move constructor/assignment vẫn giữ nguyên
    Leg(Leg&&) = default;
    Leg& operator=(Leg&&) = default;


    void setTargetPosition(float x, float y, float z);
    void update();
    std::array<float, 3> getServoAngles() const;

    void getCurrentAngles(float& coxa, float& femur, float& tibia) const {
        coxa = coxa_angle;
        femur = femur_angle; 
        tibia = tibia_angle;
    }

    bool isIKValid() const {
        return (femur_angle >= -90 && femur_angle <= 180) &&
               (tibia_angle >= -90 && tibia_angle <= 90);
    }

private:
    int leg_id;
    int pca_id;
    int coxa_channel;
    int femur_channel;
    int tibia_channel;

    float target_x;
    float target_y;
    float target_z;

    float coxa_angle;
    float femur_angle;
    float tibia_angle;

    void computeIK();
};
