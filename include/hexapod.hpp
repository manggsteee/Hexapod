#ifndef HEXAPOD_HPP
#define HEXAPOD_HPP

#include "leg.hpp"
#include "kinematics.hpp" // Để dùng Vec3 và NUM_LEGS
#include <array>
#include <vector>

class Hexapod {
public:
    Hexapod();

    // Thiết lập các chân với kênh servo tương ứng
    void setupLegs();

    // Cập nhật vị trí mục tiêu cho tất cả các chân
    void setTargetPositions(const std::array<Vec3, Kinematics::NUM_LEGS>& positions);

    // Gọi hàm update() của mỗi chân để di chuyển servo
    void update();

    void goHome();

    void moveWithJoystick(float joystickX, float joystickY, bool continuous);
    
private:
    // Sử dụng std::vector và std::move để khởi tạo danh sách các chân
    std::vector<Leg> legs;
};

#endif // HEXAPOD_HPP