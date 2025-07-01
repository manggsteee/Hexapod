#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <cstdint>

class ServoController {
public:
    // Khởi tạo giao tiếp I2C và cả hai mạch PCA9685
    static void initialize();

    // Đặt góc cho một servo cụ thể (0 - 180 độ)
    // Controller sẽ tự biết kênh này thuộc mạch PCA9685 nào
    static void setAngle(int pca_id, int channel, float angle);

private:
    static bool initialized;
};

#endif // SERVO_CONTROLLER_HPP