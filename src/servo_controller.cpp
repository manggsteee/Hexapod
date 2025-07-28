#include "servo_controller.hpp"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Cấu hình cho servo 
#define SERVOMIN  150
#define SERVOMAX  600
#define SERVO_FREQ 50

// Khai báo 2 driver cho 2 mạch
static Adafruit_PWMServoDriver pwm_driver1 = Adafruit_PWMServoDriver(0x40); // PCA_ID = 0
static Adafruit_PWMServoDriver pwm_driver2 = Adafruit_PWMServoDriver(0x41); // PCA_ID = 1

bool ServoController::initialized = false;

void ServoController::initialize() {
    if (!initialized) {
        Wire.begin();
        pwm_driver1.begin();
        pwm_driver1.setPWMFreq(SERVO_FREQ);
        pwm_driver2.begin();
        pwm_driver2.setPWMFreq(SERVO_FREQ);
        initialized = true;
    }
}

static float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Cập nhật hàm setAngle để nhận thêm pca_id
void ServoController::setAngle(int pca_id, int channel, float angle) {
    if (!initialized) return;

    angle = std::max(0.0f, std::min(180.0f, angle));
    int pulse_length = static_cast<int>(map_float(angle, 0, 180, SERVOMIN, SERVOMAX));

    // Dựa vào pca_id để chọn đúng driver
    if (pca_id == 0) {
        pwm_driver1.setPWM(channel, 0, pulse_length);
    } else if (pca_id == 1) {
        pwm_driver2.setPWM(channel, 0, pulse_length);
    }
}
