#include "hexapod.hpp"
#include "servo_controller.hpp"
#include "gait_controller.hpp"
#include <Arduino.h> 

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

void Hexapod::goHome() {
    Serial.println("Going to home position using IK...");
    
    // Vị trí home trong không gian 3D (đơn vị: mm)
    // Điều chỉnh các giá trị này cho phù hợp với robot của bạn
    const float homePositions[6][3] = {
        // [x, y, z] - tọa độ trong hệ tọa độ của từng chân
        {198.5f, -17.5f, -50.1f},   // Leg 0 (phải-trước)
        {198.5f, -17.5f, -50.1f},   // Leg 1 (phải-giữa) 
        {198.5f, -17.5f, -50.1f},   // Leg 2 (phải-sau)
        {198.5f, 17.5f, -50.1f},    // Leg 3 (trái-trước) - Lưu ý dấu Y đảo cho chân trái
        {198.5f, 17.5f, -50.1f},    // Leg 4 (trái-giữa) - Lưu ý dấu Y đảo cho chân trái
        {198.5f, 17.5f, -50.1f}     // Leg 5 (trái-sau) - Lưu ý dấu Y đảo cho chân trái
    };
    
    // Áp dụng vị trí home cho tất cả các chân
    for (int i = 0; i < 6; i++) {
        if (i < legs.size()) {
            Serial.print("Setting home position for leg ");
            Serial.print(i);
            Serial.print(": x=");
            Serial.print(homePositions[i][0]);
            Serial.print(", y=");
            Serial.print(homePositions[i][1]);
            Serial.print(", z=");
            Serial.println(homePositions[i][2]);
            
            legs[i].setTargetPosition(
                homePositions[i][0], 
                homePositions[i][1], 
                homePositions[i][2]
            );
            legs[i].update();
            
            delay(100); // Thời gian để servo di chuyển
        }
    }
    
    Serial.println("Home position completed using IK!");
}

void Hexapod::moveWithJoystick(float joystickX, float joystickY, bool continuous = false)
{
    // Áp dụng deadzone
    const float DEADZONE = 0.1f;
    if (abs(joystickX) < DEADZONE)
        joystickX = 0.0f;
    if (abs(joystickY) < DEADZONE)
        joystickY = 0.0f;

    // Bỏ qua nếu joystick ở vùng deadzone
    if (joystickX == 0.0f && joystickY == 0.0f)
    {
        if (!continuous)
        {
            goHome();
        }
        return;
    }

    // Tạo chân cho hexapod
    static Leg legs[6] = {
        Leg(0, 0, 2, 1, 0),  // Leg 0
        Leg(1, 0, 6, 5, 4),  // Leg 1
        Leg(2, 0, 10, 9, 8), // Leg 2
        Leg(3, 1, 2, 1, 0),  // Leg 3
        Leg(4, 1, 6, 5, 4),  // Leg 4
        Leg(5, 1, 10, 9, 8)  // Leg 5
    };

    // Định nghĩa tripods
    static int tripod1[] = {0, 2, 4};
    static int tripod2[] = {1, 3, 5};

    // Chuyển đổi giá trị joystick thành tham số di chuyển
    const float STRIDE_LENGTH = 70.0f; // Bước chân tối đa (mm)

    // Điều chỉnh tốc độ dựa trên mức độ nghiêng của joystick
    float speedFactor = max(abs(joystickX), abs(joystickY));
    float minDelay = 200; // Tốc độ tối đa
    float maxDelay = 400; // Tốc độ tối thiểu
    float baseStepDelay = maxDelay - speedFactor * (maxDelay - minDelay);

    // Xác định hướng di chuyển chủ đạo
    float absX = abs(joystickX);
    float absY = abs(joystickY);
    
    // Biến static cho việc luân phiên tripod
    static bool firstTripodActive = true;

    // Debug
    Serial.print("Speed: ");
    Serial.print(speedFactor);
    Serial.print(", Delay: ");
    Serial.println(baseStepDelay);

    // Xác định chế độ di chuyển: chỉ có 2 chế độ
    // 0: tiến/lùi, 1: rẽ trái/phải
    int moveMode;
    
    if (absY > absX) {
        moveMode = 0; // Tiến/lùi (Y dominant)
    } else {
        moveMode = 1; // Rẽ trái/phải (X dominant)
    }

    // Chế độ tiến/lùi
    if (moveMode == 0)
    {
        // Chế độ tiến/lùi
        float y = -joystickY * STRIDE_LENGTH; // Ngược dấu: joystick -Y = tiến tới
        float stepDelay = baseStepDelay;

        // Ghi log
        Serial.print("Forward/Backward: Y=");
        Serial.println(joystickY);

        // Thực hiện di chuyển tiến/lùi
        if (firstTripodActive)
        {
            GaitController::moveDirection(legs, tripod1, y, stepDelay);
            firstTripodActive = false;
            GaitController::moveDirection(legs, tripod1, y, stepDelay);
        }
        else
        {
            GaitController::moveDirection(legs, tripod2, y, stepDelay);
            firstTripodActive = true;
        }
    }
    // Chế độ rẽ trái/phải
    else if (moveMode == 1)
    {
        // Chế độ rẽ (quay)
        float turnFactor = joystickX; // -1 to 1
        float stepDelay = baseStepDelay;

        // Ghi log
        Serial.print("Turning: X=");
        Serial.println(joystickX);

        // Thực hiện chuyển động quay
        if (firstTripodActive)
        {
            GaitController::turnRobot(legs, tripod1, tripod2, turnFactor, stepDelay, true);
            firstTripodActive = false;
        }
        else
        {
            GaitController::turnRobot(legs, tripod2, tripod1, turnFactor, stepDelay, true);
            firstTripodActive = true;
        }
    }
}