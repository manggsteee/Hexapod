#include <Arduino.h>
#include "hexapod.hpp"
#include "servo_controller.hpp"
#include "leg.hpp"
#include "ble_controller.hpp"

BLEController ble;

Hexapod hexapod;
Leg testLeg(1, 0, 6, 5, 4); // Test leg 1: coxa=ch6, femur=ch5, tibia=ch4

// Biến lưu góc hiện tại cho test
int currentLeg = 0;
float currentX = 0;
float currentY = 0;
float currentZ = 0;
Leg currentLegs[6] = {
    Leg(0, 0, 2, 1, 0),  // Leg 0
    Leg(1, 0, 6, 5, 4),  // Leg 1
    Leg(2, 0, 10, 9, 8), // Leg 2
    Leg(3, 1, 2, 1, 0),  // Leg 3
    Leg(4, 1, 6, 5, 4),  // Leg 4
    Leg(5, 1, 10, 9, 8)  // Leg 5
};

// Biến lưu góc servo hiện tại cho điều chỉnh trực tiếp
float currentServoAngles[6][3] = {
    {90.0f, 90.0f, 90.0f}, // Leg 0 (coxa, femur, tibia)
    {90.0f, 90.0f, 90.0f}, // Leg 1
    {90.0f, 90.0f, 90.0f}, // Leg 2
    {90.0f, 90.0f, 90.0f}, // Leg 3
    {90.0f, 90.0f, 90.0f}, // Leg 4
    {90.0f, 90.0f, 90.0f}  // Leg 5
};

// Tên của các khớp
const char* jointNames[] = {"Coxa", "Femur", "Tibia"};


// Các khai báo hàm (function prototypes)
void computeFK(int legId, float coxaAngle, float femurAngle, float tibiaAngle, float& x, float& y, float& z);
void computeFKArray(int legId, float coxaAngle, float femurAngle, float tibiaAngle);
void goHome();
void testLegForward();
void showTripodAngles(Leg legs[], const char *phase);
void testTripodGait();
void handlePosition();
void testServoAngles();
void chooseLeg();
void moveDirection(Leg legs[], int tripod1[], int tripod2[], float targetX, float targetY, float targetZ, int stepDelay, bool useInterpolation);
void moveLateral(Leg legs[], int tripod1[], int tripod2[], float x, float stepHeight, int stepDelay, bool useInterpolation);
float calculateTurnAdjustment(float y, int legIndex, float turnFactor);
void moveForward(Leg legs[], int tripod1[], int tripod2[]);
void turnRobot(Leg legs[], int tripod1[], int tripod2[], float turnFactor, int stepDelay, bool useInterpolation);
void moveWithJoystick(float joystickX, float joystickY, bool continuous);
void handleBLECommand(char input);
void testJoystickSimulation();
void createLegTrajectory(float t, float& x, float& y, float& z);
float bezier(float t, float p0, float p1, float p2, float p3);
void createSmoothTrajectory(float t, float& x, float& y, float& z);
void createBioinspiredTrajectory(float t, float& x, float& y, float& z);


void computeFK(int legId, float coxaAngle, float femurAngle, float tibiaAngle, 
              float& x, float& y, float& z) 
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
        {0.0f, 8.0f, -2.0f},     // Leg 0: 0°, +8°, -2°
        {-2.0f, 3.0f, 3.0f},   // Leg 1: -2°, +3°, +3°
        {-5.0f, -4.0f, 12.0f},   // Leg 2: -5°, -4°, +12°
        {8.0f, -10.0f, 7.0f},   // Leg 3: 8°, -10°, +7°
        {-6.0f, 3.0f, -10.0f},   // Leg 4: -6°, +3°, -10°
        {-0.02f, 5.22f, -5.20f}    // Leg 5: -0.02°, +5.22°, -5.20°
    };
    
    // Xác định hướng servo dựa vào ID chân
    int coxa_dir = (legId < 3) ? 1 : -1;
    int femur_dir = (legId < 3) ? 1 : -1;
    int tibia_dir = (legId < 3) ? 1 : -1;  // Hướng tibia luôn ngược

    // Thêm HOME_CORRECTION vào tính toán
    float coxa_correction = HOME_CORRECTION[legId][0];
    float femur_correction = HOME_CORRECTION[legId][1];
    float tibia_correction = HOME_CORRECTION[legId][2];

    float coxa_angle = (coxaAngle - COXA_OFFSET - coxa_correction) / coxa_dir;
    float femur_angle = (femurAngle - FEMUR_OFFSET - femur_correction) / femur_dir;
    float tibia_angle = (tibiaAngle - TIBIA_OFFSET - tibia_correction) / tibia_dir;

    // Chuyển đổi sang radians
    float coxa_rad = coxa_angle * M_PI / 180.0f;
    float femur_rad = femur_angle * M_PI / 180.0f;
    
    // Tính điểm cuối của coxa
    float coxa_x = COXA_LENGTH * cos(coxa_rad);
    float coxa_y = COXA_LENGTH * sin(coxa_rad);
    
    // Tính chiều dài phần femur trên mặt phẳng ngang và chiều cao
    float femur_proj = FEMUR_LENGTH * cos(femur_rad);
    float femur_height = FEMUR_LENGTH * sin(femur_rad);
    
    // Tính góc tổng của tibia so với mặt phẳng ngang
    float tibia_angle_total = femur_angle + tibia_angle;
    float tibia_rad_total = tibia_angle_total * M_PI / 180.0f;
    
    // Tính chiều dài phần tibia trên mặt phẳng ngang và chiều cao
    float tibia_proj = TIBIA_LENGTH * cos(tibia_rad_total);
    float tibia_height = TIBIA_LENGTH * sin(tibia_rad_total);
    
    // Tính tọa độ cuối cùng của đầu chân
    x = coxa_x + femur_proj * cos(coxa_rad) + tibia_proj * cos(coxa_rad);
    y = coxa_y + femur_proj * sin(coxa_rad) + tibia_proj * sin(coxa_rad);
    z = -(femur_height + tibia_height); // Đảo dấu z vì trục z hướng xuống trong hệ tọa độ robot
}

// Phiên bản trả về tọa độ dạng mảng
void computeFKArray(int legId, float coxaAngle, float femurAngle, float tibiaAngle) 
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

void goHome()
{
    Serial.println("Going to home position using IK...");

    // Vị trí home trong không gian 3D (đơn vị: mm)
    // Điều chỉnh các giá trị này cho phù hợp với robot của bạn
    const float homePositions[6][3] = {
        // [x, y, z] - tọa độ trong hệ tọa độ của từng chân
        {198.5f, 0.0f, -50.1f},   // Leg 0 (phải-trước)
        {198.5f, 0.0f, -50.1f},   // Leg 1 (phải-giữa) 
        {198.5f, 0.0f, -50.1f},   // Leg 2 (phải-sau)
        {198.5f, 0.0f, -50.1f},    // Leg 3 (trái-trước) - Lưu ý dấu Y đảo cho chân trái
        {198.5f, 0.0f, -50.1f},    // Leg 4 (trái-giữa) - Lưu ý dấu Y đảo cho chân trái
        {198.5f, 0.0f, -50.1f}     // Leg 5 (trái-sau) - Lưu ý dấu Y đảo cho chân trái
    };

    // Áp dụng vị trí home cho tất cả các chân
    for (int i = 0; i < 6; i++)
    {
        Serial.print("Setting home position for leg ");
        Serial.print(i);
        Serial.print(": x=");
        Serial.print(homePositions[i][0]);
        Serial.print(", y=");
        Serial.print(homePositions[i][1]);
        Serial.print(", z=");
        Serial.println(homePositions[i][2]);

        currentLegs[i].setTargetPosition(
            homePositions[i][0],
            homePositions[i][1],
            homePositions[i][2]);
        currentLegs[i].update();

        delay(100); // Thời gian để servo di chuyển
    }

    Serial.println("Home position completed using IK!");
}

void testLegForward()
{
    Serial.println("\n=== TESTING LEG 0 FORWARD ===");
    testLeg.setTargetPosition(200.0f, 0.0f, 0.0f);
    testLeg.update();
}

void showTripodAngles(Leg legs[], const char *phase)
{
    Serial.print("  ");
    Serial.print(phase);
    Serial.println(":");

    // Show Tripod 1
    Serial.print("    Tripod 1 (0,2,4): ");
    for (int i : {0, 2, 4})
    {
        auto angles = legs[i].getServoAngles();
        Serial.print("L");
        Serial.print(i);
        Serial.print("(");
        Serial.print((int)angles[0]);
        Serial.print(",");
        Serial.print((int)angles[1]);
        Serial.print(",");
        Serial.print((int)angles[2]);
        Serial.print(") ");
    }
    Serial.println();

    // Show Tripod 2
    Serial.print("    Tripod 2 (1,3,5): ");
    for (int i : {1, 3, 5})
    {
        auto angles = legs[i].getServoAngles();
        Serial.print("L");
        Serial.print(i);
        Serial.print("(");
        Serial.print((int)angles[0]);
        Serial.print(",");
        Serial.print((int)angles[1]);
        Serial.print(",");
        Serial.print((int)angles[2]);
        Serial.print(") ");
    }
    Serial.println();
}

// Thêm function này:
void testTripodGait()
{
    // Thông số di chuyển
    const float x_home = 198.5f;        // X coordinate for all legs
    const float y_home = 0.0f;          // Y coordinate for all legs
    const float z_home = -50.1f;        // Z coordinate for all legs
    const int stepDelay = 400;          // Delay giữa các bước (ms)
    const int UPDATE_INTERVAL = 15;     // Cập nhật mỗi 15ms để có chuyển động mượt
    const int STEPS = stepDelay / UPDATE_INTERVAL;
    
    Serial.println("\n=== TRIPOD GAIT TEST WITH SMOOTH TRAJECTORIES ===");
    Serial.println("Using Bezier curves for natural leg movement");
    Serial.println("Tripod 1: Legs 0, 2, 4 (phải-trước, phải-sau, trái-giữa)");
    Serial.println("Tripod 2: Legs 1, 3, 5 (phải-giữa, trái-trước, trái-sau)");
    Serial.println("Press any key to start walking sequence...");

    while (!Serial.available())
        delay(100);
    Serial.read();
    while (Serial.available())
        Serial.read();

    // Create all leg objects
    Leg legs[6] = {
        Leg(0, 0, 2, 1, 0),  // Leg 0
        Leg(1, 0, 6, 5, 4),  // Leg 1
        Leg(2, 0, 10, 9, 8), // Leg 2
        Leg(3, 1, 2, 1, 0),  // Leg 3
        Leg(4, 1, 6, 5, 4),  // Leg 4
        Leg(5, 1, 10, 9, 8)  // Leg 5
    };

    int tripod1[] = {0, 2, 4};
    int tripod2[] = {1, 3, 5};

    Serial.println("🚀 Starting tripod gait sequence with smooth trajectories...");

    // Walking cycles
    for (int cycle = 0; cycle < 3; cycle++)
    {
        Serial.print("\n=== WALKING CYCLE ");
        Serial.print(cycle + 1);
        Serial.println(" ===");

        // === TRIPOD 1 WALKING SEQUENCE (SMOOTH) ===
        Serial.println("TRIPOD 1 (0,2,4) WALKING WITH SMOOTH TRAJECTORY:");
        
        // Quỹ đạo mượt mà cho tripod1
        for (int step = 0; step < STEPS; step++) {
            float t = (float)step / STEPS;  // 0.0 -> 1.0
            
            for (int i = 0; i < 3; i++) {
                float x, y, z;
                // Sử dụng quỹ đạo Bezier
                createLegTrajectory(t, x, y, z);
                legs[tripod1[i]].setTargetPosition(x, y, z, t < 0.4f); // Elbow up for first half
                legs[tripod1[i]].update();
            }
            
            delay(UPDATE_INTERVAL);
        }

        showTripodAngles(legs, "Tripod 1 cycle completed");
        delay(100);  // Nhỏ delay giữa các chu kỳ

        // === TRIPOD 2 WALKING SEQUENCE (SMOOTH) ===
        Serial.println("TRIPOD 2 (1,3,5) WALKING WITH SMOOTH TRAJECTORY:");
        
        // Quỹ đạo mượt mà cho tripod2
        for (int step = 0; step < STEPS; step++) {
            float t = (float)step / STEPS;
            
            for (int i = 0; i < 3; i++) {
                float x, y, z;
                createLegTrajectory(t, x, y, z);
                legs[tripod2[i]].setTargetPosition(x, y, z, t < 0.4f); // Elbow up for first half
                legs[tripod2[i]].update();
            }
            
            delay(UPDATE_INTERVAL);
        }

        showTripodAngles(legs, "Tripod 2 cycle completed");
        
        if (cycle < 2)
        {
            Serial.println("Press any key for next cycle...");
            while (!Serial.available())
                delay(100);
            Serial.read();
            while (Serial.available())
                Serial.read();
        }
    }

    // Quay trở về vị trí home sau khi hoàn thành
    Serial.println("\nReturning to home position...");
    for (int i = 0; i < 6; i++)
    {
        legs[i].setTargetPosition(x_home, y_home, z_home);
        legs[i].update();
    }
    delay(500);

    Serial.println("\n🎉 SMOOTH TRIPOD GAIT TEST COMPLETED! 🎉");
}

void handlePosition()
{
    Serial.print("🦿 Điều chỉnh vị trí chân ");
    Serial.print(currentLeg);
    Serial.println(" (a/s: X, d/f: Y, g/h: Z, q: Quay lại)");
    Serial.print("Vị trí hiện tại: X=");
    Serial.print(currentX);
    Serial.print(", Y=");
    Serial.print(currentY);
    Serial.print(", Z=");
    Serial.println(currentZ);

    bool continueAdjusting = true;
    while (continueAdjusting)
    {
        while (Serial.available())
        {
            char input = Serial.read();
            switch (input)
            {
            case 'a':
                currentX += 1.0;
                Serial.print("X tăng: ");
                Serial.println(currentX);
                break;
            case 's':
                currentX -= 1.0;
                Serial.print("X giảm: ");
                Serial.println(currentX);
                break;
            case 'd':
                currentY += 1.0;
                Serial.print("Y tăng: ");
                Serial.println(currentY);
                break;
            case 'f':
                currentY -= 1.0;
                Serial.print("Y giảm: ");
                Serial.println(currentY);
                break;
            case 'g':
                currentZ += 1.0;
                Serial.print("Z tăng: ");
                Serial.println(currentZ);
                break;
            case 'h':
                currentZ -= 1.0;
                Serial.print("Z giảm: ");
                Serial.println(currentZ);
                break;
            case 'q':
            case 'Q':
                continueAdjusting = false;
                Serial.println("Quay lại menu chọn chân");
                return;
            default:
                break;
            }

            Serial.print("Đặt vị trí mới: X=");
            Serial.print(currentX);
            Serial.print(", Y=");
            Serial.print(currentY);
            Serial.print(", Z=");
            Serial.println(currentZ);

            currentLegs[currentLeg].setTargetPosition(currentX, currentY, currentZ);
            currentLegs[currentLeg].update();
        }
    }
}

void testServoAngles()
{
    Serial.print("🔄 Điều chỉnh góc servo chân ");
    Serial.print(currentLeg);
    Serial.println(" (q/w: Coxa, a/s: Femur, z/x: Tibia, r: Reset, e: Quay lại)");
    
    // Hiển thị góc hiện tại
    Serial.print("Góc hiện tại: Coxa=");
    Serial.print(currentServoAngles[currentLeg][0]);
    Serial.print("°, Femur=");
    Serial.print(currentServoAngles[currentLeg][1]);
    Serial.print("°, Tibia=");
    Serial.print(currentServoAngles[currentLeg][2]);
    Serial.println("°");


    bool continueAdjusting = true;
    while (continueAdjusting)
    {
        while (Serial.available())
        {
            char input = Serial.read();
            switch (input)
            {
            case 'q': // Tăng góc Coxa
                currentServoAngles[currentLeg][0] += 1.0f;
                Serial.print("Coxa +1°: ");
                Serial.println(currentServoAngles[currentLeg][0]);
                break;
            case 'w': // Giảm góc Coxa
                currentServoAngles[currentLeg][0] -= 1.0f;
                Serial.print("Coxa -1°: ");
                Serial.println(currentServoAngles[currentLeg][0]);
                break;
            case 'a': // Tăng góc Femur
                currentServoAngles[currentLeg][1] += 1.0f;
                Serial.print("Femur +1°: ");
                Serial.println(currentServoAngles[currentLeg][1]);
                break;
            case 's': // Giảm góc Femur
                currentServoAngles[currentLeg][1] -= 1.0f;
                Serial.print("Femur -1°: ");
                Serial.println(currentServoAngles[currentLeg][1]);
                break;
            case 'z': // Tăng góc Tibia
                currentServoAngles[currentLeg][2] += 1.0f;
                Serial.print("Tibia +1°: ");
                Serial.println(currentServoAngles[currentLeg][2]);
                break;
            case 'x': // Giảm góc Tibia
                currentServoAngles[currentLeg][2] -= 1.0f;
                Serial.print("Tibia -1°: ");
                Serial.println(currentServoAngles[currentLeg][2]);
                break;
            case 'Q': // Tăng góc Coxa 5°
                currentServoAngles[currentLeg][0] += 5.0f;
                Serial.print("Coxa +5°: ");
                Serial.println(currentServoAngles[currentLeg][0]);
                break;
            case 'W': // Giảm góc Coxa 5°
                currentServoAngles[currentLeg][0] -= 5.0f;
                Serial.print("Coxa -5°: ");
                Serial.println(currentServoAngles[currentLeg][0]);
                break;
            case 'A': // Tăng góc Femur 5°
                currentServoAngles[currentLeg][1] += 5.0f;
                Serial.print("Femur +5°: ");
                Serial.println(currentServoAngles[currentLeg][1]);
                break;
            case 'S': // Giảm góc Femur 5°
                currentServoAngles[currentLeg][1] -= 5.0f;
                Serial.print("Femur -5°: ");
                Serial.println(currentServoAngles[currentLeg][1]);
                break;
            case 'Z': // Tăng góc Tibia 5°
                currentServoAngles[currentLeg][2] += 5.0f;
                Serial.print("Tibia +5°: ");
                Serial.println(currentServoAngles[currentLeg][2]);
                break;
            case 'X': // Giảm góc Tibia 5°
                currentServoAngles[currentLeg][2] -= 5.0f;
                Serial.print("Tibia -5°: ");
                Serial.println(currentServoAngles[currentLeg][2]);
                break;
            case 'r': // Reset về 90°
                currentServoAngles[currentLeg][0] = 90.0f;
                currentServoAngles[currentLeg][1] = 90.0f;
                currentServoAngles[currentLeg][2] = 90.0f;
                Serial.println("Reset góc về 90°");
                break;
            case 'h': // Tính góc HOME_CORRECTION từ góc hiện tại
                Serial.println("\n=== Tính HOME_CORRECTION ===");
                Serial.print("Coxa_correction = ");
                Serial.print(currentServoAngles[currentLeg][0] - 90.0f);
                Serial.print(", Femur_correction = ");
                Serial.print(currentServoAngles[currentLeg][1] - 90.0f);
                Serial.print(", Tibia_correction = ");
                Serial.println(currentServoAngles[currentLeg][2] - 90.0f);
                Serial.println("Sao chép giá trị này vào mảng HOME_CORRECTION của bạn");
                break;
            case 'e':
                continueAdjusting = false;
                Serial.println("Quay lại menu chọn chân");
                return;
            default:
                break;
            }

            // Giới hạn góc trong khoảng hợp lệ (0-180°)
            for (int j = 0; j < 3; j++) {
                currentServoAngles[currentLeg][j] = std::max(0.0f, std::min(180.0f, currentServoAngles[currentLeg][j]));
            }

            // Áp dụng góc mới
            int pca_id = currentLeg / 3;  // 0 cho chân 0-2, 1 cho chân 3-5
            
            // Xác định các kênh servo cho từng chân
            int channels[3];
            if (currentLeg % 3 == 0) {         // Chân 0 hoặc 3
                channels[0] = 2;  // coxa
                channels[1] = 1;  // femur
                channels[2] = 0;  // tibia
            } else if (currentLeg % 3 == 1) {  // Chân 1 hoặc 4
                channels[0] = 6;  // coxa
                channels[1] = 5;  // femur
                channels[2] = 4;  // tibia
            } else {                           // Chân 2 hoặc 5
                channels[0] = 10; // coxa
                channels[1] = 9;  // femur
                channels[2] = 8;  // tibia
            }
            
            // Áp dụng góc mới trực tiếp cho servo
            for (int j = 0; j < 3; j++) {
                ServoController::setAngle(pca_id, channels[j], currentServoAngles[currentLeg][j]);
            }
            
            // Hiển thị góc hiện tại của tất cả các khớp
            Serial.print("Góc hiện tại: Coxa=");
            Serial.print(currentServoAngles[currentLeg][0]);
            Serial.print("°, Femur=");
            Serial.print(currentServoAngles[currentLeg][1]);
            Serial.print("°, Tibia=");
            Serial.print(currentServoAngles[currentLeg][2]);
            Serial.println("°");
            computeFKArray(currentLeg, 
                currentServoAngles[currentLeg][0], 
                currentServoAngles[currentLeg][1], 
                currentServoAngles[currentLeg][2]);
        }
    }
}

// Sửa đổi hàm chooseLeg để thêm tùy chọn chọn phương pháp điều chỉnh
void chooseLeg()
{
    Serial.println("\n===== CHỌN CHÂN ĐỂ ĐIỀU CHỈNH =====");
    Serial.println("Nhấn số từ 0-5 để chọn chân tương ứng:");
    Serial.println("  0: Chân phải-trước");
    Serial.println("  1: Chân phải-giữa");
    Serial.println("  2: Chân phải-sau");
    Serial.println("  3: Chân trái-trước");
    Serial.println("  4: Chân trái-giữa");
    Serial.println("  5: Chân trái-sau");
    Serial.println("  q: Quay lại menu chính");

    bool selecting = true;

    while (selecting)
    {
        while (Serial.available())
        {
            char input = Serial.read();
            switch (input)
            {
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            {
                currentLeg = input - '0';
                Serial.print("Đã chọn chân ");
                Serial.print(currentLeg);
                Serial.println(" để điều chỉnh.");
                
                Serial.println("\nChọn phương pháp điều chỉnh:");
                Serial.println("  1: Điều chỉnh vị trí (X,Y,Z)");
                Serial.println("  2: Điều chỉnh góc servo trực tiếp");
                Serial.println("  q: Quay lại");
                
                bool choosingMethod = true;
                while (choosingMethod) {
                    if (Serial.available()) {
                        char methodInput = Serial.read();
                        switch (methodInput) {
                            case '1':
                                // Đọc vị trí hiện tại của chân từ target_x, target_y, target_z
                                {
                                    auto position = currentLegs[currentLeg].getTargetPosition();
                                    currentX = position[0];
                                    currentY = position[1];
                                    currentZ = position[2];
                                    
                                    handlePosition();
                                    choosingMethod = false;
                                }
                                break;
                            case '2':
                                // Đọc góc servo hiện tại
                                {
                                    auto angles = currentLegs[currentLeg].getServoAngles();
                                    currentServoAngles[currentLeg][0] = angles[0];
                                    currentServoAngles[currentLeg][1] = angles[1];
                                    currentServoAngles[currentLeg][2] = angles[2];
                                    
                                    testServoAngles();
                                    choosingMethod = false;
                                }
                                break;
                            case 'q':
                            case 'Q':
                                choosingMethod = false;
                                break;
                            default:
                                Serial.println("Lựa chọn không hợp lệ. Vui lòng chọn lại.");
                                break;
                        }
                    }
                }
            }
            break;
            case 'q':
            case 'Q':
                selecting = false;
                Serial.println("Quay lại menu chính");
                return;
            default:
                Serial.println("Lựa chọn không hợp lệ. Vui lòng chọn lại.");
                break;
            }
        }
    }
}

void moveDirection(Leg legs[], int tripod[],
                   float targetX, float targetY, float targetZ,
                   int stepDelay = 200, bool useInterpolation = true)
{
    const int UPDATE_INTERVAL = 15;
    const int STEPS = stepDelay / UPDATE_INTERVAL;
    
    // === TRIPOD 1 MOVEMENT ===
    for (int step = 0; step < STEPS; step++) {
        float t = (float)step / STEPS;
        
        for (int i = 0; i < 3; i++) {
            float x, y, z;
            createLegTrajectory(t, x, y, z);
            // Chỉ dùng elbow_up khi nhấc chân
            legs[tripod[i]].setTargetPosition(x, y, z, t < 0.5f);
            legs[tripod[i]].update();
        }
        
        delay(UPDATE_INTERVAL);
    }
}

void moveLateral(Leg legs[], int tripod1[], int tripod2[],
                 float x = 225.0f, float stepHeight = 270.0f, int stepDelay = 200,
                 bool useInterpolation = true)
{

    // Thêm các tham số cho chuyển động mượt mà
    const int UPDATE_INTERVAL = 10; // Cập nhật mỗi 10ms để có chuyển động mượt

    // Step 1: Nhấc tripod 1 lên
    Serial.println("  Step 1: Tripod 1 UP for lateral movement");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, 0.0f, stepHeight);
    }

    // Cập nhật liên tục để có chuyển động mượt mà
    if (useInterpolation)
    {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL)
        {
            for (int i = 0; i < 3; i++)
            {
                legs[tripod1[i]].update();
            }
            delay(UPDATE_INTERVAL);
        }
    }
    else
    {
        delay(stepDelay);
    }

    // Step 2: Di chuyển tripod 1 sang trái/phải
    Serial.println("  Step 2: Tripod 1 LATERAL movement");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(x, 0.0f, stepHeight);
    }

    // Cập nhật liên tục
    if (useInterpolation)
    {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL)
        {
            for (int i = 0; i < 3; i++)
            {
                legs[tripod1[i]].update();
            }
            delay(UPDATE_INTERVAL);
        }
    }
    else
    {
        delay(stepDelay);
    }

    // Step 3: Hạ tripod 1 xuống và nhấc tripod 2 lên
    Serial.println("  Step 3: Tripod 1 DOWN, Tripod 2 UP");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(x, 0.0f, 0.0f);

        legs[tripod2[i]].setTargetPosition(225.0f, 0.0f, stepHeight);
    }

    // Cập nhật liên tục
    if (useInterpolation)
    {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL)
        {
            for (int i = 0; i < 3; i++)
            {
                legs[tripod1[i]].update();
                legs[tripod2[i]].update();
            }
            delay(UPDATE_INTERVAL);
        }
    }
    else
    {
        delay(stepDelay);
    }

    // Step 4: Di chuyển thân robot (tripod 1) và tripod 2
    Serial.println("  Step 4: Body shift and Tripod 2 LATERAL");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, 0.0f, 0.0f);

        legs[tripod2[i]].setTargetPosition(x, 0.0f, stepHeight);
    }

    // Cập nhật liên tục
    if (useInterpolation)
    {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL)
        {
            for (int i = 0; i < 3; i++)
            {
                legs[tripod1[i]].update();
                legs[tripod2[i]].update();
            }
            delay(UPDATE_INTERVAL);
        }
    }
    else
    {
        delay(stepDelay);
    }

    // Step 5: Hạ tripod 2 xuống
    Serial.println("  Step 5: Tripod 2 DOWN");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod2[i]].setTargetPosition(x, 0.0f, 0.0f);
    }

    // Cập nhật liên tục
    if (useInterpolation)
    {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL)
        {
            for (int i = 0; i < 3; i++)
            {
                legs[tripod2[i]].update();
            }
            delay(UPDATE_INTERVAL);
        }
    }
    else
    {
        delay(stepDelay);
    }
}

// Hàm tính toán giá trị Y đã điều chỉnh cho việc rẽ
float calculateTurnAdjustment(float y, int legIndex, float turnFactor = 0.0f) 
{
    const float DEADZONE = 20.0f;   // Giới hạn dưới cho hiệu chỉnh rẽ
    float adjusted_y = y;

    // Chỉ áp dụng hiệu chỉnh nếu có turnFactor hoặc y đủ lớn (đang rẽ)
    if (abs(turnFactor) > DEADZONE || abs(y) > DEADZONE) 
    {
        // Xác định hệ số điều chỉnh dựa trên hướng và vị trí chân
        bool isRightLeg = (legIndex < 3); // 0, 1, 2 là chân phải
        bool isTurningRight;
        
        // Nếu có turnFactor riêng, dùng nó để xác định hướng rẽ
        if (abs(turnFactor) > DEADZONE) {
            isTurningRight = (turnFactor > 0);
        } else {
            // Ngược lại, dùng y (cách cũ)
            isTurningRight = (y < 0);
        }

        // Khi rẽ phải, chân trái bước dài hơn và ngược lại
        if ((isRightLeg && !isTurningRight) || (!isRightLeg && isTurningRight)) 
        {
            adjusted_y *= 1.3f; // Bước dài hơn
        } 
        else 
        {
            adjusted_y *= 0.75f; // Bước ngắn hơn
        }
        
        // Nếu có turnFactor, áp dụng thêm vào y
        if (abs(turnFactor) > DEADZONE) {
            adjusted_y += turnFactor;
        }
    }

    return adjusted_y;
}

void moveForward(Leg legs[], int tripod1[], int tripod2[])
{
    // Step 2: Tripod 1 FORWARD
    Serial.println("  Step 2: Tripod 1 FORWARD");
    for (int i = 0; i < 3; i++)
    {
        if (tripod1[i] < 3)
        {
            legs[tripod1[i]].setTargetPosition(225.0f, -150.0f, 270.0f); // SAME as case '6' step 2
        }
        else
        {
            legs[tripod1[i]].setTargetPosition(225.0f, -150.0f, 270.0f); // SAME as case '6' step 2
        }
        legs[tripod1[i]].update();
    }
    delay(600);

    // Step 3: Tripod 1 DOWN
    Serial.println("  Step 3: Tripod 1 DOWN");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, -150.0f, 0.0f); // SAME as case '6' step 3
        legs[tripod1[i]].update();
        // legs[tripod2[i]].setTargetPosition(225.0f, 0.0f, 270.0f,true); // SAME as case '6' step 3
        // legs[tripod2[i]].update();
    }
    delay(600);

    // Step 4: Tripod 1 BACKWARD (body moves forward)
    Serial.println("  Step 4: Tripod 1 BACKWARD");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, 0.0f, 0.0f); // SAME as case '6' step 4
        legs[tripod1[i]].update();
    }
}

void turnRobot(Leg legs[], int tripod1[], int tripod2[], float turnFactor, int stepDelay = 200, bool useInterpolation = true)
{
    const int UPDATE_INTERVAL = 10;
    const float TURN_STRIDE = 100.0f; // Biên độ rẽ cơ bản
    const float STEP_HEIGHT = 270.0f;
    const float BASE_X = 225.0f;
    
    // Xác định hướng rẽ
    bool isTurningRight = (turnFactor > 0);
    float magnitude = min(abs(turnFactor), 1.0f); // Giới hạn trong khoảng [0, 1]
    
    // Tính toán các offset cơ bản cho toàn bộ hàm
    float offsetRight = isTurningRight ? -TURN_STRIDE * magnitude : TURN_STRIDE * magnitude;
    float offsetLeft = -offsetRight; // Ngược dấu với offsetRight
    
    Serial.print("TURNING ");
    Serial.print(isTurningRight ? "RIGHT" : "LEFT");
    Serial.print(" with magnitude: ");
    Serial.println(magnitude);
    
    // BƯỚC 1: Nhấc các chân tripod1 lên
    Serial.println("  Step 1: Tripod 1 UP");
    for (int i = 0; i < 3; i++) {
        legs[tripod1[i]].setTargetPosition(BASE_X, 0.0f, STEP_HEIGHT);
        legs[tripod1[i]].update();
    }
    
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }
    
    // BƯỚC 2: Di chuyển các chân tripod1 sang ngang (ngược hướng rẽ)
    Serial.println("  Step 2: Tripod 1 ROTATE");
    for (int i = 0; i < 3; i++) {
        int legId = tripod1[i];
        bool isRightLeg = (legId < 3);
        bool isFrontLeg = (legId == 0 || legId == 3);
        bool isMiddleLeg = (legId == 1 || legId == 4);
        
        // Điều chỉnh tỷ lệ dựa trên vị trí chân
        float scaleFactor = 1.0f;
        if (isMiddleLeg) scaleFactor = 0.6f; // Chân giữa quay ít hơn
        if (!isFrontLeg && !isMiddleLeg) scaleFactor = 0.8f; // Chân sau quay ít hơn chân trước
        
        // Tính toán offset chuyển động cho từng chân
        float yOffset = isRightLeg ? 
            offsetRight * scaleFactor : 
            offsetLeft * scaleFactor;
        
        // Điều chỉnh X để tạo góc rẽ tự nhiên hơn
        float xOffset = 0.0f;
        if ((isRightLeg && isTurningRight) || (!isRightLeg && !isTurningRight)) {
            // Chân bên ngoài vòng quay: di chuyển ra ngoài hơn
            xOffset = 15.0f * magnitude;
        } else {
            // Chân bên trong vòng quay: di chuyển vào trong hơn
            xOffset = -10.0f * magnitude;
        }
        
        legs[legId].setTargetPosition(BASE_X + xOffset, yOffset, STEP_HEIGHT);
        legs[legId].update();
    }
    
    // Các bước tiếp theo tương tự...
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }
    
    // BƯỚC 3: Hạ các chân tripod1 xuống
    Serial.println("  Step 3: Tripod 1 DOWN");
    for (int i = 0; i < 3; i++) {
        int legId = tripod1[i];
        bool isRightLeg = (legId < 3);
        bool isFrontLeg = (legId == 0 || legId == 3);
        bool isMiddleLeg = (legId == 1 || legId == 4);
        
        // Sử dụng cùng phương pháp tính toán như Bước 2
        float scaleFactor = 1.0f;
        if (isMiddleLeg) scaleFactor = 0.6f;
        if (!isFrontLeg && !isMiddleLeg) scaleFactor = 0.8f;
        
        float yOffset = isRightLeg ? 
            offsetRight * scaleFactor : 
            offsetLeft * scaleFactor;
        
        float xOffset = 0.0f;
        if ((isRightLeg && isTurningRight) || (!isRightLeg && !isTurningRight)) {
            xOffset = 15.0f * magnitude;
        } else {
            xOffset = -10.0f * magnitude;
        }
        
        legs[legId].setTargetPosition(BASE_X + xOffset, yOffset, 0.0f);
        legs[legId].update();
    }
    
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }
    
    // BƯỚC 4: Nhấc các chân tripod2 lên
    Serial.println("  Step 4: Tripod 2 UP");
    for (int i = 0; i < 3; i++) {
        legs[tripod2[i]].setTargetPosition(BASE_X, 0.0f, STEP_HEIGHT);
        legs[tripod2[i]].update();
    }
    
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }
    
    // BƯỚC 5: Di chuyển các chân tripod2 sang ngang (ngược hướng rẽ)
    Serial.println("  Step 5: Tripod 2 ROTATE");
    for (int i = 0; i < 3; i++) {
        bool isRightLeg = (tripod2[i] < 3);
        float yOffset = isRightLeg ? offsetRight : offsetLeft;
        legs[tripod2[i]].setTargetPosition(BASE_X, yOffset, STEP_HEIGHT);
        legs[tripod2[i]].update();
    }
    
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }
    
    // BƯỚC 6: Hạ các chân tripod2 xuống
    Serial.println("  Step 6: Tripod 2 DOWN");
    for (int i = 0; i < 3; i++) {
        bool isRightLeg = (tripod2[i] < 3);
        float yOffset = isRightLeg ? offsetRight : offsetLeft;
        legs[tripod2[i]].setTargetPosition(BASE_X, yOffset, 0.0f);
        legs[tripod2[i]].update();
    }
    
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }
    
    // BƯỚC 7: Đưa tất cả các chân về vị trí trung tâm
    Serial.println("  Step 7: All legs to center");
    for (int i = 0; i < 6; i++) {
        legs[i].setTargetPosition(BASE_X, 0.0f, 0.0f);
        legs[i].update();
    }
    
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }
}


void moveWithJoystick(float joystickX, float joystickY, bool continuous = false)
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
    const float STEP_HEIGHT = 56.29f;   // Độ cao bước chân (mm)
    const float BASE_X = 179.44f;        // Vị trí X mặc định (mm)
    const float LATERAL_MOVE = 80.0f;   // Phạm vi di chuyển ngang tối đa (mm)

    // Điều chỉnh tốc độ dựa trên mức độ nghiêng của joystick
    float speedFactor = max(abs(joystickX), abs(joystickY));
    float minDelay = 200; // Tốc độ tối đa
    float maxDelay = 400; // Tốc độ tối thiểu
    float baseStepDelay = maxDelay - speedFactor * (maxDelay - minDelay);

    // Xác định hướng di chuyển chủ đạo
    float absX = abs(joystickX);
    float absY = abs(joystickY);
    
    // Biến static riêng cho mỗi chế độ
    static bool firstTripodActiveForward = true;
    static bool firstTripodActiveLateral = true;

    // Debug
    Serial.print("Speed: ");
    Serial.print(speedFactor);
    Serial.print(", Delay: ");
    Serial.println(baseStepDelay);

    // Xác định chế độ di chuyển
    // 0: tiến/lùi, 1: rẽ trái/phải, 2: di chuyển ngang
    int moveMode;
    
    if (absY > absX * 1.5) {
        moveMode = 0; // Tiến/lùi
    } else if (absX > absY * 1.5) {
        moveMode = 1; // Rẽ trái/phải
    } else {
        moveMode = 2; // Di chuyển ngang (lateral)
    }

    // Chế độ tiến/lùi
    if (moveMode == 0)
    {
        // Chế độ tiến/lùi (không có rẽ)
        float y = -joystickY * STRIDE_LENGTH; // Ngược dấu: joystick -Y = tiến tới
        float stepDelay = baseStepDelay;

        // Ghi log
        Serial.print("Forward/Backward: Y=");
        Serial.println(joystickY);

        // Thực hiện di chuyển tiến/lùi
        if (firstTripodActiveForward)
        {
            moveDirection(legs, tripod1, BASE_X, y, STEP_HEIGHT, stepDelay, true);
            firstTripodActiveForward = false;
        }
        else
        {
            moveDirection(legs, tripod2, BASE_X, y, STEP_HEIGHT, stepDelay, true);
            firstTripodActiveForward = true;
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
        if (firstTripodActiveForward)
        {
            turnRobot(legs, tripod1, tripod2, turnFactor, stepDelay, true);
            firstTripodActiveForward = false;
        }
        else
        {
            turnRobot(legs, tripod2, tripod1, turnFactor, stepDelay, true);
            firstTripodActiveForward = true;
        }
    }
    // Chế độ di chuyển ngang
    else if (moveMode == 2)
    {
        // Chế độ di chuyển ngang (trái/phải)
        float x = BASE_X + (joystickX * LATERAL_MOVE);
        float stepDelay = baseStepDelay;

        // Ghi log
        Serial.print("Lateral movement: X=");
        Serial.println(joystickX);

        // Thực hiện di chuyển ngang
        if (firstTripodActiveLateral)
        {
            moveLateral(legs, tripod1, tripod2, x, STEP_HEIGHT, stepDelay);
            firstTripodActiveLateral = false;
        }
        else
        {
            moveLateral(legs, tripod2, tripod1, x, STEP_HEIGHT, stepDelay);
            firstTripodActiveLateral = true;
        }
    }
}

// void controlBodyTilt(float joystickX, float joystickY) {
//     // Bỏ qua nếu joystick ở vùng deadzone
//     if (abs(joystickX) < 0.1f && abs(joystickY) < 0.1f) {
//         return;
//     }

//     // Tạo chân và offset height
//     const float MAX_TILT = 50.0f;       // Độ nghiêng tối đa (mm)
//     const float BASE_HEIGHT = 0.0f;     // Chiều cao mặc định (mm)
//     const float BASE_X = 225.0f;        // Vị trí X mặc định (mm)

//     // Tính toán độ cao cho từng chân
//     float heights[6] = {0}; // 6 giá trị chiều cao

//     // Nghiêng trước/sau: chân trước và sau có chiều cao khác nhau
//     float frontBack = -joystickY * MAX_TILT; // Đảo dấu: -Y = nghiêng về phía trước

//     // Nghiêng trái/phải: chân trái và phải có chiều cao khác nhau
//     float leftRight = joystickX * MAX_TILT;

//     // Tính chiều cao cho từng chân
//     // Chân 0: phải-trước, 1: phải-giữa, 2: phải-sau
//     // Chân 3: trái-trước, 4: trái-giữa, 5: trái-sau

//     // Phía trước (0, 3)
//     heights[0] = BASE_HEIGHT + frontBack - leftRight;
//     heights[3] = BASE_HEIGHT + frontBack + leftRight;

//     // Phía giữa (1, 4)
//     heights[1] = BASE_HEIGHT - leftRight;
//     heights[4] = BASE_HEIGHT + leftRight;

//     // Phía sau (2, 5)
//     heights[2] = BASE_HEIGHT - frontBack - leftRight;
//     heights[5] = BASE_HEIGHT - frontBack + leftRight;

//     // Áp dụng vị trí cho tất cả các chân
//     for (int i = 0; i < 6; i++) {
//         currentLegs[i].setTargetPosition(BASE_X, 0.0f, heights[i]);
//         currentLegs[i].update();
//     }
// }

void handleBLECommand(char input)
{
    Serial.print("BLE Command: ");
    Serial.println(input);

    switch (input)
    {
    case 'r':
        Serial.println("Lệnh: Về home position");
        goHome();
        break;
    case 'j':
        Serial.println("Lệnh: Test leg forward");
        testLegForward();
        break;
    case 'm':
        Serial.println("Lệnh: Test tripod gait");
        testTripodGait();
        break;
    case 'h':
        Serial.println("Lệnh: Chọn chân để điều chỉnh");
        chooseLeg();
        break;
    case 'J': // Nhận dữ liệu joystick
        if (ble.hasJoystickData())
        {
            float joyX = ble.getJoystickX();
            float joyY = ble.getJoystickY();
            moveWithJoystick(joyX, joyY);
        }
        break;
    default:
        Serial.println("Lệnh không hợp lệ! Nhấn 'h' để hiển thị trợ giúp.");
        break;
    }
}

/**
 * Giả lập điều khiển joystick với nhiều hướng di chuyển
 * Thử nghiệm robot di chuyển theo 8 hướng cơ bản
 */
void testJoystickSimulation()
{
    Serial.println("\n=== JOYSTICK SIMULATION TEST ===");
    Serial.println("Testing movement in different directions");
    Serial.println("Press any key to start...");

    while (!Serial.available())
        delay(100);
    Serial.read();
    while (Serial.available())
        Serial.read();

    // Khởi tạo các chân
    Leg legs[6] = {
        Leg(0, 0, 2, 1, 0),  // Leg 0
        Leg(1, 0, 6, 5, 4),  // Leg 1
        Leg(2, 0, 10, 9, 8), // Leg 2
        Leg(3, 1, 2, 1, 0),  // Leg 3
        Leg(4, 1, 6, 5, 4),  // Leg 4
        Leg(5, 1, 10, 9, 8)  // Leg 5
    };

    int tripod1[] = {0, 2, 4};
    int tripod2[] = {1, 3, 5};

    // Các trường hợp kiểm tra với các tọa độ (x, y) khác nhau
    struct TestCase
    {
        const char *name;
        float x;
        float y;
        int cycles;
    };

    TestCase testCases[] = {
        {"FORWARD", 225.0f, -150.0f, 3},       // Tiến
        {"BACKWARD", 225.0f, 150.0f, 3},       // Lùi
        {"LEFT TURN", 225.0f, 100.0f, 4},      // Rẽ trái
        {"RIGHT TURN", 225.0f, -100.0f, 4},    // Rẽ phải
        {"FORWARD LEFT", 235.0f, -75.0f, 3},   // Tiến-trái
        {"FORWARD RIGHT", 235.0f, -225.0f, 3}, // Tiến-phải
        {"BACKWARD LEFT", 235.0f, 225.0f, 3},  // Lùi-trái
        {"BACKWARD RIGHT", 235.0f, 75.0f, 3}   // Lùi-phải
    };

    // Test từng trường hợp
    for (const auto &test : testCases)
    {
        Serial.print("\n\n🚶 TESTING ");
        Serial.print(test.name);
        Serial.println(" MOVEMENT:");
        Serial.print("  X = ");
        Serial.print(test.x);
        Serial.print(", Y = ");
        Serial.print(test.y);
        Serial.print(", Cycles = ");
        Serial.println(test.cycles);

        // Trở về vị trí home trước khi test hướng mới
        goHome();
        delay(1000);

        // Thực hiện số chu kỳ bước chân
        for (int cycle = 0; cycle < test.cycles; cycle++)
        {
            Serial.print("  Cycle ");
            Serial.print(cycle + 1);
            Serial.print(" of ");
            Serial.println(test.cycles);

            // Di chuyển sử dụng tripod gait
            moveDirection(legs, tripod1, tripod2, test.x, test.y, 56.29f, 200, true);
            delay(100);
            moveDirection(legs, tripod2, tripod1, test.x, test.y, 56.29f, 200, true);
            delay(100);
        }

        Serial.print("✅ ");
        Serial.print(test.name);
        Serial.println(" test completed!");

        // Dừng trước khi chuyển sang hướng tiếp theo
        Serial.println("Press any key for next direction...");
        while (!Serial.available())
            delay(100);
        Serial.read();
        while (Serial.available())
            Serial.read();
    }

    // Quay về vị trí home khi hoàn thành
    goHome();
    Serial.println("\n🎉 JOYSTICK SIMULATION TEST COMPLETED! 🎉");
}

void setup()
{
    Serial.begin(115200);
    ble.init("Hexapod_BLE");
    ble.enableAutoConnect(true);
    ble.setCommandHandler(handleBLECommand);

    ServoController::initialize();
    delay(1000);

    // Set all servos to 90° initially
    goHome();
    // delay(1000);
    // testLegForward();

    // Leg legs[6] = {
    //     Leg(0, 0, 2, 1, 0),  // Leg 0
    //     Leg(1, 0, 6, 5, 4),  // Leg 1
    //     Leg(2, 0, 10, 9, 8), // Leg 2
    // };

    // int tripod1[] = {0, 2, 4};
    // for (int i = 0; i < 3; i++)
    // {
    //     if (i < 2)
    //         legs[tripod1[i]].setTargetPosition(150.0f, 0.0f, 300.0f,true); // SAME as case '6' step 1
    //     else
    //         legs[tripod1[i]].setTargetPosition(250.0f, 0.0f, 300.0f,true); // SAME as case '6' step 1
    //     legs[tripod1[i]].update();
    // }
}

void loop()
{
    ble.loop();

    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 10000)
    { // Mỗi 10 giây
        if (ble.isConnected())
        {
            // Gửi thông báo trạng thái khi đã kết nối
            ble.sendMessage("STATUS:CONNECTED");
        }
        lastStatusUpdate = millis();
    }

    // Xử lý dữ liệu joystick theo thời gian thực
    static unsigned long lastJoystickUpdate = 0;
    static unsigned long lastJoystickActivity = 0;
    static bool isMoving = false;
    if (ble.hasJoystickData() && millis() - lastJoystickUpdate > 50)
    { // Tăng lên 2Hz
        float joyX = ble.getJoystickX();
        float joyY = ble.getJoystickY();

        // Debug
        if (abs(joyX) > 0.1f || abs(joyY) > 0.1f)
        {
            Serial.print("Joy: X=");
            Serial.print(joyX);
            Serial.print(", Y=");
            Serial.println(joyY);
            // Đánh dấu robot đang di chuyển
            isMoving = true;
            lastJoystickActivity = millis();
        }
        else if (isMoving)
        {
            // Dừng robot khi joystick về trung tâm
            Serial.println("Joystick centered, stopping movement");
            goHome();
            isMoving = false;
        }

        // Di chuyển robot theo joystick
        moveWithJoystick(joyX, joyY, false);
        lastJoystickUpdate = millis();
    }

    // Kiểm tra timeout: nếu không nhận được dữ liệu trong 500ms và robot đang di chuyển
    if (isMoving && (millis() - lastJoystickActivity > 300))
    {
        Serial.println("Joystick timeout, stopping movement");
        goHome();
        isMoving = false;
    }

    // Xử lý lệnh đơn ký tự (cho các chức năng khác)
    if (ble.hasCommand())
    {
        char cmd = ble.getCommand();
        if (cmd != 0)
        {
            handleBLECommand(cmd);
        }
    }

    // Xử lý lệnh từ Serial (debugging)
    if (Serial.available())
    {
        char input = Serial.read();
        handleBLECommand(input);
    }

    // Xử lý lệnh từ Serial console (cho debugging)

    // if (Serial.available())
    // {
    //     char input = Serial.read();
    //     handleBLECommand(input);
    // }
    // Leg legs[6] = {
    //     Leg(0, 0, 2, 1, 0),  // Leg 0
    //     Leg(1, 0, 6, 5, 4),  // Leg 1
    //     Leg(2, 0, 10, 9, 8), // Leg 2
    //     Leg(3, 1, 2, 1, 0),  // Leg 3
    //     Leg(4, 1, 6, 5, 4),  // Leg 4
    //     Leg(5, 1, 10, 9, 8)  // Leg 5
    // };

    // int tripod1[] = {0, 2, 4};
    // int tripod2[] = {1, 3, 5};
    // moveForward(legs, tripod1, tripod2);
    // delay(200);
    // moveForward(legs, tripod2, tripod1);
    // delay(200);
    // testJoystickSimulation();
}

void createLegTrajectory(float t, float& x, float& y, float& z) {
    // t: tham số từ 0.0 đến 1.0 (tiến trình của chuyển động)
    // Quỹ đạo hình bán nguyệt khi nhấc chân
    
    const float HOME_X = 198.5f;
    const float HOME_Y = 0.0f; 
    const float HOME_Z = -50.1f;
    
    const float STRIDE_LENGTH = 150.0f;  // Độ dài bước chân
    const float LIFT_HEIGHT = 56.29f;   // Độ cao nhấc chân tối đa
    const float X_RETRACTION = 19.06f;  // Mức thu chân vào khi nhấc
    
    if (t < 0.5f) {
        // Nửa đầu: Nhấc chân lên và di chuyển về phía trước
        float phase = t * 2.0f; // 0 -> 1
        
        // Quỹ đạo hình sin cho Z (nhấc lên)
        float liftFactor = sin(phase * M_PI);
        
        // X co vào khi nhấc lên cao nhất, sau đó duỗi ra
        float xFactor = sin(phase * M_PI);
        
        // Y di chuyển tiến tới mục tiêu
        float yFactor = phase;
        
        x = HOME_X - X_RETRACTION * xFactor;
        y = HOME_Y - STRIDE_LENGTH * yFactor;
        z = HOME_Z + LIFT_HEIGHT * liftFactor;
    } else {
        // Nửa sau: Hạ chân xuống và di chuyển thân robot
        float phase = (t - 0.5f) * 2.0f; // 0 -> 1
        
        // Chân đã ở phía trước, giờ kéo thân robot tiến lên
        x = HOME_X - X_RETRACTION * (1.0f - phase);
        y = HOME_Y - STRIDE_LENGTH * (1.0f - phase);
        z = HOME_Z; // Chân đã được hạ xuống
    }
}

// Bezier bậc 3 (cubic)
float bezier(float t, float p0, float p1, float p2, float p3) {
    float t2 = t * t;
    float t3 = t2 * t;
    return p0 * (1 - 3*t + 3*t2 - t3) + 
           p1 * (3*t - 6*t2 + 3*t3) + 
           p2 * (3*t2 - 3*t3) + 
           p3 * t3;
}

void createSmoothTrajectory(float t, float& x, float& y, float& z) {
    // Điểm kiểm soát cho đường cong
    const float HOME_X = 198.5f;
    const float HOME_Y = 0.0f;
    const float HOME_Z = -50.1f;
    
    // Điểm kiểm soát cho X (co vào khi nhấc)
    float x0 = HOME_X;         // Bắt đầu
    float x1 = HOME_X - 5.0f; // Điểm kiểm soát 1
    float x2 = HOME_X - 10.0f; // Điểm kiểm soát 2
    float x3 = HOME_X;         // Kết thúc (về vị trí ban đầu)
    
    // Điểm kiểm soát cho Y (di chuyển về phía trước)
    float y0 = HOME_Y;            // Bắt đầu
    float y1 = HOME_Y - 20.0f;    // Điểm kiểm soát 1
    float y2 = HOME_Y - 50.0f;    // Điểm kiểm soát 2
    float y3 = HOME_Y - 83.68f;    // Kết thúc (về phía trước)
    
    // Điểm kiểm soát cho Z (nhấc lên và hạ xuống)
    float z0 = HOME_Z;              // Bắt đầu
    float z1 = HOME_Z + 40.0f;      // Điểm kiểm soát 1 (hướng lên)
    float z2 = HOME_Z + 80.0f;      // Điểm kiểm soát 2 (cao nhất)
    float z3 = HOME_Z;              // Kết thúc (hạ xuống)

    // Tính tọa độ X, Y, Z trên đường cong Bezier
    if (t < 0.5f) {
        // Pha nhấc chân (nửa đầu tiên)
        float phase = t * 2.0f; // Chuẩn hóa 0->1
        x = bezier(phase, x0, x1, x2, x3);
        y = bezier(phase, y0, y1, y2, y3);
        z = bezier(phase, z0, z1, z2, z3);
    } else {
        // Pha kéo thân (nửa sau)
        float phase = (t - 0.5f) * 2.0f; // Chuẩn hóa 0->1
        x = HOME_X;
        y = HOME_Y - 70.0f + phase * 70.0f;
        z = HOME_Z;
    }
}

void createBioinspiredTrajectory(float t, float& x, float& y, float& z) {
    const float HOME_X = 198.5f;
    const float HOME_Y = 0.0f;
    const float HOME_Z = -50.1f;
    
    const float STRIDE_LENGTH = 70.0f;
    const float LIFT_HEIGHT = 56.29f;
    
    if (t < 0.3f) {
        // Pha 1: Nhấc nhanh (30% chu kỳ)
        float phase = t / 0.3f;
        
        // Nhấc chân theo đường cong: co vào và lên
        x = HOME_X - 20.0f * sin(phase * M_PI_2);
        z = HOME_Z + LIFT_HEIGHT * sin(phase * M_PI_2);
        y = HOME_Y - STRIDE_LENGTH * 0.3f * phase; // Di chuyển nhẹ về phía trước
    }
    else if (t < 0.5f) {
        // Pha 2: Di chuyển về phía trước (20% chu kỳ)
        float phase = (t - 0.3f) / 0.2f;
        
        // Giữ chân ở độ cao tối đa, đưa về phía trước
        x = HOME_X - 20.0f * cos(phase * M_PI_2);

        z = HOME_Z + LIFT_HEIGHT;
        y = HOME_Y - STRIDE_LENGTH * (0.3f + 0.7f * phase);
    }
    else if (t < 0.6f) {
        // Pha 3: Hạ xuống (10% chu kỳ)
        float phase = (t - 0.5f) / 0.1f;
        
        // Hạ chân xuống nhanh
        x = HOME_X;
        z = HOME_Z + LIFT_HEIGHT * (1.0f - phase);
        y = HOME_Y - STRIDE_LENGTH;
    }
    else {
        // Pha 4: Kéo thân (40% chu kỳ)
        float phase = (t - 0.6f) / 0.4f;
        
        // Chân đã tiếp đất, kéo thân robot về phía trước
        x = HOME_X;
        z = HOME_Z;
        y = HOME_Y - STRIDE_LENGTH * (1.0f - phase);
    }
}
