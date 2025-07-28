#include <Arduino.h>
#include "hexapod.hpp"
#include "servo_controller.hpp"
#include "leg.hpp"
#include "ble_controller.hpp"
#include "trajectory.hpp"
#include "gait_controller.hpp"

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
void showTripodAngles(Leg legs[], const char *phase);
void testTripodGait();
void handlePosition();
void testServoAngles();
void chooseLeg();
void moveForward(Leg legs[], int tripod1[], int tripod2[]);
void handleBLECommand(char input);

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

    Serial.println("Starting tripod gait sequence with smooth trajectories...");

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
                // Sử dụng quỹ đạo Bezier với giá trị mặc định (tiến)
                Trajectory::createLegTrajectory(t, x, y, z, -150.0f);
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
                Trajectory::createLegTrajectory(t, x, y, z, -150.0f);
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
            Kinematics::computeFKArray(currentLeg, 
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

void handleBLECommand(char input)
{
    Serial.print("BLE Command: ");
    Serial.println(input);

    switch (input)
    {
    case 'r':
        Serial.println("Lệnh: Về home position");
        hexapod.goHome();
        break;
    case 'm':
        Serial.println("Lệnh: Test tripod gait");
        testTripodGait();
        break;
    case 'h':
        Serial.println("Lệnh: Chọn chân để điều chỉnh");
        chooseLeg();
        break;
    default:
        Serial.println("Lệnh không hợp lệ! Nhấn 'h' để hiển thị trợ giúp.");
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    ble.init("Hexapod_BLE");
    ble.enableAutoConnect(true);
    ble.setCommandHandler(handleBLECommand);

    ServoController::initialize();
    delay(1000);

    hexapod.goHome();
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
            hexapod.goHome();
            isMoving = false;
        }

        // Di chuyển robot theo joystick
        hexapod.moveWithJoystick(joyX, joyY, false);
        lastJoystickUpdate = millis();
    }

    // Kiểm tra timeout: nếu không nhận được dữ liệu trong 500ms và robot đang di chuyển
    if (isMoving && (millis() - lastJoystickActivity > 300))
    {
        Serial.println("Joystick timeout, stopping movement");
        hexapod.goHome();
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
}

