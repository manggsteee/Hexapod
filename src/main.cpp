#include <Arduino.h>
#include "hexapod.hpp"
#include "servo_controller.hpp"
#include "leg.hpp"
#include "ble_controller.hpp"

BLEController ble;

// === FUNCTION PROTOTYPES ===
void adjustAngle(int delta);
void setServo(int leg, int servoType, float angle);
String getServoName(int servoType);
void printCurrentStatus();
void printAllAngles();
void printHomeArrayFormat();
void applyAllAngles();
void resetAllTo90();
void nextLeg();
void nextServo();
void printHelp();
void showAngles();
void testLegForward();
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

// Thêm function này:
void testTripodGait()
{
    Serial.println("\n=== TRIPOD GAIT TEST ===");
    Serial.println("Using EXACT same coordinates as testLegForward() case '6'");
    Serial.println("Tripod 1: Legs 0, 2, 4");
    Serial.println("Tripod 2: Legs 1, 3, 5");
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

    Serial.println("🚀 Starting tripod gait sequence...");

    // Phase 1: All legs HOME
    Serial.println("\n=== PHASE 1: ALL LEGS HOME ===");
    for (int i = 0; i < 6; i++)
    {
        legs[i].setTargetPosition(225.0f, 0.0f, 0.0f);
        legs[i].update();
    }
    delay(1000);
    showTripodAngles(legs, "All legs home");

    // Walking cycles
    for (int cycle = 0; cycle < 3; cycle++)
    {
        Serial.print("\n=== WALKING CYCLE ");
        Serial.print(cycle + 1);
        Serial.println(" ===");

        // TRIPOD 1 WALKING SEQUENCE (using exact coordinates from case '6')
        Serial.println("TRIPOD 1 (0,2,4) WALKING:");
        for (int i = 0; i < 3; i++)
        {
            legs[tripod1[i]].setTargetPosition(225.0f, 0.0f, 250.0f); // SAME as case '6' step 1
            legs[tripod1[i]].update();
        }
        delay(800);

        // Step 2: Tripod 1 FORWARD
        Serial.println("  Step 2: Tripod 1 FORWARD");
        for (int i = 0; i < 3; i++)
        {
            legs[tripod1[i]].setTargetPosition(225.0f, -150.0f, 250.0f); // SAME as case '6' step 2
            legs[tripod1[i]].update();
        }
        delay(800);

        // Step 3: Tripod 1 DOWN
        Serial.println("  Step 3: Tripod 1 DOWN");
        for (int i = 0; i < 3; i++)
        {
            legs[tripod1[i]].setTargetPosition(225.0f, -150.0f, 0.0f); // SAME as case '6' step 3
            legs[tripod1[i]].update();
        }
        delay(800);

        // Step 4: Tripod 1 BACKWARD (body moves forward)
        Serial.println("  Step 4: Tripod 1 BACKWARD");
        for (int i = 0; i < 3; i++)
        {
            legs[tripod1[i]].setTargetPosition(225.0f, 0.0f, 0.0f); // SAME as case '6' step 4
            legs[tripod1[i]].update();
        }
        // Step 1: Tripod 1 UP
        Serial.println("  Step 1: Tripod 1 UP");

        showTripodAngles(legs, "Tripod 1 backward");
        delay(800);

        // TRIPOD 2 WALKING SEQUENCE (same coordinates)
        Serial.println("TRIPOD 2 (1,3,5) WALKING:");

        // Step 1: Tripod 2 UP
        for (int i = 0; i < 3; i++)
        {
            legs[tripod2[i]].setTargetPosition(225.0f, 0.0f, 250.0f); // SAME as case '6' step 1
            legs[tripod2[i]].update();
        }
        delay(800);

        // Step 2: Tripod 2 FORWARD
        Serial.println("  Step 2: Tripod 2 FORWARD");
        for (int i = 0; i < 3; i++)
        {
            legs[tripod2[i]].setTargetPosition(225.0f, -150.0f, 250.0f); // SAME as case '6' step 2
            legs[tripod2[i]].update();
        }
        delay(800);

        // Step 3: Tripod 2 DOWN
        Serial.println("  Step 3: Tripod 2 DOWN");
        for (int i = 0; i < 3; i++)
        {
            legs[tripod2[i]].setTargetPosition(225.0f, -150.0f, 0.0f); // SAME as case '6' step 3
            legs[tripod2[i]].update();
        }
        delay(800);

        // Step 4: Tripod 2 BACKWARD (body moves forward)
        Serial.println("  Step 4: Tripod 2 BACKWARD");
        for (int i = 0; i < 3; i++)
        {
            legs[tripod2[i]].setTargetPosition(225.0f, 150.0f, 0.0f); // SAME as case '6' step 4
            legs[tripod2[i]].update();
        }
        delay(800);

        // Return both tripods to home
        Serial.println("  Return to HOME");
        for (int i = 0; i < 6; i++)
        {
            legs[i].setTargetPosition(225.0f, 0.0f, 0.0f);
            legs[i].update();
        }
        showTripodAngles(legs, "Both tripods home");

        Serial.print("✅ Cycle ");
        Serial.print(cycle + 1);
        Serial.println(" completed!");

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

    Serial.println("\n🎉 TRIPOD GAIT TEST COMPLETED! 🎉");
}

Hexapod hexapod;
Leg testLeg(5, 1, 10, 9, 8); // Test leg 1: coxa=ch6, femur=ch5, tibia=ch4

// Biến lưu góc hiện tại cho test
int currentLeg = 0;
int currentServo = 0; // 0=coxa, 1=femur, 2=tibia
float currentAngles[6][3] = {
    {90, 45, 135}, // Leg 0 - Coxa thẳng, Femur hạ, Tibia gấp
    {90, 45, 135}, // Leg 1
    {90, 45, 135}, // Leg 2
    {90, 45, 135}, // Leg 3
    {90, 45, 135}, // Leg 4
    {90, 45, 135}  // Leg 5
};

void setup()
{
    Serial.begin(115200);
    ble.init("Hexapod_BLE");
    ble.setCommandHandler(handleBLECommand);
    Serial.println("✅ Hexapod + BLE ready");
    Serial.println("The device started, now you can pair it with bluetooth!");
    Serial.println("=== HEXAPOD HOME ANGLE FINDER ===");
    printHelp();

    ServoController::initialize();
    delay(1000);

    // Set all servos to 90° initially
    hexapod.goHome();

    printCurrentStatus();

    Leg legs[6] = {
        Leg(0, 0, 2, 1, 0),  // Leg 0
        Leg(1, 0, 6, 5, 4),  // Leg 1
        Leg(2, 0, 10, 9, 8), // Leg 2
    };

    int tripod1[] = {0, 2, 4};
     for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, 0.0f, 250.0f); // SAME as case '6' step 1
        legs[tripod1[i]].update();
    }
}

void handleBLECommand(char input)
{
    Serial.print("BLE Command: ");
    Serial.println(input);

    switch (input)
    {
    case 'q':
        nextLeg();
        break;
    case 'w':
        nextServo();
        break;
    case 'a':
        adjustAngle(-5);
        break;
    case 's':
        adjustAngle(-1);
        break;
    case 'd':
        adjustAngle(1);
        break;
    case 'f':
        adjustAngle(5);
        break;
    case 'z':
        currentAngles[currentLeg][currentServo] = 90;
        setServo(currentLeg, currentServo, 90);
        printCurrentStatus();
        break;
    case 'x':
        printAllAngles();
        break;
    case 'c':
        printHomeArrayFormat();
        break;
    case 'v':
        applyAllAngles();
        break;
    case 'b':
        resetAllTo90();
        break;
    case 'h':
        printHelp();
        break;
    case 'r':
        hexapod.goHome();
        break;
    case 'j':
        testLegForward();
        break;
    case 'm':
        testTripodGait();
        break;
    default:
        Serial.println("Unknown command! Press 'h' for help.");
        break;
    }
}

void moveForward(Leg legs[], int tripod1[], int tripod2[])
{
   // Step 2: Tripod 1 FORWARD
    Serial.println("  Step 2: Tripod 1 FORWARD");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, -150.0f, 250.0f); // SAME as case '6' step 2
        legs[tripod1[i]].update();
    }
    delay(800);

    // Step 3: Tripod 1 DOWN
    Serial.println("  Step 3: Tripod 1 DOWN");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, -150.0f, 0.0f); // SAME as case '6' step 3
        legs[tripod1[i]].update();
    }
    delay(800);

    for (int i = 0; i < 3; i++)
    {
        legs[tripod2[i]].setTargetPosition(225.0f, 0.0f, 250.0f); // SAME as case '6' step 3
        legs[tripod2[i]].update();
    }
    delay(800);

    // Step 4: Tripod 1 BACKWARD (body moves forward)
    Serial.println("  Step 4: Tripod 1 BACKWARD");
    for (int i = 0; i < 3; i++)
    {
        legs[tripod1[i]].setTargetPosition(225.0f, 0.0f, 0.0f); // SAME as case '6' step 4
        legs[tripod1[i]].update();
    }
}

void loop()
{
    // ble.loop();
    // if (Serial.available())
    // {
    //     char input = Serial.read();
    //     handleBLECommand(input);
    // }
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
    moveForward(legs, tripod1, tripod2);
    delay(800);
    moveForward(legs, tripod2, tripod1);
    delay(800);
}

void nextLeg()
{
    currentLeg = (currentLeg + 1) % 6;
    Serial.print("Switched to leg ");
    Serial.println(currentLeg);
    printCurrentStatus();
}

void nextServo()
{
    currentServo = (currentServo + 1) % 3;
    Serial.print("Switched to servo ");
    Serial.print(currentServo);
    Serial.print(" (");
    Serial.print(getServoName(currentServo));
    Serial.println(")");
    printCurrentStatus();
}

void printHelp()
{
    Serial.println("\n=== SINGLE KEY COMMANDS ===");
    Serial.println("Navigation:");
    Serial.println("  q - Next leg (0→1→2→3→4→5→0...)");
    Serial.println("  w - Next servo (Coxa→Femur→Tibia→Coxa...)");
    Serial.println("");
    Serial.println("Angle adjustment:");
    Serial.println("  a - Decrease angle by 5°");
    Serial.println("  s - Decrease angle by 1°");
    Serial.println("  d - Increase angle by 1°");
    Serial.println("  f - Increase angle by 5°");
    Serial.println("");
    Serial.println("Actions:");
    Serial.println("  z - Reset current servo to 90°");
    Serial.println("  x - Print all current angles");
    Serial.println("  c - Copy home array format");
    Serial.println("  v - Verify (apply all angles)");
    Serial.println("  b - Reset all servos to 90°");
    Serial.println("  h - Show this help");
    Serial.println("");
    Serial.println("TIP: Layout like WASD + nearby keys for easy use!");
    Serial.println("=====================================");
}

void adjustAngle(int delta)
{
    currentAngles[currentLeg][currentServo] += delta;

    // Clamp to 0-180
    if (currentAngles[currentLeg][currentServo] < 0)
    {
        currentAngles[currentLeg][currentServo] = 0;
    }
    if (currentAngles[currentLeg][currentServo] > 180)
    {
        currentAngles[currentLeg][currentServo] = 180;
    }

    // Apply to servo
    setServo(currentLeg, currentServo, currentAngles[currentLeg][currentServo]);

    printCurrentStatus();
}

void setServo(int leg, int servoType, float angle)
{
    const int LEG_SERVO_CHANNELS[6][4] = {
        {0, 2, 1, 0},  // Leg 0
        {0, 6, 5, 4},  // Leg 1
        {0, 10, 9, 8}, // Leg 2
        {1, 2, 1, 0},  // Leg 3
        {1, 6, 5, 4},  // Leg 4
        {1, 10, 9, 8}, // Leg 5
    };

    int pca_id = LEG_SERVO_CHANNELS[leg][0];
    int channel;
    switch (servoType)
    {
    case 0: // Coxa (Hip)
        channel = LEG_SERVO_CHANNELS[leg][1];
        break;
    case 1: // Femur (Knee)
        channel = LEG_SERVO_CHANNELS[leg][2];
        break;
    case 2: // Tibia (Ankle)
        channel = LEG_SERVO_CHANNELS[leg][3];
        break;
    default:
        Serial.println("Error: Invalid servo type!");
        return;
    }

    ServoController::setAngle(pca_id, channel, angle);

    Serial.print("→ Set Leg ");
    Serial.print(leg);
    Serial.print(" ");
    Serial.print(getServoName(servoType));
    Serial.print(" to ");
    Serial.print(angle);
    Serial.println("°");
}

String getServoName(int servoType)
{
    switch (servoType)
    {
    case 0:
        return "Coxa";
    case 1:
        return "Femur";
    case 2:
        return "Tibia";
    default:
        return "Unknown";
    }
}

void printCurrentStatus()
{
    Serial.print("Current: Leg ");
    Serial.print(currentLeg);
    Serial.print(", ");
    Serial.print(getServoName(currentServo));
    Serial.print(": ");
    Serial.print(currentAngles[currentLeg][currentServo]);
    Serial.println("°");
}

void printAllAngles()
{
    Serial.println("\n=== CURRENT ANGLES ===");
    for (int leg = 0; leg < 6; leg++)
    {
        Serial.print("Leg ");
        Serial.print(leg);
        Serial.print(": Coxa=");
        Serial.print(currentAngles[leg][0]);
        Serial.print("° Femur=");
        Serial.print(currentAngles[leg][1]);
        Serial.print("° Tibia=");
        Serial.print(currentAngles[leg][2]);
        Serial.println("°");
    }
    Serial.println();
}

void printHomeArrayFormat()
{
    Serial.println("\n=== HOME ANGLES ARRAY FORMAT ===");
    Serial.println("const float homeAngles[6][3] = {");
    for (int leg = 0; leg < 6; leg++)
    {
        Serial.print("    {");
        Serial.print(currentAngles[leg][0]);
        Serial.print("f, ");
        Serial.print(currentAngles[leg][1]);
        Serial.print("f, ");
        Serial.print(currentAngles[leg][2]);
        Serial.print("f}");
        if (leg < 5)
            Serial.print(",");
        Serial.print("  // Leg ");
        Serial.println(leg);
    }
    Serial.println("};");
    Serial.println();
}

void applyAllAngles()
{
    Serial.println("Applying all angles...");
    for (int leg = 0; leg < 6; leg++)
    {
        for (int servo = 0; servo < 3; servo++)
        {
            setServo(leg, servo, currentAngles[leg][servo]);
            delay(100);
        }
    }
    Serial.println("All angles applied!");
}

void resetAllTo90()
{
    Serial.println("Resetting all servos to 90°...");
    for (int leg = 0; leg < 6; leg++)
    {
        for (int servo = 0; servo < 3; servo++)
        {
            currentAngles[leg][servo] = 90;
            setServo(leg, servo, 90);
            delay(100);
        }
    }
    Serial.println("Reset completed!");
    printCurrentStatus();
}

void testLegForward()
{
    Serial.println("\n=== TESTING LEG 0 FORWARD ===");
    Serial.println("→ HOME position");
    testLeg.setTargetPosition(225.0f, 0.0f, 0.0f);
    testLeg.update();
    showAngles();
}

void showAngles()
{
    auto angles = testLeg.getServoAngles();
    Serial.print("IK Angles: Coxa=");
    Serial.print(angles[0]);
    Serial.print("° Femur=");
    Serial.print(angles[1]);
    Serial.print("° Tibia=");
    Serial.print(angles[2]);
    Serial.println("°");
}
