#pragma once

#include <Arduino.h>

namespace hexapod {
namespace config {

// Số lượng chân
constexpr int NUM_LEGS = 6;

// Thông số vật lý robot
constexpr float COXA_LENGTH = 65.0f;
constexpr float FEMUR_LENGTH = 65.0f;
constexpr float TIBIA_LENGTH = 95.0f;

// Thông số vị trí home
constexpr float HOME_X = 198.5f;
constexpr float HOME_Y = 0.0f;
constexpr float HOME_Z = -50.1f;

// Thông số di chuyển
constexpr float STEP_HEIGHT = 56.29f;
constexpr float STRIDE_LENGTH = 70.0f;
constexpr float BASE_X = 198.5f;

// Timing constants
constexpr int UPDATE_INTERVAL = 15;
constexpr int DEFAULT_STEP_DELAY = 400;

// Servo offsets và hướng
constexpr float COXA_OFFSET = 90.0f;
constexpr float FEMUR_OFFSET = 90.0f;
constexpr float TIBIA_OFFSET = 90.0f;

// Home corrections - Điều chỉnh góc servo khi ở vị trí home
constexpr float HOME_CORRECTION[6][3] = {
    {0.0f, 8.0f, -2.0f},     // Leg 0: 0°, +8°, -2°
    {-2.0f, 3.0f, 3.0f},     // Leg 1: -2°, +3°, +3°
    {-5.0f, -4.0f, 12.0f},   // Leg 2: -5°, -4°, +12°
    {8.0f, -10.0f, 7.0f},    // Leg 3: 8°, -10°, +7°
    {-6.0f, 3.0f, -10.0f},   // Leg 4: -6°, +3°, -10°
    {-0.02f, 5.22f, -5.20f}  // Leg 5: -0.02°, +5.22°, -5.20°
};

// Tripod patterns
constexpr int TRIPOD1[] = {0, 2, 4}; // Right front, right back, left middle
constexpr int TRIPOD2[] = {1, 3, 5}; // Right middle, left front, left back

} // namespace config
} // namespace hexapod
