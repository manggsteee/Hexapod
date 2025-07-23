#include "gait_controller.hpp"
#include <cmath>

namespace hexapod {

GaitController::GaitController(GaitType type, float step_height, float step_length)
    : gait_type(type)
    , current_mode(Mode::STAND)
    , current_direction(Direction::FORWARD)
    , hexapod(nullptr)
    , time(0.0f)
    , step_height(step_height)
    , step_length(step_length)
    , speed_factor(1.0f) {
    positions.fill(Point3D{0, 0, 0});
}

void GaitController::attachHexapod(Hexapod* hexapod) {
    this->hexapod = hexapod;
}

void GaitController::setMode(Mode mode) {
    current_mode = mode;
}

void GaitController::setDirection(Direction dir) {
    current_direction = dir;
}

void GaitController::setSpeed(float speed) {
    speed_factor = speed;
}

void GaitController::setGaitType(GaitType type) {
    gait_type = type;
}

void GaitController::update(float time_delta) {
    if (!hexapod || current_mode == Mode::STAND) {
        positions.fill(Point3D{0, 0, 0});
        return;
    }

    time += time_delta;
    
    switch (gait_type) {
        case GaitType::TRIPOD:
            updateTripod(time);
            break;
        case GaitType::WAVE:
            updateWave(time);
            break;
        case GaitType::RIPPLE:
            updateRipple(time);
            break;
    }
}

std::array<Point3D, config::NUM_LEGS> GaitController::getTargetPositions() const {
    return positions;
}

void GaitController::updateTripod(float t) {
    
    float phase[6] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
    for (size_t i = 0; i < positions.size(); ++i) {
        float local_t = fmod(t + phase[i], 1.0f);
        if (local_t < 0.5f) {
            // Convert all expressions to float explicitly
            float x = step_length * (local_t - 0.25f) * 4.0f;
            float z = -step_height * std::sin(local_t * M_PI);
            positions[i] = {x, 0.0f, z};
        } else {
            float x = step_length * (local_t - 0.75f) * 4.0f;
            positions[i] = {x, 0.0f, 0.0f};
        }
    }
}

void GaitController::updateWave(float t) {
    float period = 1.2f;
    
    for (size_t i = 0; i < positions.size(); ++i) {
        float phase = fmod(t + i * (period / 6.0f), period) / period;
        if (phase < 0.5f) {
            float x = step_length * (phase - 0.25f) * 4.0f;
            float z = -step_height * std::sin(phase * M_PI);
            positions[i] = {x, 0.0f, z};
        } else {
            float x = step_length * (phase - 0.75f) * 4.0f;
            positions[i] = {x, 0.0f, 0.0f};
        }
    }
}

void GaitController::updateRipple(float t) {
    // Simple placeholder implementation
    // In a real implementation, you would generate a ripple gait pattern
    for (size_t i = 0; i < positions.size(); ++i) {
        float phase = fmod(t + i * 0.167f, 1.0f);
        if (phase < 0.5f) {
            float x = step_length * (phase - 0.25f) * 4.0f;
            float z = -step_height * std::sin(phase * M_PI);
            positions[i] = {x, 0.0f, z};
        } else {
            float x = step_length * (phase - 0.75f) * 4.0f;
            positions[i] = {x, 0.0f, 0.0f};
        }
    }
}

Point3D GaitController::generateTrajectory(float phase, const Point3D& home_pos, const Point3D& direction_vector) {
    Point3D result = home_pos;
    
    if (phase < 0.5f) {
        // Swing phase (foot in air)
        float x = direction_vector.x * (phase - 0.25f) * 4.0f;
        float y = direction_vector.y * (phase - 0.25f) * 4.0f;
        float z = -step_height * std::sin(phase * M_PI);
        
        result.x += x;
        result.y += y;
        result.z += z;
    } else {
        // Stance phase (foot on ground)
        float x = direction_vector.x * (phase - 0.75f) * 4.0f;
        float y = direction_vector.y * (phase - 0.75f) * 4.0f;
        
        result.x += x;
        result.y += y;
    }
    
    return result;
}

} // namespace hexapod
