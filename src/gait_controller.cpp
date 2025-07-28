#include "gait_controller.hpp"
#include "leg.hpp"
#include "trajectory.hpp"
#include <Arduino.h>

GaitController::GaitController() {}

void GaitController::moveDirection(Leg legs[], int tripod[], float targetY, int stepDelay)
{
    const int UPDATE_INTERVAL = 15;
    const int STEPS = stepDelay / UPDATE_INTERVAL;
    
    // === TRIPOD MOVEMENT ===
    for (int step = 0; step < STEPS; step++) {
        float t = (float)step / STEPS;
        
        for (int i = 0; i < 3; i++) {
            float x, y, z;
            Trajectory::createLegTrajectory(t, x, y, z, targetY);
            legs[tripod[i]].setTargetPosition(x, y, z, t < 0.4f);
            legs[tripod[i]].update();
        }
        
        delay(UPDATE_INTERVAL);
    }
}

void GaitController::turnRobot(Leg legs[], int tripod1[], int tripod2[], float turnFactor, int stepDelay, bool useInterpolation) {
    const int UPDATE_INTERVAL = 10;
    const float TURN_STRIDE = 100.0f;
    const float STEP_HEIGHT = 270.0f;
    const float BASE_X = 198.5f;

    bool isTurningRight = (turnFactor > 0);
    float magnitude = min(abs(turnFactor), 1.0f);
    float offsetRight = isTurningRight ? -TURN_STRIDE * magnitude : TURN_STRIDE * magnitude;
    float offsetLeft = -offsetRight;

    // Step 1: Tripod 1 UP
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

    // Step 2: Tripod 1 ROTATE
    for (int i = 0; i < 3; i++) {
        int legId = tripod1[i];
        bool isRightLeg = (legId < 3);
        float yOffset = isRightLeg ? offsetRight : offsetLeft;
        legs[legId].setTargetPosition(BASE_X, yOffset, STEP_HEIGHT);
        legs[legId].update();
    }
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }

    // Step 3: Tripod 1 DOWN
    for (int i = 0; i < 3; i++) {
        int legId = tripod1[i];
        bool isRightLeg = (legId < 3);
        float yOffset = isRightLeg ? offsetRight : offsetLeft;
        legs[legId].setTargetPosition(BASE_X, yOffset, 0.0f);
        legs[legId].update();
    }
    if (useInterpolation) {
        for (int t = 0; t < stepDelay; t += UPDATE_INTERVAL) {
            delay(UPDATE_INTERVAL);
        }
    } else {
        delay(stepDelay);
    }

    // Step 4: Tripod 1 BACKWARD (body moves forward)
    for (int i = 0; i < 3; i++) {
        int legId = tripod1[i];
        legs[legId].setTargetPosition(BASE_X, 0.0f, 0.0f);
        legs[legId].update();
    }
}
