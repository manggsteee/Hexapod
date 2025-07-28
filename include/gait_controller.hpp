#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include "leg.hpp"


class GaitController {
public:
// Hàm điều khiển chuyển động tiến/lùi cho tripod
static void moveDirection(Leg legs[], int tripod[], float targetY, int stepDelay = 200);

// Hàm điều khiển chuyển động quay (rẽ trái/phải)
static void turnRobot(Leg legs[], int tripod1[], int tripod2[], float turnFactor, int stepDelay = 200, bool useInterpolation = true);
private:
    // Hàm khởi tạo
    GaitController();
};

#endif // GAIT_CONTROLLER_HPP
