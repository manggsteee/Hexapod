#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <array>
#include <cstdint>
#include <cstddef>
#include <cmath>

struct Vec3 {
    float x, y, z;
};


class Kinematics {
public:
    static constexpr size_t NUM_LEGS = 6;

    // Forward Kinematics
    static void computeFK(int legId, float coxaAngle, float femurAngle, float tibiaAngle, float& x, float& y, float& z);
    static void computeFKArray(int legId, float coxaAngle, float femurAngle, float tibiaAngle);

    // Inverse Kinematics
    static bool computeIK(int leg_id, float x, float y, float z, float& coxa_angle, float& femur_angle, float& tibia_angle, bool elbow_up = false);
};

#endif // KINEMATICS_HPP