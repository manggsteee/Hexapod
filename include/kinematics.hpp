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

    Kinematics(float coxa_length, float femur_length, float tibia_length);
    std::array<float, 3> computeIK(const Vec3& foot_pos) const;
    std::array<std::array<float, 3>, NUM_LEGS> computeAllIK(const std::array<Vec3, NUM_LEGS>& target_positions) const;

private:
    float l1, l2, l3;
};

#endif // KINEMATICS_HPP