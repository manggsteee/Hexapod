#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <cstdint>

class Trajectory {
public:
    // Tính toán vị trí của chân dựa trên góc servo
    static void createLegTrajectory(float t, float& x, float& y, float& z, float targetY = -150.0f);
    static void computeFKArray(int legId, float coxaAngle, float femurAngle, float tibiaAngle);
    static float bezier(float t, float p0, float p1, float p2, float p3);
    // Tính toán quỹ đạo mượt mà
    static void createSmoothTrajectory(float t, float& x, float& y, float& z);
    static void createBioinspiredTrajectory(float t, float& x, float& y, float& z);
private:
    Trajectory(); // Constructor mặc định
};

#endif // TRAJECTORY_HPP