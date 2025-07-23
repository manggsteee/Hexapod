#pragma once

#include "types.hpp"
#include <cmath>

namespace hexapod {
namespace utils {

/**
 * Convert degrees to radians
 * 
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline float degToRad(float degrees) {
    return degrees * (M_PI / 180.0f);
}

/**
 * Convert radians to degrees
 * 
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline float radToDeg(float radians) {
    return radians * (180.0f / M_PI);
}

/**
 * Constrain a value between minimum and maximum
 * Renamed to avoid conflict with Arduino's constrain
 * 
 * @param value Value to constrain
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @return Constrained value
 */
inline float limit(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * Linear interpolation between two values
 * 
 * @param a First value
 * @param b Second value
 * @param t Interpolation factor (0.0 - 1.0)
 * @return Interpolated value
 */
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

/**
 * Map a value from one range to another
 * 
 * @param x Value to map
 * @param in_min Input range minimum
 * @param in_max Input range maximum
 * @param out_min Output range minimum
 * @param out_max Output range maximum
 * @return Mapped value
 */
inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Calculate distance between two points
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Distance between points
 */
inline float distance(const Point3D& p1, const Point3D& p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

} // namespace utils
} // namespace hexapod
