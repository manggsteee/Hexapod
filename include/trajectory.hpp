#pragma once

#include "types.hpp"

namespace hexapod {
namespace trajectory {

/**
 * Generate a point along a linear trajectory
 * 
 * @param start Start position
 * @param end End position
 * @param t Interpolation factor (0.0 - 1.0)
 * @return Interpolated position
 */
Point3D linear(const Point3D& start, const Point3D& end, float t);

/**
 * Generate a point along a Bezier curve trajectory
 * 
 * @param start Start position
 * @param end End position
 * @param height Maximum height of the curve
 * @param t Interpolation factor (0.0 - 1.0)
 * @return Point on the curve
 */
Point3D bezier(const Point3D& start, const Point3D& end, float height, float t);

/**
 * Generate a point along a trajectory with arc for leg lifting
 * 
 * @param start Start position
 * @param end End position
 * @param height Maximum height of the arc
 * @param t Interpolation factor (0.0 - 1.0, 0.0-0.5 is lifting, 0.5-1.0 is lowering)
 * @return Point on the trajectory
 */
Point3D arcTrajectory(const Point3D& start, const Point3D& end, float height, float t);

/**
 * Generate a sinusoidal trajectory for smooth leg movements
 * 
 * @param start Start position
 * @param end End position
 * @param height Maximum height of the sine wave
 * @param t Interpolation factor (0.0 - 1.0)
 * @return Point on the sinusoidal trajectory
 */
Point3D sineTrajectory(const Point3D& start, const Point3D& end, float height, float t);

/**
 * Bezier bậc 3 (cubic) cho mỗi thành phần x,y,z
 * 
 * @param t Interpolation factor (0.0 - 1.0)
 * @param p0 Control point 0
 * @param p1 Control point 1
 * @param p2 Control point 2
 * @param p3 Control point 3
 * @return Interpolated value
 */
float bezierCubic(float t, float p0, float p1, float p2, float p3);

/**
 * Di chuyển chân theo đường cong cơ bản
 * 
 * @param t Interpolation factor (0.0 - 1.0)
 * @param x Output x coordinate
 * @param y Output y coordinate
 * @param z Output z coordinate
 */
void createLegTrajectory(float t, float& x, float& y, float& z);

/**
 * Di chuyển chân theo đường cong mượt mà
 * 
 * @param t Interpolation factor (0.0 - 1.0)
 * @param x Output x coordinate
 * @param y Output y coordinate
 * @param z Output z coordinate
 */
void createSmoothTrajectory(float t, float& x, float& y, float& z);

/**
 * Di chuyển chân với quỹ đạo sinh học tự nhiên
 * 
 * @param t Interpolation factor (0.0 - 1.0)
 * @param x Output x coordinate
 * @param y Output y coordinate
 * @param z Output z coordinate
 */
void createBioinspiredTrajectory(float t, float& x, float& y, float& z);

} // namespace trajectory
} // namespace hexapod
