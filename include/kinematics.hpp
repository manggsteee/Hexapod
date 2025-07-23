#pragma once

#include "types.hpp"
#include "config.hpp"

namespace hexapod {
namespace kinematics {

/**
 * Convert IK angles to servo angles
 * @param angles IK angles (coxa, femur, tibia)
 * @param isRight indicates if this is for a right leg (true) or left leg (false)
 * @return angles converted to servo angles with proper offset and direction
 */
ServoAngles ikToServoAngles(const ServoAngles& angles, bool isRight);

/**
 * Convert servo angles to IK angles
 * @param servoAngles servo angles (coxa, femur, tibia)
 * @param isRight indicates if this is for a right leg (true) or left leg (false)
 * @return servo angles converted to IK angles
 */
ServoAngles servoToIkAngles(const ServoAngles& servoAngles, bool isRight);

/**
 * Calculate inverse kinematics
 * @param target target position in leg coordinate system (x, y, z)
 * @param result calculated servo angles
 * @param elbow_up use elbow up configuration (true) or elbow down (false)
 * @return true if target position is reachable, false otherwise
 */
bool computeIK(const Point3D& target, ServoAngles& result, bool elbow_up = true);

/**
 * Calculate forward kinematics
 * @param angles IK angles (coxa, femur, tibia)
 * @return calculated end effector position
 */
Point3D computeFK(const ServoAngles& angles);

/**
 * Compute forward kinematics from servo angles for a specific leg
 * Used for debug purposes
 * 
 * @param legId ID of the leg (0-5)
 * @param coxaAngle Coxa servo angle
 * @param femurAngle Femur servo angle
 * @param tibiaAngle Tibia servo angle
 * @param x Output x coordinate
 * @param y Output y coordinate
 * @param z Output z coordinate
 */
void computeLegFK(int legId, float coxaAngle, float femurAngle, float tibiaAngle, 
                float& x, float& y, float& z);

/**
 * Compute forward kinematics and print the result
 * Used for debug purposes
 * 
 * @param legId ID of the leg (0-5)
 * @param coxaAngle Coxa servo angle
 * @param femurAngle Femur servo angle
 * @param tibiaAngle Tibia servo angle
 */
void computeLegFKArray(int legId, float coxaAngle, float femurAngle, float tibiaAngle);

} // namespace kinematics
} // namespace hexapod