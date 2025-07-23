#pragma once

#include "types.hpp"
#include "config.hpp"
#include "kinematics.hpp"
#include "servo_controller.hpp"

namespace hexapod {

/**
 * Represents a single leg of the hexapod with its servos and kinematics
 */
class Leg {
public:
    /**
     * Constructor for a leg
     * 
     * @param leg_id ID of the leg (0-5)
     * @param pca_id ID of the PCA9685 board this leg is connected to
     * @param coxa_channel Servo channel for coxa joint
     * @param femur_channel Servo channel for femur joint
     * @param tibia_channel Servo channel for tibia joint
     * @param is_right Whether this is a right side leg
     */
    Leg(int leg_id, int pca_id, int coxa_channel, int femur_channel, int tibia_channel, bool is_right);

    // Allow copying and moving
    Leg() = default;
    Leg(const Leg&) = default;
    Leg& operator=(const Leg&) = default;
    Leg(Leg&&) = default;
    Leg& operator=(Leg&&) = default;

    /**
     * Set the target position for this leg
     * 
     * @param position Target position in leg's coordinate system
     * @param elbow_up Use elbow up configuration (default: false)
     * @return True if the position is reachable, false otherwise
     */
    bool setTargetPosition(const Point3D& position, bool elbow_up = false);
    
    /**
     * Update servo positions based on target position
     */
    void update();
    
    /**
     * Get current servo angles
     * 
     * @return Current servo angles
     */
    ServoAngles getServoAngles() const;
    
    /**
     * Get current target position
     * 
     * @return Current target position
     */
    Point3D getTargetPosition() const {
        return target_position;
    }
    
    /**
     * Set servo angles directly
     * 
     * @param isSetHome Whether this is setting the home position
     * @param servoAngles The servo angles to set
     */
    void setServoAngles(bool isSetHome, const ServoAngles& servoAngles);
    
    /**
     * Get current IK angles
     * 
     * @return Current IK angles
     */
    ServoAngles getCurrentAngles() const {
        return current_angles;
    }
    
    /**
     * Check if the inverse kinematics solution is valid
     * 
     * @return True if the solution is valid, false otherwise
     */
    bool isIKValid() const;
    
    // Getters for servo channels and PCA ID
    int getCoxaChannel() const { return coxa_channel; }
    int getFemurChannel() const { return femur_channel; }
    int getTibiaChannel() const { return tibia_channel; }
    int getPCA_ID() const { return pca_id; }
    bool isRightLeg() const { return is_right; }

private:
    int leg_id;
    int pca_id;
    int coxa_channel;
    int femur_channel;
    int tibia_channel;
    bool is_right;

    Point3D target_position;
    ServoAngles current_angles;
    ServoAngles servo_angles;  // Actual servo angles after conversion
};

} // namespace hexapod
