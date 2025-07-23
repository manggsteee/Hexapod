#pragma once

#include <cstdint>
#include "types.hpp"

namespace hexapod {

/**
 * Controller class for servo motors using PCA9685 drivers
 */
class ServoController {
public:
    /**
     * Initialize I2C communication and PCA9685 drivers
     */
    static void initialize();
    
    /**
     * Set angle for a specific servo
     * 
     * @param pca_id ID of the PCA9685 board
     * @param channel Servo channel on the PCA9685 board
     * @param angle Angle to set (0-180 degrees)
     */
    static void setAngle(int pca_id, int channel, float angle);
    
    /**
     * Set multiple servo angles at once for a leg
     * 
     * @param pca_id ID of the PCA9685 board
     * @param coxa_channel Coxa servo channel
     * @param femur_channel Femur servo channel
     * @param tibia_channel Tibia servo channel
     * @param angles ServoAngles containing angles for coxa, femur and tibia
     */
    static void setLegAngles(int pca_id, int coxa_channel, int femur_channel, 
                             int tibia_channel, const ServoAngles& angles);

private:
    static bool initialized;
};

} // namespace hexapod