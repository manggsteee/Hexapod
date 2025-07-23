#pragma once

#include "leg.hpp"
#include "config.hpp"
#include "types.hpp"
#include <array>
#include <vector>

namespace hexapod {

/**
 * Main Hexapod class representing the complete robot
 */
class Hexapod {
public:
    /**
     * Constructor - initializes servo controller and legs
     */
    Hexapod();
    
    /**
     * Set up all legs with their correct servo channels and parameters
     */
    void setupLegs();
    
    /**
     * Set target positions for all legs
     * 
     * @param positions Array of target positions for all legs
     * @return True if all positions are reachable, false otherwise
     */
    bool setTargetPositions(const std::array<Point3D, config::NUM_LEGS>& positions);
    
    /**
     * Update servo positions for all legs
     */
    void update();
    
    /**
     * Return all legs to home position
     */
    void goHome();
    
    /**
     * Get reference to a specific leg
     * 
     * @param leg_id ID of the leg (0-5)
     * @return Reference to the requested leg
     */
    Leg& getLeg(int leg_id);
    
    /**
     * Get reference to all legs
     * 
     * @return Reference to the vector of legs
     */
    const std::vector<Leg>& getLegs() const { return legs; }
    
private:
    std::vector<Leg> legs;
};

} // namespace hexapod