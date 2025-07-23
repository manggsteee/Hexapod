#pragma once

#include "hexapod.hpp"
#include "config.hpp"
#include "types.hpp"
#include <array>

namespace hexapod {

/**
 * Controls the gait patterns and movement of the hexapod
 */
class GaitController {
public:
    /**
     * Operating mode of the hexapod
     */
    enum class Mode { STAND, WALK };
    
    /**
     * Walking direction
     */
    enum class Direction { FORWARD, BACKWARD, LEFT, RIGHT, ROTATE_LEFT, ROTATE_RIGHT };
    
    /**
     * Type of gait pattern
     */
    enum class GaitType { TRIPOD, WAVE, RIPPLE };

    /**
     * Constructor
     * 
     * @param type Gait type (default: TRIPOD)
     * @param step_height Height of leg lifting during steps (default: 30.0f)
     * @param step_length Length of each step (default: 50.0f)
     */
    explicit GaitController(GaitType type = GaitType::TRIPOD, 
                           float step_height = 30.0f, 
                           float step_length = 50.0f);
    
    /**
     * Attach hexapod to this controller
     * 
     * @param hexapod Pointer to hexapod instance
     */
    void attachHexapod(Hexapod* hexapod);
    
    /**
     * Set operating mode
     * 
     * @param mode New operating mode
     */
    void setMode(Mode mode);
    
    /**
     * Set movement direction
     * 
     * @param dir New direction
     */
    void setDirection(Direction dir);
    
    /**
     * Set gait speed
     * 
     * @param speed Speed factor (0.0 - 1.0)
     */
    void setSpeed(float speed);
    
    /**
     * Set gait type (pattern)
     * 
     * @param type The gait type to use (TRIPOD, WAVE, RIPPLE)
     */
    void setGaitType(GaitType type);
    
    /**
     * Update leg positions based on gait pattern and elapsed time
     * 
     * @param time_delta Time elapsed since last update (seconds)
     */
    void update(float time_delta);
    
    /**
     * Get calculated leg positions for current gait
     * 
     * @return Array of target positions for all legs
     */
    std::array<Point3D, config::NUM_LEGS> getTargetPositions() const;

private:
    GaitType gait_type;
    Mode current_mode;
    Direction current_direction;
    Hexapod* hexapod;
    float time;
    float step_height;
    float step_length;
    float speed_factor;
    std::array<Point3D, config::NUM_LEGS> positions;

    // Update methods for different gait patterns
    void updateTripod(float t);
    void updateWave(float t);
    void updateRipple(float t);
    
    // Generate trajectory for a leg based on phase
    Point3D generateTrajectory(float phase, const Point3D& home_pos, const Point3D& direction_vector);
};

} // namespace hexapod
