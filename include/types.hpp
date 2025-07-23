#pragma once

#include <Arduino.h>

namespace hexapod {

// 3D Point structure
struct Point3D {
    float x;
    float y;
    float z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

// Servo angles
struct ServoAngles {
    float coxa;
    float femur;
    float tibia;
    
    ServoAngles() : coxa(90), femur(90), tibia(90) {}
    ServoAngles(float _coxa, float _femur, float _tibia) 
        : coxa(_coxa), femur(_femur), tibia(_tibia) {}
};

// Structure for servo channel configuration
struct ServoChannels {
    int pca_id;    // PCA9685 board ID (0 or 1)
    int coxa;      // Channel for coxa servo
    int femur;     // Channel for femur servo  
    int tibia;     // Channel for tibia servo
    
    ServoChannels() : pca_id(0), coxa(0), femur(0), tibia(0) {}
    ServoChannels(int _pca_id, int _coxa, int _femur, int _tibia)
        : pca_id(_pca_id), coxa(_coxa), femur(_femur), tibia(_tibia) {}
};

} // namespace hexapod
