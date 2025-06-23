#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Constants
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50
#define ANGLE_INCREMENT 5

// PCA9685 drivers
extern Adafruit_PWMServoDriver pca1;
extern Adafruit_PWMServoDriver pca2;

// Leg servo configuration
struct LegServos {
  uint8_t pcaIndex;  // 0 for pca1, 1 for pca2
  uint8_t hip;
  uint8_t knee;
  uint8_t ankle;
};

// External declarations for leg configurations
extern const LegServos leg1;
extern const LegServos leg2;
extern const LegServos leg3;
extern const LegServos leg4;
extern const LegServos leg5;
extern const LegServos leg6;

// External declarations for home positions
extern const float leg1Home[3];
extern const float leg2Home[3];
extern const float leg3Home[3];
extern const float leg4Home[3];
extern const float leg5Home[3];
extern const float leg6Home[3];

// Function declarations
void setServoAngle(uint8_t pcaIndex, uint8_t servoIndex, float angle);
void controlLeg(const LegServos &leg, float hipAngle, float kneeAngle, float ankleAngle);
void homePosition();
void scanI2CDevices();

// Movement function declarations
void moveForward(float distance);
void moveBackward(float distance);
void moveLeft(float distance);
void moveRight(float distance);

#endif // HEXAPOD_H