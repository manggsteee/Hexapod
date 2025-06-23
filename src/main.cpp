#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "hexapod.h"

// PCA9685 drivers
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);

// Current servo position tracking
uint8_t currentPca = 0;
uint8_t currentServo = 0;
float currentAngle = 90.0;

// Input states
enum InputState {
  SELECT_PCA,
  SELECT_SERVO,
  ADJUST_ANGLE
};

InputState inputState = SELECT_PCA;
String inputBuffer = "";

// Define servo indexes for each leg
// LegServos đã được định nghĩa trong hexapod.h, không cần định nghĩa lại
const LegServos leg1 = {0, 0, 1, 2};    // Leg 1 (PCA1)
const LegServos leg2 = {0, 4, 5, 6};    // Leg 2 (PCA1)
const LegServos leg3 = {0, 8, 9, 10};   // Leg 3 (PCA1)
const LegServos leg4 = {1, 0, 1, 2};    // Leg 4 (PCA2)
const LegServos leg5 = {1, 4, 5, 6};    // Leg 5 (PCA2)
const LegServos leg6 = {1, 8, 9, 10};   // Leg 6 (PCA2)

// Home position angles for each leg (from your provided values)
const float leg1Home[3] = {90, 125, 65};   // Hip, knee, ankle for leg1
const float leg2Home[3] = {80, 85, 55};    // Hip, knee, ankle for leg2
const float leg3Home[3] = {75, 60, 70};    // Hip, knee, ankle for leg3
const float leg4Home[3] = {95, 65, 65};    // Hip, knee, ankle for leg4
const float leg5Home[3] = {80, 20, 135};   // Hip, knee, ankle for leg5
const float leg6Home[3] = {55, 70, 15};    // Hip, knee, ankle for leg6

/**
 * @brief Sets a single servo to a specified angle
 */
void setServoAngle(uint8_t pcaIndex, uint8_t servoIndex, float angle) {
    // Constrain angle to valid range
    angle = constrain(angle, 0, 180);
    
    // Convert angle to pulse length
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    
    // Set the servo position on the appropriate PCA
    if (pcaIndex == 0) {
        pca1.setPWM(servoIndex, 0, pulse);
    } else {
        pca2.setPWM(servoIndex, 0, pulse);
    }
    
    Serial.print("PCA");
    Serial.print(pcaIndex + 1);
    Serial.print(" Servo ");
    Serial.print(servoIndex);
    Serial.print(": ");
    Serial.print(angle);
    Serial.println("°");
}

/**
 * @brief Controls a single leg
 */
void controlLeg(const LegServos &leg, float hipAngle, float kneeAngle, float ankleAngle) {
    // Set all servo angles with minimal delay between movements
    setServoAngle(leg.pcaIndex, leg.hip, hipAngle);
    delay(500);
    setServoAngle(leg.pcaIndex, leg.knee, kneeAngle);
    delay(500);
    setServoAngle(leg.pcaIndex, leg.ankle, ankleAngle);
    delay(500);
    Serial.print("Leg controlled: Hip=");
    Serial.print(hipAngle);
    Serial.print("° Knee=");
    Serial.print(kneeAngle);
    Serial.print("° Ankle=");
    Serial.print(ankleAngle);
    Serial.println("°");
}

/**
 * @brief Scans the I2C bus for devices
 */
void scanI2CDevices() {
  Serial.println("Scanning I2C bus...");
  uint8_t deviceCount = 0;
  
  for (uint8_t address = 1; address < 128; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  Serial.print("Found ");
  Serial.print(deviceCount);
  Serial.println(" I2C device(s)");
}

/**
 * @brief Sets all legs to the home position using your custom values
 */
/**
 * @brief Sets all legs to the home position using your custom values
 * Activates all ankles first, then hips, then knees
 */
void homePosition() {
  Serial.println("Setting legs to custom home position");
  
  // Bước 1: Thiết lập tất cả các ankle (bắp chân) trước
  Serial.println("Setting all ankles...");
  setServoAngle(leg1.pcaIndex, leg1.ankle, leg1Home[2]);
  setServoAngle(leg2.pcaIndex, leg2.ankle, leg2Home[2]);
  setServoAngle(leg3.pcaIndex, leg3.ankle, leg3Home[2]);
  setServoAngle(leg4.pcaIndex, leg4.ankle, leg4Home[2]);
  setServoAngle(leg5.pcaIndex, leg5.ankle, leg5Home[2]);
  setServoAngle(leg6.pcaIndex, leg6.ankle, leg6Home[2]);
  delay(100); // Đợi ankle di chuyển đến vị trí
  
  // Bước 2: Thiết lập tất cả các hip (hông) tiếp theo
  Serial.println("Setting all hips...");
  setServoAngle(leg1.pcaIndex, leg1.hip, leg1Home[0]);
  setServoAngle(leg2.pcaIndex, leg2.hip, leg2Home[0]);
  setServoAngle(leg3.pcaIndex, leg3.hip, leg3Home[0]);
  setServoAngle(leg4.pcaIndex, leg4.hip, leg4Home[0]);
  setServoAngle(leg5.pcaIndex, leg5.hip, leg5Home[0]);
  setServoAngle(leg6.pcaIndex, leg6.hip, leg6Home[0]);
  delay(100); // Đợi hip di chuyển đến vị trí
  
  // Bước 3: Cuối cùng thiết lập tất cả các knee (đầu gối)
  Serial.println("Setting all knees...");
  setServoAngle(leg1.pcaIndex, leg1.knee, leg1Home[1]);
  setServoAngle(leg2.pcaIndex, leg2.knee, leg2Home[1]);
  setServoAngle(leg3.pcaIndex, leg3.knee, leg3Home[1]);
  setServoAngle(leg4.pcaIndex, leg4.knee, leg4Home[1]);
  setServoAngle(leg5.pcaIndex, leg5.knee, leg5Home[1]);
  setServoAngle(leg6.pcaIndex, leg6.knee, leg6Home[1]);
  delay(100); // Đợi knee di chuyển đến vị trí
  
  Serial.println("Home position set");
}

/**
 * @brief Display current input prompt based on state
 */
void displayPrompt() {
  switch (inputState) {
    case SELECT_PCA:
      Serial.println("\n--- Select PCA ---");
      Serial.println("Enter PCA number (1 or 2):");
      break;
      
    case SELECT_SERVO:
      Serial.print("\n--- PCA");
      Serial.print(currentPca + 1);
      Serial.println(" selected ---");
      Serial.println("Enter servo number (0-15):");
      break;
      
    case ADJUST_ANGLE:
      Serial.print("\n--- PCA");
      Serial.print(currentPca + 1);
      Serial.print(" Servo ");
      Serial.print(currentServo);
      Serial.println(" selected ---");
      Serial.print("Current angle: ");
      Serial.print(currentAngle);
      Serial.println("°");
      Serial.println("Press 'A' to increase by 5°");
      Serial.println("Press 'D' to decrease by 5°");
      Serial.println("Press 'H' to go to home position");
      Serial.println("Press ESC to select a different servo");
      break;
  }
}

/**
 * @brief Process user input based on current state
 */
void processInput(char input) {
  // ESC key pressed (ASCII 27)
  if (input == 27 && inputState == ADJUST_ANGLE) {
    inputState = SELECT_PCA;
    inputBuffer = "";
    displayPrompt();
    return;
  }
  
  // 'A' or 'a' key pressed (increase angle)
  if ((input == 'A' || input == 'a') && inputState == ADJUST_ANGLE) {
    // Increment angle by 5 degrees
    currentAngle += ANGLE_INCREMENT;
    if (currentAngle > 180) currentAngle = 0; // Wrap around
    
    // Apply the new angle
    setServoAngle(currentPca, currentServo, currentAngle);
    
    // Display current angle
    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.println("°");
    return;
  }
  
  // 'D' or 'd' key pressed (decrease angle)
  if ((input == 'D' || input == 'd') && inputState == ADJUST_ANGLE) {
    // Decrement angle by 5 degrees
    currentAngle -= ANGLE_INCREMENT;
    if (currentAngle < 0) currentAngle = 180; // Wrap around
    
    // Apply the new angle
    setServoAngle(currentPca, currentServo, currentAngle);
    
    // Display current angle
    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.println("°");
    return;
  }
  
  // 'H' or 'h' key pressed (go to home position)
  if ((input == 'H' || input == 'h') && inputState == ADJUST_ANGLE) {
    homePosition();
    return;
  }
  
  // Handle backspace (ASCII 8 or 127)
  if ((input == 8 || input == 127) && inputBuffer.length() > 0) {
    inputBuffer.remove(inputBuffer.length() - 1);
    Serial.print("\b \b"); // Backspace, space, backspace
    return;
  }
  
  // Handle newline/carriage return (Enter key)
  if (input == '\r' || input == '\n') {
    if (inputBuffer.length() > 0) {
      int value = inputBuffer.toInt();
      
      switch (inputState) {
        case SELECT_PCA:
          if (value == 1 || value == 2) {
            currentPca = value - 1; // Convert to 0-based index
            inputState = SELECT_SERVO;
          } else {
            Serial.println("Invalid PCA number. Enter 1 or 2.");
          }
          break;
          
        case SELECT_SERVO:
          if (value >= 0 && value <= 15) {
            currentServo = value;
            
            // Try to set a sensible starting angle based on which servo this is
            currentAngle = 90; // Default
            
            // If this is a servo we know about, use its home position value
            if (currentPca == 0) {
              if (currentServo == leg1.hip) currentAngle = leg1Home[0];
              else if (currentServo == leg1.knee) currentAngle = leg1Home[1];
              else if (currentServo == leg1.ankle) currentAngle = leg1Home[2];
              else if (currentServo == leg2.hip) currentAngle = leg2Home[0];
              else if (currentServo == leg2.knee) currentAngle = leg2Home[1];
              else if (currentServo == leg2.ankle) currentAngle = leg2Home[2];
              else if (currentServo == leg3.hip) currentAngle = leg3Home[0];
              else if (currentServo == leg3.knee) currentAngle = leg3Home[1];
              else if (currentServo == leg3.ankle) currentAngle = leg3Home[2];
            } else {
              if (currentServo == leg4.hip) currentAngle = leg4Home[0];
              else if (currentServo == leg4.knee) currentAngle = leg4Home[1];
              else if (currentServo == leg4.ankle) currentAngle = leg4Home[2];
              else if (currentServo == leg5.hip) currentAngle = leg5Home[0];
              else if (currentServo == leg5.knee) currentAngle = leg5Home[1];
              else if (currentServo == leg5.ankle) currentAngle = leg5Home[2];
              else if (currentServo == leg6.hip) currentAngle = leg6Home[0];
              else if (currentServo == leg6.knee) currentAngle = leg6Home[1];
              else if (currentServo == leg6.ankle) currentAngle = leg6Home[2];
            }
            
            setServoAngle(currentPca, currentServo, currentAngle);
            inputState = ADJUST_ANGLE;
          } else {
            Serial.println("Invalid servo number. Enter 0-15.");
          }
          break;
          
        default:
          break;
      }
      
      inputBuffer = "";
      displayPrompt();
    }
    return;
  }
  
  // Only add digits to buffer when in selection mode
  if ((inputState == SELECT_PCA || inputState == SELECT_SERVO) && 
      input >= '0' && input <= '9') {
    inputBuffer += input;
    Serial.print(input);
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== Hexapod Servo Adjustment Tool ===");
  
  Wire.begin(21, 22); // SDA, SCL
  scanI2CDevices();
  
  // Initialize PCA9685 drivers
  pca1.begin();
  pca1.setPWMFreq(SERVO_FREQ);
  
  pca2.begin();
  pca2.setPWMFreq(SERVO_FREQ);
  
  // Set to custom home position
  homePosition();
  
  // displayPrompt();
}

void loop() {
  // Đợi lệnh từ Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'w': // Tiến
        Serial.println("Command: Move Forward");
        moveForward(50); // Di chuyển tiến 50mm
        break;
        
      case 's': // Lùi
        Serial.println("Command: Move Backward");
        moveBackward(50); // Di chuyển lùi 50mm
        break;
        
      case 'a': // Trái
        Serial.println("Command: Move Left");
        moveLeft(50); // Di chuyển sang trái 50mm
        break;
        
      case 'd': // Phải
        Serial.println("Command: Move Right");
        moveRight(50); // Di chuyển sang phải 50mm
        break;
        
      case 'h': // Home position
        Serial.println("Command: Home Position");
        homePosition();
        break;
        
      default:
        break;
    }
    
    // Xóa buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
  
  // Thời gian delay để tránh xử lý quá nhanh
  delay(50);
}