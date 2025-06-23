#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "hexapod.h" // Header file chứa khai báo LegServos và các hàm như controlLeg

// Khai báo các hằng số và cấu trúc cần thiết cho inverse kinematics
#define LEG_COXA_LENGTH 65  // mm - Chiều dài đốt coxa (hip)
#define LEG_FEMUR_LENGTH 65 // mm - Chiều dài đốt femur (thigh)
#define LEG_TIBIA_LENGTH 90 // mm - Chiều dài đốt tibia (shin)

// Sử dụng các biến từ file chính
extern const LegServos leg1;
extern const LegServos leg2;
extern const LegServos leg3;
extern const LegServos leg4;
extern const LegServos leg5;
extern const LegServos leg6;

// Cấu trúc lưu tọa độ 3D
struct Point3D
{
  float x;
  float y;
  float z;
};

// Cấu trúc lưu góc các khớp
struct LegAngles
{
  float hip;   // Góc hip (coxa)
  float knee;  // Góc knee (femur)
  float ankle; // Góc ankle (tibia)
};

// Khai báo các hàm được định nghĩa trong file này
LegAngles calculateInverseKinematics(Point3D targetPos, int legNum);
void moveLegToPosition(const LegServos &leg, Point3D targetPos, int legNum);
void moveForward(float distance);
void moveBackward(float distance);
void moveLeft(float distance);
void moveRight(float distance);

/**
 * @brief Tính toán các góc khớp từ tọa độ đầu chân (inverse kinematics)
 * @param targetPos Tọa độ đầu chân trong không gian 3D
 * @param legNum Số thứ tự chân (1-6)
 * @return Góc các khớp của chân
 */
LegAngles calculateInverseKinematics(Point3D targetPos, int legNum) {
  LegAngles angles;
  
  // Debug input
  Serial.print("IK Input - X: ");
  Serial.print(targetPos.x);
  Serial.print(" Y: ");
  Serial.print(targetPos.y);
  Serial.print(" Z: ");
  Serial.println(targetPos.z);
  
  // ============= TÍNH GÓC HIP (COXA) =============
  // Góc hip là góc quay ngang của chân
  float hipAngleRad = atan2(targetPos.y, targetPos.x);
  angles.hip = hipAngleRad * 180.0 / PI;
  
  // Chuyển góc từ -180->180 sang 0->180 cho servo
  if (angles.hip < 0) {
    angles.hip += 180;
  }
  
  // ============= TÍNH KHOẢNG CÁCH CHO KNEE VÀ ANKLE =============
  // Khoảng cách từ gốc tọa độ đến chân theo mặt phẳng XY
  float horizontalDistance = sqrt(targetPos.x * targetPos.x + targetPos.y * targetPos.y);
  
  // Khoảng cách thực tế mà femur và tibia cần với tới (trừ đi phần coxa)
  float workingDistance = horizontalDistance - LEG_COXA_LENGTH;
  
  // Kiểm tra khoảng cách có hợp lệ không
  if (workingDistance < 0) {
    Serial.println("WARNING: Target too close to hip, adjusting...");
    workingDistance = 5; // Minimum working distance
  }
  
  // Khoảng cách thẳng từ đầu femur đến điểm đích
  float targetDistance = sqrt(workingDistance * workingDistance + targetPos.z * targetPos.z);
  
  // Kiểm tra giới hạn với tới của chân
  float maxReach = LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH;
  float minReach = abs(LEG_FEMUR_LENGTH - LEG_TIBIA_LENGTH);
  
  if (targetDistance > maxReach) {
    Serial.println("WARNING: Target too far, adjusting...");
    targetDistance = maxReach * 0.95;
  } else if (targetDistance < minReach) {
    Serial.println("WARNING: Target too close, adjusting...");
    targetDistance = minReach * 1.05;
  }
  
  // ============= TÍNH GÓC KNEE (FEMUR) =============
  // Sử dụng định lý cosin để tính góc tại khớp knee
  float cosKneeAngle = (LEG_FEMUR_LENGTH * LEG_FEMUR_LENGTH + 
                        LEG_TIBIA_LENGTH * LEG_TIBIA_LENGTH - 
                        targetDistance * targetDistance) / 
                       (2 * LEG_FEMUR_LENGTH * LEG_TIBIA_LENGTH);
  
  // Đảm bảo giá trị trong phạm vi hợp lệ cho acos
  cosKneeAngle = constrain(cosKneeAngle, -1.0, 1.0);
  
  // Góc tại khớp knee (góc giữa femur và tibia)
  float kneeJointAngle = acos(cosKneeAngle);
  
  // Chuyển đổi sang góc servo (180° - góc khớp để servo ở vị trí thẳng khi góc khớp = 180°)
  angles.knee = 180.0 - (kneeJointAngle * 180.0 / PI);
  
  // ============= TÍNH GÓC ANKLE (TIBIA) =============
  // Góc femur so với đường ngang
  float femurAngle = atan2(-targetPos.z, workingDistance); // Âm vì Z âm là xuống dưới
  
  // Góc giữa femur và đường thẳng nối với target  
  float cosAlpha = (LEG_FEMUR_LENGTH * LEG_FEMUR_LENGTH + 
                    targetDistance * targetDistance - 
                    LEG_TIBIA_LENGTH * LEG_TIBIA_LENGTH) / 
                   (2 * LEG_FEMUR_LENGTH * targetDistance);
  
  cosAlpha = constrain(cosAlpha, -1.0, 1.0);
  float alpha = acos(cosAlpha);
  
  // Góc tuyệt đối của femur
  float femurAbsoluteAngle = femurAngle + alpha;
  
  // Góc của tibia (để tibia thẳng đứng xuống, ankle = 90°)
  float tibiaAngle = femurAbsoluteAngle - kneeJointAngle;
  
  // Chuyển đổi sang góc servo cho ankle
  angles.ankle = 90.0 + (tibiaAngle * 180.0 / PI);
  
  // ============= HIỆU CHỈNH THEO TỪNG CHÂN =============
  // switch(legNum) {
  //   case 1: // Chân trái trước
  //     angles.hip += 0;   // Không điều chỉnh
  //     angles.knee += 0;
  //     angles.ankle += 0;
  //     break;
      
  //   case 2: // Chân trái giữa  
  //     angles.hip += 0;
  //     angles.knee += 0;
  //     angles.ankle += 0;
  //     break;
      
  //   case 3: // Chân trái sau
  //     angles.hip += 0;
  //     angles.knee += 0;
  //     angles.ankle += 0;
  //     break;
      
  //   case 4: // Chân phải trước
  //     angles.hip = 180 - angles.hip; // Đảo ngược cho bên phải
  //     angles.knee += 0;
  //     angles.ankle += 0;
  //     break;
      
  //   case 5: // Chân phải giữa
  //     angles.hip = 180 - angles.hip; // Đảo ngược cho bên phải
  //     angles.knee += 0;
  //     angles.ankle += 0;
  //     break;
      
  //   case 6: // Chân phải sau
  //     angles.hip = 180 - angles.hip; // Đảo ngược cho bên phải
  //     angles.knee += 0;
  //     angles.ankle += 0;
  //     break;
  // }
  
  // ============= GIỚI HẠN GÓC AN TOÀN =============
  angles.hip = constrain(angles.hip, 20, 160);
  angles.knee = constrain(angles.knee, 30, 150);
  angles.ankle = constrain(angles.ankle, 30, 150);
  
  // Debug output
  Serial.print("Calculated angles - Hip: ");
  Serial.print(angles.hip);
  Serial.print("° Knee: ");
  Serial.print(angles.knee);
  Serial.print("° Ankle: ");
  Serial.print(angles.ankle);
  Serial.println("°");
  
  return angles;
}

/**
 * @brief Di chuyển một chân đến vị trí 3D cụ thể
 * @param leg Cấu trúc chân cần điều khiển
 * @param targetPos Tọa độ đích trong không gian 3D
 * @param legNum Số thứ tự chân (1-6)
 */
void moveLegToPosition(const LegServos &leg, Point3D targetPos, int legNum)
{
  // Tính toán góc các khớp từ tọa độ đích
  LegAngles angles = calculateInverseKinematics(targetPos, legNum);

  // Điều khiển chân với các góc tính được
  controlLeg(leg, angles.hip, angles.knee, angles.ankle);

  delay(1000); // Đợi chân di chuyển đến vị trí
}

/**
 * @brief Di chuyển hexapod về phía trước
 * @param distance Khoảng cách di chuyển (mm)
 */
void moveForward(float distance)
{
  Serial.println("TEST MODE: Moving only leg 1 forward");

  // Đặt các tham số di chuyển
  float stepHeight = 20; // Độ cao bước chân (mm)
  float stepLength = 20; // Độ dài bước chân (mm)

  // Tọa độ gốc cho chân test (khi ở home position)
  Point3D leg1Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};

  Serial.println("Step 1: Nâng chân lên");
  // Nâng chân lên
  leg1Pos.z += stepHeight;
  moveLegToPosition(leg4, leg1Pos, 1);
  delay(500); // Delay dài hơn để dễ quan sát

    Serial.println("Step 2: Di chuyển chân về phía trước");
    // Di chuyển chân về phía trước
    leg1Pos.x += stepLength;
    moveLegToPosition(leg4, leg1Pos, 1);
    delay(500);

    Serial.println("Step 3: Hạ chân xuống");
    // Hạ chân xuống
    leg1Pos.z -= stepHeight;
    moveLegToPosition(leg4, leg1Pos, 1);
    delay(500);

    Serial.println("Step 4: Kéo thân về phía trước (di chuyển chân về sau)");
    // Kéo thân về phía trước (di chuyển chân về sau)
    leg1Pos.x -= stepLength;
    moveLegToPosition(leg4, leg1Pos, 1);
    delay(500);

  Serial.println("Test complete - Leg returned to starting position");

  // In ra góc hiện tại của các servo để debug
  LegAngles angles = calculateInverseKinematics(leg1Pos, 1);
  Serial.print("Final angles - Hip: ");
  Serial.print(angles.hip);
  Serial.print("° Knee: ");
  Serial.print(angles.knee);
  Serial.print("° Ankle: ");
  Serial.print(angles.ankle);
  Serial.println("°");
}

/**
 * @brief Di chuyển hexapod về phía sau
 * @param distance Khoảng cách di chuyển (mm)
 */
void moveBackward(float distance)
{
  Serial.print("Moving backward: ");
  Serial.println(distance);

  // Đặt các tham số di chuyển
  float stepHeight = 30; // Độ cao bước chân (mm)
  float stepLength = 40; // Độ dài bước chân (mm)

  // Tọa độ gốc cho mỗi chân (khi ở home position)
  Point3D leg1Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg2Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg3Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg4Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg5Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg6Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};

  // Di chuyển nhóm 1 lên (nâng chân)
  leg1Pos.z += stepHeight;
  leg3Pos.z += stepHeight;
  leg5Pos.z += stepHeight;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Di chuyển nhóm 1 về phía sau
  leg1Pos.x -= stepLength;
  leg3Pos.x -= stepLength;
  leg5Pos.x -= stepLength;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Hạ nhóm 1 xuống
  leg1Pos.z -= stepHeight;
  leg3Pos.z -= stepHeight;
  leg5Pos.z -= stepHeight;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Di chuyển nhóm 2 lên
  leg2Pos.z += stepHeight;
  leg4Pos.z += stepHeight;
  leg6Pos.z += stepHeight;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Di chuyển nhóm 2 về phía sau
  leg2Pos.x -= stepLength;
  leg4Pos.x -= stepLength;
  leg6Pos.x -= stepLength;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Hạ nhóm 2 xuống
  leg2Pos.z -= stepHeight;
  leg4Pos.z -= stepHeight;
  leg6Pos.z -= stepHeight;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Kéo thân robot về phía sau
  leg1Pos.x += stepLength;
  leg2Pos.x += stepLength;
  leg3Pos.x += stepLength;
  leg4Pos.x += stepLength;
  leg5Pos.x += stepLength;
  leg6Pos.x += stepLength;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg5, leg5Pos, 5);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);
}

/**
 * @brief Di chuyển hexapod sang trái
 * @param distance Khoảng cách di chuyển (mm)
 */
void moveLeft(float distance)
{
  Serial.print("Moving left: ");
  Serial.println(distance);

  // Đặt các tham số di chuyển
  float stepHeight = 30; // Độ cao bước chân (mm)
  float stepLength = 40; // Độ dài bước chân (mm)

  // Tọa độ gốc cho mỗi chân (khi ở home position)
  Point3D leg1Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg2Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg3Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg4Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg5Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg6Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};

  // Di chuyển nhóm 1 lên (nâng chân)
  leg1Pos.z += stepHeight;
  leg3Pos.z += stepHeight;
  leg5Pos.z += stepHeight;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Di chuyển nhóm 1 sang trái
  leg1Pos.y += stepLength;
  leg3Pos.y += stepLength;
  leg5Pos.y += stepLength;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Hạ nhóm 1 xuống
  leg1Pos.z -= stepHeight;
  leg3Pos.z -= stepHeight;
  leg5Pos.z -= stepHeight;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Di chuyển nhóm 2 lên
  leg2Pos.z += stepHeight;
  leg4Pos.z += stepHeight;
  leg6Pos.z += stepHeight;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Di chuyển nhóm 2 sang trái
  leg2Pos.y += stepLength;
  leg4Pos.y += stepLength;
  leg6Pos.y += stepLength;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Hạ nhóm 2 xuống
  leg2Pos.z -= stepHeight;
  leg4Pos.z -= stepHeight;
  leg6Pos.z -= stepHeight;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Kéo thân robot sang trái
  leg1Pos.y -= stepLength;
  leg2Pos.y -= stepLength;
  leg3Pos.y -= stepLength;
  leg4Pos.y -= stepLength;
  leg5Pos.y -= stepLength;
  leg6Pos.y -= stepLength;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg5, leg5Pos, 5);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);
}

void moveRight(float distance)
{
  Serial.print("Moving right: ");
  Serial.println(distance);

  // Đặt các tham số di chuyển
  float stepHeight = 30; // Độ cao bước chân (mm)
  float stepLength = 40; // Độ dài bước chân (mm)

  // Tọa độ gốc cho mỗi chân (khi ở home position)
  Point3D leg1Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg2Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg3Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg4Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg5Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};
  Point3D leg6Pos = {LEG_COXA_LENGTH + LEG_FEMUR_LENGTH / 2, 0, -60};

  // Di chuyển nhóm 1 lên (nâng chân)
  leg1Pos.z += stepHeight;
  leg3Pos.z += stepHeight;
  leg5Pos.z += stepHeight;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Di chuyển nhóm 1 sang phải
  leg1Pos.y -= stepLength;
  leg3Pos.y -= stepLength;
  leg5Pos.y -= stepLength;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Hạ nhóm 1 xuống
  leg1Pos.z -= stepHeight;
  leg3Pos.z -= stepHeight;
  leg5Pos.z -= stepHeight;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg5, leg5Pos, 5);
  delay(200);

  // Di chuyển nhóm 2 lên
  leg2Pos.z += stepHeight;
  leg4Pos.z += stepHeight;
  leg6Pos.z += stepHeight;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Di chuyển nhóm 2 sang phải
  leg2Pos.y -= stepLength;
  leg4Pos.y -= stepLength;
  leg6Pos.y -= stepLength;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Hạ nhóm 2 xuống
  leg2Pos.z -= stepHeight;
  leg4Pos.z -= stepHeight;
  leg6Pos.z -= stepHeight;

  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);

  // Kéo thân robot sang phải
  leg1Pos.y += stepLength;
  leg2Pos.y += stepLength;
  leg3Pos.y += stepLength;
  leg4Pos.y += stepLength;
  leg5Pos.y += stepLength;
  leg6Pos.y += stepLength;

  moveLegToPosition(leg1, leg1Pos, 1);
  moveLegToPosition(leg2, leg2Pos, 2);
  moveLegToPosition(leg3, leg3Pos, 3);
  moveLegToPosition(leg4, leg4Pos, 4);
  moveLegToPosition(leg5, leg5Pos, 5);
  moveLegToPosition(leg6, leg6Pos, 6);
  delay(200);
}