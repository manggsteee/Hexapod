#include "trajectory.hpp"
#include "utils.hpp"
#include "config.hpp"
#include <cmath>
#include <Arduino.h>

namespace hexapod {
namespace trajectory {

Point3D linear(const Point3D& start, const Point3D& end, float t) {
    t = utils::limit(t, 0.0f, 1.0f);
    
    Point3D result;
    result.x = start.x + t * (end.x - start.x);
    result.y = start.y + t * (end.y - start.y);
    result.z = start.z + t * (end.z - start.z);
    
    return result;
}

Point3D bezier(const Point3D& start, const Point3D& end, float height, float t) {
    t = utils::limit(t, 0.0f, 1.0f);
    
    // Định nghĩa điểm kiểm soát giữa
    Point3D mid;
    mid.x = (start.x + end.x) / 2.0f;
    mid.y = (start.y + end.y) / 2.0f;
    mid.z = std::max(start.z, end.z) + height;
    
    // Công thức bezier bậc 2: P(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
    float t2 = t * t;
    float mt = 1.0f - t;
    float mt2 = mt * mt;
    
    Point3D result;
    result.x = mt2 * start.x + 2 * mt * t * mid.x + t2 * end.x;
    result.y = mt2 * start.y + 2 * mt * t * mid.y + t2 * end.y;
    result.z = mt2 * start.z + 2 * mt * t * mid.z + t2 * end.z;
    
    return result;
}

// Bezier bậc 3 (cubic) cho mỗi thành phần x,y,z
float bezierCubic(float t, float p0, float p1, float p2, float p3) {
    float t2 = t * t;
    float t3 = t2 * t;
    return p0 * (1 - 3*t + 3*t2 - t3) + 
           p1 * (3*t - 6*t2 + 3*t3) + 
           p2 * (3*t2 - 3*t3) + 
           p3 * t3;
}

/**
 * Di chuyển chân theo đường cong cơ bản
 */
void createLegTrajectory(float t, float& x, float& y, float& z) {
    // t: tham số từ 0.0 đến 1.0 (tiến trình của chuyển động)
    // Quỹ đạo hình bán nguyệt khi nhấc chân
    
    const float HOME_X = 198.5f;
    const float HOME_Y = 0.0f; 
    const float HOME_Z = -50.1f;
    
    const float STRIDE_LENGTH = 150.0f;  // Độ dài bước chân
    const float LIFT_HEIGHT = 56.29f;   // Độ cao nhấc chân tối đa
    const float X_RETRACTION = 19.06f;  // Mức thu chân vào khi nhấc
    
    if (t < 0.5f) {
        // Nửa đầu: Nhấc chân lên và di chuyển về phía trước
        float phase = t * 2.0f; // 0 -> 1
        
        // Quỹ đạo hình sin cho Z (nhấc lên)
        float liftFactor = sin(phase * M_PI);
        
        // X co vào khi nhấc lên cao nhất, sau đó duỗi ra
        float xFactor = sin(phase * M_PI);
        
        // Y di chuyển tiến tới mục tiêu
        float yFactor = phase;
        
        x = HOME_X - X_RETRACTION * xFactor;
        y = HOME_Y - STRIDE_LENGTH * yFactor;
        z = HOME_Z + LIFT_HEIGHT * liftFactor;
    } else {
        // Nửa sau: Hạ chân xuống và di chuyển thân robot
        float phase = (t - 0.5f) * 2.0f; // 0 -> 1
        
        // Chân đã ở phía trước, giờ kéo thân robot tiến lên
        x = HOME_X - X_RETRACTION * (1.0f - phase);
        y = HOME_Y - STRIDE_LENGTH * (1.0f - phase);
        z = HOME_Z; // Chân đã được hạ xuống
    }
}

/**
 * Di chuyển chân theo đường cong mượt mà
 */
void createSmoothTrajectory(float t, float& x, float& y, float& z) {
    // Điểm kiểm soát cho đường cong
    const float HOME_X = 198.5f;
    const float HOME_Y = 0.0f;
    const float HOME_Z = -50.1f;
    
    // Điểm kiểm soát cho X (co vào khi nhấc)
    float x0 = HOME_X;         // Bắt đầu
    float x1 = HOME_X - 5.0f; // Điểm kiểm soát 1
    float x2 = HOME_X - 10.0f; // Điểm kiểm soát 2
    float x3 = HOME_X;         // Kết thúc (về vị trí ban đầu)
    
    // Điểm kiểm soát cho Y (di chuyển về phía trước)
    float y0 = HOME_Y;            // Bắt đầu
    float y1 = HOME_Y - 20.0f;    // Điểm kiểm soát 1
    float y2 = HOME_Y - 50.0f;    // Điểm kiểm soát 2
    float y3 = HOME_Y - 83.68f;    // Kết thúc (về phía trước)
    
    // Điểm kiểm soát cho Z (nhấc lên và hạ xuống)
    float z0 = HOME_Z;              // Bắt đầu
    float z1 = HOME_Z + 40.0f;      // Điểm kiểm soát 1 (hướng lên)
    float z2 = HOME_Z + 80.0f;      // Điểm kiểm soát 2 (cao nhất)
    float z3 = HOME_Z;              // Kết thúc (hạ xuống)

    // Tính tọa độ X, Y, Z trên đường cong Bezier
    if (t < 0.5f) {
        // Pha nhấc chân (nửa đầu tiên)
        float phase = t * 2.0f; // Chuẩn hóa 0->1
        x = bezierCubic(phase, x0, x1, x2, x3);
        y = bezierCubic(phase, y0, y1, y2, y3);
        z = bezierCubic(phase, z0, z1, z2, z3);
    } else {
        // Pha kéo thân (nửa sau)
        float phase = (t - 0.5f) * 2.0f; // Chuẩn hóa 0->1
        x = HOME_X;
        y = HOME_Y - 70.0f + phase * 70.0f;
        z = HOME_Z;
    }
}

/**
 * Di chuyển chân với quỹ đạo sinh học tự nhiên
 */
void createBioinspiredTrajectory(float t, float& x, float& y, float& z) {
    const float HOME_X = 198.5f;
    const float HOME_Y = 0.0f;
    const float HOME_Z = -50.1f;
    
    const float STRIDE_LENGTH = 70.0f;
    const float LIFT_HEIGHT = 56.29f;
    
    if (t < 0.3f) {
        // Pha 1: Nhấc nhanh (30% chu kỳ)
        float phase = t / 0.3f;
        
        // Nhấc chân theo đường cong: co vào và lên
        x = HOME_X - 20.0f * sin(phase * M_PI_2);
        z = HOME_Z + LIFT_HEIGHT * sin(phase * M_PI_2);
        y = HOME_Y - STRIDE_LENGTH * 0.3f * phase; // Di chuyển nhẹ về phía trước
    }
    else if (t < 0.5f) {
        // Pha 2: Di chuyển về phía trước (20% chu kỳ)
        float phase = (t - 0.3f) / 0.2f;
        
        // Giữ chân ở độ cao tối đa, đưa về phía trước
        x = HOME_X - 20.0f * cos(phase * M_PI_2);

        z = HOME_Z + LIFT_HEIGHT;
        y = HOME_Y - STRIDE_LENGTH * (0.3f + 0.7f * phase);
    }
    else if (t < 0.6f) {
        // Pha 3: Hạ xuống (10% chu kỳ)
        float phase = (t - 0.5f) / 0.1f;
        
        // Hạ chân xuống nhanh
        x = HOME_X;
        z = HOME_Z + LIFT_HEIGHT * (1.0f - phase);
        y = HOME_Y - STRIDE_LENGTH;
    }
    else {
        // Pha 4: Kéo thân (40% chu kỳ)
        float phase = (t - 0.6f) / 0.4f;
        
        // Chân đã tiếp đất, kéo thân robot về phía trước
        x = HOME_X;
        z = HOME_Z;
        y = HOME_Y - STRIDE_LENGTH * (1.0f - phase);
    }
}

Point3D arcTrajectory(const Point3D& start, const Point3D& end, float height, float t) {
    t = utils::limit(t, 0.0f, 1.0f);
    
    // Di chuyển ngang theo đường thẳng
    Point3D result;
    result.x = start.x + t * (end.x - start.x);
    result.y = start.y + t * (end.y - start.y);
    
    // Phần z tạo hình cung
    if (t <= 0.5f) {
        // Nửa đầu: nâng chân lên
        float lift_factor = sin(t * M_PI); // 0 -> 1 -> 0
        result.z = start.z + height * lift_factor;
    } else {
        // Nửa sau: hạ chân xuống
        result.z = start.z + (1.0f - t) * (end.z - start.z);
    }
    
    return result;
}

Point3D sineTrajectory(const Point3D& start, const Point3D& end, float height, float t) {
    t = utils::limit(t, 0.0f, 1.0f);
    
    // Di chuyển ngang theo đường thẳng
    Point3D result;
    result.x = start.x + t * (end.x - start.x);
    result.y = start.y + t * (end.y - start.y);
    
    // Phần z tạo hình sin
    float sine_factor = sin(t * M_PI);
    float base_z = start.z + t * (end.z - start.z); // Đường thẳng cơ sở
    result.z = base_z + height * sine_factor;
    
    return result;
}

} // namespace trajectory
} // namespace hexapod
