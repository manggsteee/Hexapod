#include "trajectory.hpp"
#include <cmath>

Trajectory::Trajectory() {}

void Trajectory::createLegTrajectory(float t, float& x, float& y, float& z, float targetY) {
    // t: tham số từ 0.0 đến 1.0 (tiến trình của chuyển động)
    // targetY: vị trí Y mục tiêu (âm = tiến, dương = lùi)
    
    const float HOME_X = 198.5f;
    const float HOME_Y = 0.0f; 
    const float HOME_Z = -50.1f;
    
    const float STRIDE_LENGTH = std::abs(targetY);  // Độ dài bước chân dựa trên targetY
    const float LIFT_HEIGHT = 56.29f;   // Độ cao nhấc chân tối đa
    const float X_RETRACTION = 19.06f;  // Mức thu chân vào khi nhấc
    
    // Xác định hướng di chuyển (1 = tiến, -1 = lùi)
    float direction = (targetY < 0) ? 1.0f : -1.0f;
    
    if (t < 0.5f) {
        // Nửa đầu: Nhấc chân lên và di chuyển về hướng mục tiêu
        float phase = t * 2.0f; // 0 -> 1
        
        // Quỹ đạo hình sin cho Z (nhấc lên)
        float liftFactor = sin(phase * M_PI);
        
        // X co vào khi nhấc lên cao nhất, sau đó duỗi ra
        float xFactor = sin(phase * M_PI);
        
        // Y di chuyển theo hướng mục tiêu
        float yFactor = phase;
        
        x = HOME_X - X_RETRACTION * xFactor;
        y = HOME_Y + direction * STRIDE_LENGTH * yFactor;
        z = HOME_Z + LIFT_HEIGHT * liftFactor;
    } else {
        // Nửa sau: Hạ chân xuống và di chuyển thân robot
        float phase = (t - 0.5f) * 2.0f; // 0 -> 1
        
        // Chân đã ở vị trí mục tiêu, giờ kéo thân robot theo
        x = HOME_X - X_RETRACTION * (1.0f - phase);
        y = HOME_Y + direction * STRIDE_LENGTH * (1.0f - phase);
        z = HOME_Z; // Chân đã được hạ xuống
    }
}

// Bezier bậc 3 (cubic)
float Trajectory::bezier(float t, float p0, float p1, float p2, float p3) {
    float t2 = t * t;
    float t3 = t2 * t;
    return p0 * (1 - 3*t + 3*t2 - t3) + 
           p1 * (3*t - 6*t2 + 3*t3) + 
           p2 * (3*t2 - 3*t3) + 
           p3 * t3;
}

void Trajectory::createSmoothTrajectory(float t, float& x, float& y, float& z) {
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
        x = bezier(phase, x0, x1, x2, x3);
        y = bezier(phase, y0, y1, y2, y3);
        z = bezier(phase, z0, z1, z2, z3);
    } else {
        // Pha kéo thân (nửa sau)
        float phase = (t - 0.5f) * 2.0f; // Chuẩn hóa 0->1
        x = HOME_X;
        y = HOME_Y - 70.0f + phase * 70.0f;
        z = HOME_Z;
    }
}

void Trajectory::createBioinspiredTrajectory(float t, float& x, float& y, float& z) {
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
