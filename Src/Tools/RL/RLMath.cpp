#include "RLMath.h"

Vector2::Vector2(float x, float y) : x(x), y(y) {}

Vector2 Vector2::operator-(const Vector2& v) const {
    return Vector2(x - v.x, y - v.y);
}

Vector2 Vector2::normalize() const {
    float length = std::sqrt(x * x + y * y);
    return Vector2(x / length, y / length);
}

float Vector2::dot(const Vector2& v1, const Vector2& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

float Vector2::det(const Vector2& v1, const Vector2& v2) {
    return v1.x * v2.y - v1.y * v2.x;
}

float calculateNormalizedAngle(const Vector2& agentPos, float agentOrientationRadians, const Vector2& ballPos) {
    Vector2 agentDir(std::cos(agentOrientationRadians), std::sin(agentOrientationRadians));
    Vector2 toBall = (ballPos - agentPos).normalize();
    float dot = Vector2::dot(agentDir, toBall);
    float det = Vector2::det(agentDir, toBall);
    float angle = std::atan2(det, dot);
    return std::abs(angle);
}

float calculate_alignment_angle(std::vector<float> robot, std::vector<float> ball, std::vector<float> goal) {
    // Step 1: Calculate goal-ball angle
    float goal_ball_angle = std::atan2(ball[1] - goal[1], ball[0] - goal[0]);
    
    // Step 2: Calculate robot-ball angle
    float robot_ball_angle = std::atan2(ball[1] - robot[1], ball[0] - robot[0]);
    
    // Step 3: Calculate the difference
    float angle_to_traverse = (goal_ball_angle - robot_ball_angle);
    
    // Normalize angle to be between -pi and pi
    return std::abs(std::atan2(std::sin(angle_to_traverse), std::cos(angle_to_traverse)));
}
