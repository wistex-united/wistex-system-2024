#ifndef RLMATH_H
#define RLMATH_H

#include <cmath>

class Vector2 {
public:
    float x, y;

    Vector2(float x, float y);
    Vector2 operator-(const Vector2& v) const;
    Vector2 normalize() const;
    static float dot(const Vector2& v1, const Vector2& v2);
    static float det(const Vector2& v1, const Vector2& v2);
};

float calculateNormalizedAngle(const Vector2& agentPos, float agentOrientationRadians, const Vector2& ballPos);

float calculate_alignment_angle(std::vector<float> robot, std::vector<float> ball, std::vector<float> goal);

#endif // RLMATH_H
