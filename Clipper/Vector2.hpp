#pragma once

struct Vec2
{
  Vec2() {}
  Vec2(float x_, float y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& rhs) const
  {
    Vec2 result = *this;
    result.x += rhs.x;
    result.y += rhs.y;
    return result;
  }
  Vec2 operator-(const Vec2& rhs) const
  {
    Vec2 result = *this;
    result.x -= rhs.x;
    result.y -= rhs.y;
    return result;
  }
  Vec2 operator*(float value) const
  {
    Vec2 result = *this;
    result.x *= value;
    result.y *= value;
    return result;
  }
  bool operator==(const Vec2& rhs) const
  {
    return x == rhs.x && y == rhs.y;
  }
  bool operator!=(const Vec2& rhs) const
  {
    return !((*this) == rhs);
  }
  static float Dot(const Vec2& lhs, const Vec2& rhs)
  {
    return lhs.x * rhs.x + lhs.y * rhs.y;
  }
  static float DistanceSq(const Vec2& lhs, const Vec2& rhs)
  {
    Vec2 v = lhs - rhs;
    return Dot(v, v);
  }

  float x;
  float y;
};
