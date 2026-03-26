#include "main.h"
#include <cmath>
#include <numbers>

struct Rotation {
private:
  float rad_;

public:
  constexpr Rotation() : rad_(0.0f) {}
  constexpr explicit Rotation(float radians) : rad_(radians) {}

  static Rotation deg(float degrees) {
    return Rotation(degrees * (std::numbers::pi_v<float> / 180.0f));
  }

  static Rotation rad(float radians) {
    return Rotation(radians);
  }

  float as_deg() const {
    return rad_ * (180.0f / std::numbers::pi_v<float>);
  }

  float as_rad() const { return rad_; }

  Rotation normalize_with_cap(Rotation cap) const {
    float theta = rad_;
    float two_cap = 2.0f * cap.rad_;
    theta = std::fmod(std::fmod(theta + cap.rad_, two_cap) + two_cap, two_cap);
    theta -= cap.rad_;
    return Rotation(theta);
  }

  Rotation normalize() const {
    return normalize_with_cap(Rotation::deg(180.0f));
  }

  Rotation round(Rotation increment) const {
    float deg_inc = increment.as_deg();
    float rounded = std::round(as_deg() / deg_inc) * deg_inc;
    return Rotation::deg(rounded);
  }

  Rotation abs() const { return Rotation(std::fabs(rad_)); }

  float sin() const { return std::sin(rad_); }
  float cos() const { return std::cos(rad_); }

  float sinc() const {
    if (std::fabs(rad_) < 1e-4f) return 1.0f;
    return std::sin(rad_) / rad_;
  }

  Rotation operator+(Rotation other) const { return Rotation(rad_ + other.rad_); }
  Rotation operator-(Rotation other) const { return Rotation(rad_ - other.rad_); }
  Rotation operator*(float scalar) const { return Rotation(rad_ * scalar); }
  Rotation operator/(float scalar) const { return Rotation(rad_ / scalar); }
  Rotation operator-() const { return Rotation(-rad_); }

  Rotation& operator+=(Rotation other) { rad_ += other.rad_; return *this; }
  Rotation& operator-=(Rotation other) { rad_ -= other.rad_; return *this; }
  Rotation& operator*=(float scalar) { rad_ *= scalar; return *this; }
  Rotation& operator/=(float scalar) { rad_ /= scalar; return *this; }

  bool operator==(Rotation other) const { return rad_ == other.rad_; }
  auto operator<=>(Rotation other) const { return rad_ <=> other.rad_; }

  explicit operator float() const { return rad_; }
};

inline Rotation operator*(float scalar, Rotation rot) { return rot * scalar; }

constexpr Rotation deg(float degrees) {
  return Rotation(degrees * (std::numbers::pi_v<float> / 180.0f));
}

constexpr Rotation rad(float radians) {
  return Rotation(radians);
}

struct Position;

struct Point {
  float x = 0.0f;
  float y = 0.0f;

  Point() = default;
  Point(float x, float y) : x(x), y(y) {}
  explicit Point(const Position& pos);

  float hypot() const { return std::hypot(x, y); }

  float dot(const Point& other) const { return x * other.x + y * other.y; }
  float cross(const Point& other) const { return x * other.y - y * other.x; }

  Point rotate(Rotation angle) const {
    float c = angle.cos(), s = angle.sin();
    return { x * c - y * s, x * s + y * c };
  }

  float dist(const Point& other) const { return (*this - other).hypot(); }

  Rotation angle(const Point& other) const {
    return Rotation::rad(std::atan2(other.y - y, other.x - x));
  }

  Point operator+(Point other) const { return { x + other.x, y + other.y }; }
  Point operator-(Point other) const { return { x - other.x, y - other.y }; }
  Point operator*(float s) const { return { x * s, y * s }; }
  Point operator/(float s) const { return { x / s, y / s }; }

  Point& operator+=(Point other) { x += other.x; y += other.y; return *this; }
  Point& operator-=(Point other) { x -= other.x; y -= other.y; return *this; }
  Point& operator*=(float s) { x *= s; y *= s; return *this; }
  Point& operator/=(float s) { x /= s; y /= s; return *this; }

  bool operator==(const Point&) const = default;
};

inline Point operator*(float s, Point p) { return p * s; }

struct Position {
  float x = 0.0f;
  float y = 0.0f;
  Rotation theta;

  Position() = default;
  Position(float x, float y, Rotation theta) : x(x), y(y), theta(theta) {}

  static Position origin() { return { 0.0f, 0.0f, Rotation::deg(0.0f) }; }

  void set(float x_, float y_, Rotation theta_) { x = x_; y = y_; theta = theta_; }

  Point point() const { return { x, y }; }

  Position rotate(Rotation angle) const {
    Point p = point().rotate(angle);
    return { p.x, p.y, theta + angle };
  }

  Position operator+(Point other) const { return { x + other.x, y + other.y, theta }; }
  Position operator-(Point other) const { return { x - other.x, y - other.y, theta }; }

  bool operator==(const Position&) const = default;
};

inline Point::Point(const Position& pos) : x(pos.x), y(pos.y) {}