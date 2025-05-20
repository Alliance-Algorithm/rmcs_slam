#pragma once

#include <cmath>
#include <numbers>

namespace rmcs::util {

class Angle {
public:
    constexpr explicit Angle(double radians)
        : radians_(radians) { }

    constexpr Angle()
        : radians_(0) { }

    constexpr double radians() const noexcept { return radians_; }
    constexpr double degrees() const noexcept { return radians_ * 180.0 / std::numbers::pi; }

    // Unary operators
    constexpr Angle operator+() const { return Angle { radians_ }; }
    constexpr Angle operator-() const { return Angle { -radians_ }; }

    // Binary arithmetic operators
    constexpr Angle operator+(const Angle& other) const {
        return Angle { radians_ + other.radians_ };
    }
    constexpr Angle operator-(const Angle& other) const {
        return Angle { radians_ - other.radians_ };
    }
    constexpr Angle operator*(double scalar) const { return Angle { radians_ * scalar }; }
    constexpr Angle operator/(double scalar) const { return Angle { radians_ / scalar }; }

    // Compound assignment operators
    Angle& operator+=(const Angle& other) {
        radians_ += other.radians_;
        return *this;
    }
    Angle& operator-=(const Angle& other) {
        radians_ -= other.radians_;
        return *this;
    }
    Angle& operator*=(double scalar) {
        radians_ *= scalar;
        return *this;
    }
    Angle& operator/=(double scalar) {
        // Consider handling division by zero if necessary.
        radians_ /= scalar;
        return *this;
    }

    constexpr auto operator<=>(const Angle& other) const = default;

    // Optional: Normalization (constrain angle to a specific range, e.g., [0, 2*pi) or (-pi, pi])
    // This might be useful depending on your application's needs.
    // Example for [0, 2*pi):
    Angle normalized_positive() const {
        double two_pi      = 2.0 * std::numbers::pi;
        double new_radians = std::fmod(radians_, two_pi);
        if (new_radians < 0) {
            new_radians += two_pi;
        }
        return Angle { new_radians };
    }

    // Example for (-pi, pi]:
    Angle normalized_symmetric() const {
        double pi          = std::numbers::pi;
        double two_pi      = 2.0 * std::numbers::pi;
        double new_radians = std::fmod(radians_ + pi, two_pi);
        if (new_radians < 0) {
            new_radians += two_pi;
        }
        return Angle { new_radians - pi };
    }

private:
    double radians_;
};

// Friend operator for scalar * Angle
constexpr Angle operator*(double scalar, const Angle& angle) {
    return angle * scalar; // Reuse Angle::operator*
}

} // namespace rmcs::util

namespace rmcs::util_literals {

constexpr auto operator""_deg(long double degrees) {
    return util::Angle { static_cast<double>(degrees) * std::numbers::pi / 180.0 };
}
constexpr auto operator""_deg(unsigned long long degrees) {
    return util::Angle { static_cast<double>(degrees) * std::numbers::pi / 180.0 };
}
constexpr auto operator""_rad(long double radians) {
    return util::Angle { static_cast<double>(radians) };
}
constexpr auto operator""_rad(unsigned long long radians) { // Added for consistency
    return util::Angle { static_cast<double>(radians) };
}

} // namespace rmcs::util_literals
