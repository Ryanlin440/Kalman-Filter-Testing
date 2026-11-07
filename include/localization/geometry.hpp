/**
 * \file localization/geometry.hpp
 *
 * Angles, frame conversions, and the ray-to-wall cast the UKF uses as its measurement model.
 *
 * Two conventions meet here:
 *   LemLib / PROS / Desmos: heading in degrees, clockwise from +Y. Robot frame +right, +forward.
 *   Inside the filter:      theta in radians, counterclockwise from +X. Body frame +forward, +left.
 * World x and y are the same in both. Convert ONLY with the functions in this file.
 */

#pragma once

#include <algorithm>
#include <cmath>

namespace loc {

constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double d) { return d * kPi / 180.0; }
inline double radToDeg(double r) { return r * 180.0 / kPi; }

/// Wraps to [-pi, pi].
inline double normalizeAngle(double a) { return std::remainder(a, 2.0 * kPi); }

/// LemLib heading (deg, clockwise from +Y) -> filter theta (rad, counterclockwise from +X).
inline double headingToTheta(double headingDeg) { return normalizeAngle(degToRad(90.0 - headingDeg)); }

/// Filter theta -> LemLib heading in [0, 360).
inline double thetaToHeading(double theta) {
    double h = std::fmod(90.0 - radToDeg(theta), 360.0);
    return h < 0 ? h + 360.0 : h;
}

/// A distance sensor in the filter's body frame: position (+forward, +left) and unit direction.
struct BodyMount {
    double forward, left;
    double dirX, dirY;
};

/// Robot-frame mount (+right, +forward, degrees clockwise from forward) -> body frame.
inline BodyMount mountFromRobotFrame(double right, double forward, double angleCwDeg) {
    double a = degToRad(-angleCwDeg);
    return {forward, -right, std::cos(a), std::sin(a)};
}

struct RayHit {
    double distance;  // inches from the sensor face to the wall
    double incidence; // radians between the beam and the wall's normal (0 = straight on)
    bool hitsXWall;   // true = left/right wall (x = +-half), false = top/bottom
};

/// Where a sensor at pose (x, y, theta) sees the wall of the square field [-half, half]^2.
///
/// Returns the ray's EXIT point from the box, since the sensor is inside it. Two details keep this
/// smooth for the filter, which pushes sigma points through it:
///   - the origin is clipped into the field, so a pose estimate outside the walls still works
///   - the result is NOT capped at the sensor's range (a channel only exists when the real sensor
///     got a reading, and capping would flatten the slope the filter learns from)
inline RayHit raycast(double x, double y, double theta, const BodyMount& m, double half) {
    double c = std::cos(theta), s = std::sin(theta);
    double ox = std::clamp(x + c * m.forward - s * m.left, -half, half);
    double oy = std::clamp(y + s * m.forward + c * m.left, -half, half);
    double dx = c * m.dirX - s * m.dirY;
    double dy = s * m.dirX + c * m.dirY;

    double tx = INFINITY, ty = INFINITY;
    if (std::fabs(dx) > 1e-12) tx = ((dx > 0 ? half : -half) - ox) / dx;
    if (std::fabs(dy) > 1e-12) ty = ((dy > 0 ? half : -half) - oy) / dy;

    bool hitsX = tx < ty;
    double t = hitsX ? tx : ty;
    if (!std::isfinite(t)) return {4 * half, 0, hitsX};
    double incidence = std::acos(std::min(1.0, std::fabs(hitsX ? dx : dy)));
    return {std::clamp(t, 0.0, 4 * half), incidence, hitsX};
}

} // namespace loc
