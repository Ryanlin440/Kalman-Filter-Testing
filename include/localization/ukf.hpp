/**
 * \file localization/ukf.hpp
 *
 * Unscented Kalman filter over the pose [x, y, theta] (inches, inches, radians CCW from +X).
 * Ported from simulation/ukf.py in github.com/Ryanlin440/ukfwithpython, with two changes:
 *   - the motion model integrates the step as a constant-curvature arc instead of a straight line
 *     at the starting heading (exact for tracking wheels, matters during fast turns)
 *   - the update can skip individual readings (innovation gate, steep beams)
 *
 * Fixed-size arrays only, no heap use in predict/update. Plain C++, no PROS.
 */

#pragma once

#include "localization/geometry.hpp"

namespace loc {

constexpr int kMaxChannels = 8; // most distance sensors the filter can take

struct Measurement {
    int mount;    // index into the table passed to setMounts()
    double range; // inches
    double sigma; // 1-sigma noise, inches
};

enum class Verdict : unsigned char { Used, Gated, Steep };

struct UpdateStats {
    int offered = 0, used = 0, gated = 0, steep = 0;
    Verdict verdict[kMaxChannels] = {};
    double predicted[kMaxChannels] = {}; // what the filter expected each sensor to read
    double nis[kMaxChannels] = {};       // (innovation / its sigma)^2
};

class Ukf {
  public:
    struct Params {
        double alpha = 1, beta = 2, kappa = 0;
        double fieldHalf = 72;
        double positionBound = 65;   // |x|, |y| never go past this (the robot can't be inside a wall)
        double gateSigmas = 3;       // <= 0 turns the gate off
        double maxIncidenceRad = kPi; // >= pi/2 turns the steep-beam check off
        // The V5 sensor's beam is a cone, so a beam hitting the wall at an angle reads the cone's
        // nearest edge: d * cos(i) / cos(max(0, i - halfAngle)). 0 = treat it as a thin ray.
        double beamHalfAngleRad = 0;
    };

    explicit Ukf(const Params& params);

    void setMounts(const BodyMount* mounts, int count);
    void reset(double x, double y, double theta, const double P[3][3]);

    /// Advances by a body-frame move (already corrected for tracking wheel offsets) with its noise.
    void predict(double forward, double lateral, double dTheta, double sigmaForward, double sigmaLateral,
                 double sigmaTheta);

    /// Corrects with distance readings. Readings that fail the gate are left out, not fed in.
    UpdateStats update(const Measurement* z, int count);

    /// Multiplies the variances (keeps correlations). Used when the filter decides it's lost.
    void inflate(double positionFactor, double headingFactor);

    const double* state() const { return x_; }
    const double (&covariance() const)[3][3] { return P_; }
    /// The 7 sigma points around the current estimate, for drawing.
    void sigmaPoints(double out[7][3]) const;
    double weightMean(int i) const { return wm_[i]; }
    double weightCov(int i) const { return wc_[i]; }

    /// Expected reading of one sensor at a pose, and the wall it would hit.
    RayHit expected(const double pose[3], int mount) const;

  private:
    void clampToField();

    Params p_;
    double lambda_;
    double wm_[7], wc_[7];
    double x_[3] = {0, 0, 0};
    double P_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    BodyMount mounts_[kMaxChannels] = {};
    int mountCount_ = 0;
};

} // namespace loc
