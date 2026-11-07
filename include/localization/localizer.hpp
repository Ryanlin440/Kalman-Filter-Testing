/**
 * \file localization/localizer.hpp
 *
 * Turns raw sensor values, exactly as PROS returns them, into a pose from the UKF.
 *
 * What it does each step:
 *   - tracking wheel + IMU deltas -> body-frame move, corrected for where the wheels sit
 *   - predict, with noise that grows with how far the robot moved and turned
 *   - checks each distance reading (9999 / error / low confidence / out of range / too soon /
 *     turning too fast), converts mm -> inches, corrects for sensor lag, and updates
 *   - if the filter keeps rejecting most readings, widens its uncertainty so it can re-lock
 *
 * Plain C++, no PROS.
 */

#pragma once

#include <cstdint>

#include "localization/robot_config.hpp"
#include "localization/ukf.hpp"

namespace loc {

constexpr int kMaxSensors = kMaxChannels;
constexpr int kDistanceError = 2147483647; // PROS_ERR
constexpr int kDistanceNothing = 9999;     // pros::Distance::get() when nothing is in range

struct RawInputs {
    std::uint32_t timeMs = 0;       // pros::millis()
    double verticalInches = 0;      // cumulative, lemlib::TrackingWheel::getDistanceTraveled()
    double horizontalInches = 0;
    double imuRotationDeg = 0;      // cumulative, clockwise positive (imu.get_rotation())
    int distanceMm[kMaxSensors] = {}; // pros::Distance::get(), in the order of RobotConfig::sensors
    int confidence[kMaxSensors] = {}; // pros::Distance::get_confidence()
};

enum class SensorStatus : unsigned char {
    Used,          // went into the filter
    NoReading,     // 9999, error, or zero
    LowConfidence,
    OutOfRange,    // shorter than minRange or longer than the sensor's maxRange
    Waiting,       // used less than sensorPeriodMs ago, so this is probably the same reading
    Turning,       // robot turning faster than maxTurnRateDegPerS
    Steep,         // beam would hit the wall at too much of an angle
    Gated,         // too far from what the filter expected (something in the way?)
};

const char* statusName(SensorStatus s);

struct StepReport {
    SensorStatus status[kMaxSensors] = {};
    double reading[kMaxSensors] = {};   // inches, -1 if none
    double predicted[kMaxSensors] = {}; // what the filter expected, if it got that far
    int used = 0;
    bool recovered = false; // the lost-filter recovery fired this step
    double turnRateDegPerS = 0;
};

class Localizer {
  public:
    Localizer(const RobotConfig& robot, const Tuning& tuning);

    /// Sets the pose in LemLib's convention (inches, heading degrees clockwise from +Y).
    void reset(double x, double y, double headingDeg, double positionSigma = 0.5, double headingSigmaDeg = 1);

    void step(const RawInputs& in);

    // Pose in LemLib's convention.
    double x() const { return ukf_.state()[0]; }
    double y() const { return ukf_.state()[1]; }
    double headingDeg() const { return thetaToHeading(ukf_.state()[2]); }

    const Ukf& filter() const { return ukf_; }
    const StepReport& report() const { return report_; }
    const RobotConfig& robot() const { return robot_; }
    const Tuning& tuning() const { return tuning_; }
    const BodyMount& mount(int i) const { return mounts_[i]; }
    int sensorCount() const { return sensorCount_; }

    /// For tuning at runtime. Changing these mid-run is fine.
    Tuning& mutableTuning() { return tuning_; }

  private:
    static Ukf::Params filterParams(const RobotConfig& robot, const Tuning& tuning);

    RobotConfig robot_;
    Tuning tuning_;
    Ukf ukf_;
    BodyMount mounts_[kMaxSensors];
    int sensorCount_;

    bool havePrev_ = false;
    RawInputs prev_;
    std::uint32_t lastUsedMs_[kMaxSensors] = {};
    bool everUsed_[kMaxSensors] = {};
    bool losing_ = false;
    std::uint32_t losingSinceMs_ = 0;
    StepReport report_;
};

} // namespace loc
