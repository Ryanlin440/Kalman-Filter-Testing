/**
 * \file localization/robot_config.hpp
 *
 * EVERYTHING ABOUT THE ROBOT'S HARDWARE THAT THE UKF NEEDS LIVES HERE.
 * When the robot is rebuilt, edit littleX() at the bottom of this file and nothing else.
 * The UKF, distanceReset() (distance_reset.cpp) and offsetTuner() all read the sensor positions
 * and field size from here. Which pros::Distance each sensor name is: distanceSensorFor() in main.cpp.
 *
 * Robot frame used in this file (same as distance_reset.cpp and LemLib):
 *   positions are inches from the tracking center (the point LemLib tracks, normally the
 *   center of rotation): +right, +forward.
 *   angles are degrees clockwise from robot forward: front 0, right 90, back 180, left 270.
 *
 * Plain C++, no PROS headers, so it compiles on the laptop too.
 */

#pragma once

#include <vector>

namespace loc {

struct DistanceSensorConfig {
    const char* name;
    double right;    // position of the sensor's FACE, inches right of center (negative = left)
    double forward;  // inches forward of center (negative = behind)
    double angleDeg; // which way it points, clockwise from robot forward
    double maxRange = 78.74; // readings past this are dropped (V5 max is 2000 mm = 78.74 in)
};

struct OdomConfig {
    // Same numbers as the lemlib::TrackingWheel offsets in main.cpp. On the robot, ukf_task.cpp
    // reads them straight from LemLib, so main.cpp is where to change them.
    double verticalWheelRight = 0;     // vertical (forward-rolling) wheel, inches right of center
    double horizontalWheelForward = 0; // horizontal (sideways-rolling) wheel, inches forward of center
    // Direction the horizontal wheel's distance counts up. Check on the robot: push it straight
    // right by hand and watch horizontal_tracking_wheel.getDistanceTraveled(). If it goes down,
    // set this to false.
    bool horizontalPositiveIsRight = true;
};

struct RobotConfig {
    double fieldHalf = 140.5 / 2; // inner wall to wall, divided by 2
    // Outside size of the robot. Only used to keep the estimate from going into a wall.
    double robotWidth = 14;
    double robotLength = 14;

    std::vector<DistanceSensorConfig> sensors;
    OdomConfig odom;

    int loopMs = 10; // localization task period
};

/// Filter tuning. These are starting guesses; Phase 5 fits them from robot logs.
struct Tuning {
    // Odometry error grows with how far the robot moves, not with time, and its VARIANCE (not its
    // sigma) is proportional to the distance. That way it doesn't depend on the loop period.
    // Same idea as FUSE_Q_PER_INCH in distance_reset.cpp.
    double wheelVarPerInch = 0.01;   // in^2 per inch rolled: 1 in sigma after 100 in (= FUSE_Q_PER_INCH)
    double wheelFloorSigma = 0.0005; // inches per step, so a parked robot still has a little noise
    // IMU heading error. The IMU drives heading in the prediction step and is NOT also used as a
    // measurement (it would be counting the same sensor twice).
    double imuVarPerRad = 3e-5;        // rad^2 per radian turned: about 2 deg sigma after 10 full turns
    double imuDriftDegPerSqrtS = 0.05; // random-walk drift while sitting still

    // Distance sensors. sigma = max(distSigmaMin, distSigmaFrac * reading), from the V5 spec.
    double distSigmaMin = 0.6;   // inches (15 mm)
    double distSigmaFrac = 0.05;
    double minRange = 0.8;       // inches; closer than this the sensor isn't trustworthy
    int minConfidence = 30;      // 0-63; PROS only reports it past 200 mm
    int sensorPeriodMs = 33;     // V5 distance sensors update about every 33 ms; don't reuse a reading
    double sensorLatencyMs = 0;  // extra delay inside the sensor beyond the 33 ms refresh (fit from logs)
    bool lagCorrection = true;   // shift each reading to "now" using how fast it was changing
    // The beam is a cone, so it reads short when it hits a wall at an angle. Beams more than
    // maxIncidenceDeg from straight on are skipped (same as MAX_INCIDENCE_DEG in distance_reset.cpp).
    // Once the cone is measured (readings at known angles to a wall), beamHalfAngleDeg can model
    // the short reads instead. 0 = off.
    double maxIncidenceDeg = 35;
    double beamHalfAngleDeg = 0;
    // Skip distance updates while turning faster than this (same as FUSE_MAX_TURN_RATE).
    double maxTurnRateDegPerS = 60;

    // Outlier handling: a reading this many sigmas from what the filter expects is skipped
    // (another robot or a game element in the way). If most readings get skipped for lostMs,
    // the filter assumes it is the one that's wrong and widens its uncertainty instead.
    double gateSigmas = 3;
    int lostMs = 500;
    double lostPositionInflation = 25; // position variance multiplier
    double lostHeadingInflation = 4;

    // Unscented transform spread. Leave these alone.
    double alpha = 1, beta = 2, kappa = 0;
};

/// The current robot. EDIT THIS when the hardware changes.
inline RobotConfig littleX() {
    RobotConfig c;
    c.fieldHalf = 140.5 / 2;
    c.robotWidth = 14;  // TODO measure the rebuilt robot
    c.robotLength = 14;

    //            name     right   forward  angle  maxRange
    c.sensors = {
        {"front",  4.0,    1.6,     0},
        {"right",  5.25,   0.268,   90,    50}, // past 50 in this one reads really badly
        {"left",  -3.95,   0.268,   270},
    };

    c.odom.verticalWheelRight = -0.55;   // vertical_tracking_wheel in main.cpp
    c.odom.horizontalWheelForward = -1.5; // horizontal_tracking_wheel in main.cpp
    c.odom.horizontalPositiveIsRight = true;

    c.loopMs = 10;
    return c;
}

} // namespace loc
