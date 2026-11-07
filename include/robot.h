/**
 * \file robot.h
 *
 * Shared handles for every device and subsystem on the robot, plus the entry
 * point into the autonomous routes.
 *
 * The objects themselves are DEFINED once in src/main.cpp. This header only
 * declares them so other translation units (src/autons.cpp, etc.) can use them.
 *
 * Routes live in src/autons.cpp and are selected inside runAuton(), so adding
 * a new one never requires touching this header.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Distance frontSensor;
extern pros::Distance leftSensor;
extern pros::Distance rightSensor;
extern pros::Distance backBottomSensor;
extern pros::Distance backTopSensor;
/// The pros::Distance for a sensor name in include/localization/robot_config.hpp, or nullptr.
pros::Distance* distanceSensorFor(const char* name);


// The V5 IMU under-reads turns slightly, and by a different amount each way.
// This scales its rotation so LemLib's odom (which reads the IMU through a
// pros::Imu*) gets true degrees. Each change in rotation is scaled by its own
// direction and added to a running total. Measured with spinTest() at full
// speed: 10 turns left read ~3588, right ~3581 instead of 3600.
class ScaledImu : public pros::Imu {
  public:
    ScaledImu(std::uint8_t port, double leftScale, double rightScale)
        : pros::Imu(port), leftScale(leftScale), rightScale(rightScale) {}
    double get_rotation() const override {
        double raw = pros::Imu::get_rotation();
        if (!std::isfinite(raw)) return raw; // still calibrating / error
        std::lock_guard<pros::Mutex> lock(mutex);
        double delta = raw - lastRaw;
        // positive = clockwise = right turn
        scaled += delta * (delta > 0 ? rightScale : leftScale);
        lastRaw = raw;
        return scaled;
    }
    double get_heading() const override {
        double heading = std::fmod(get_rotation(), 360.0);
        return heading < 0 ? heading + 360.0 : heading;
    }
    // these jump the raw rotation, so the running total has to follow
    std::int32_t reset(bool blocking = false) const override {
        std::int32_t result = pros::Imu::reset(blocking);
        zero();
        return result;
    }
    std::int32_t tare_rotation() const override {
        std::int32_t result = pros::Imu::tare_rotation();
        zero();
        return result;
    }
  private:
    void zero() const {
        std::lock_guard<pros::Mutex> lock(mutex);
        lastRaw = 0;
        scaled = 0;
    }
    double leftScale;
    double rightScale;
    mutable pros::Mutex mutex;
    mutable double lastRaw = 0;
    mutable double scaled = 0;
};
extern ScaledImu imu;
extern pros::Rotation vertical_rotation_sensor;
extern pros::Rotation horizontal_rotation_sensor;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

// ---------------------------------------------------------------- controller
extern pros::Controller controller;

// -------------------------------------------------------- autonomous (autons.cpp)
/// Runs the route picked with autonSelector(). Routes are listed in src/autons.cpp.
void runAuton();
/// True while runAuton() is running. The driver button task in initialize() waits on it.
extern bool autonRunning;
/// Controller auton picker: Left/Right to choose, X to lock in. initialize() waits on it,
/// except on a competition switch/field, where it returns right away.
void autonSelector();
extern bool pickingAuton;
/// Brain left/right buttons step through the autons. Call after pros::lcd::initialize().
void registerBrainAutonSelector();
/// Controller Left/Right picker for competition_initialize(); runs until PROS ends it.
void competitionAutonSelector();
/// For autonomous(): sets the selected route's start pose (unless locked in with X), then runs it.
void runSelectedAuton();

// Tuning routines, callable from opcontrol() for testing.
void darwin_test();
void frictiontest();
void torquetest();
void tuningPID();
void spinTest();
/// Distance sensor offset tuner (src/offset_tuner.cpp). Blocks until B. Put the robot's center
/// square on x = 0 or y = 0; it calculates the offset each sensor needs to read exactly that.
void offsetTuner();
/// Distance sensor noise test (src/sensor_noise_test.cpp). Blocks until B. Park the robot facing
/// walls, press A to measure; shows each sensor's average and spread (sigma) for Tuning.
void sensorNoiseTest();
/// True while offsetTuner() or sensorNoiseTest() runs, so the controller pose line stays out of the way.
extern bool tuningToolActive;
/// Thread-safe brain screen print for lines 5-7 (the screen task draws it).
/// Use this instead of pros::lcd::print anywhere outside the screen task.
void screenPrint(int line, const char* fmt, ...);

// Read by the screen task in main.cpp to show the spin test readout.
extern bool spinTestActive;
extern double spinTestRotation;
extern double spinTestPeakRate;

void rotationChassisPID();
void linearChassisPID();

/// Wall reset from the distance sensors (src/distance_reset.cpp). Uses the current odom pose to
/// pick which wall each sensor faces, then sets x and/or y from the readings, even if they disagree
/// with odom. Heading is left alone. Averages `samples` readings (~35 ms each), so it blocks briefly.
/// Returns true if any axis was corrected.
/// `heading` (degrees, 0 = +Y, clockwise) sets the heading first, e.g. distanceReset(180) at the
/// start of opcontrol. Leave it out (NAN) to keep the current heading.
/// `skipSensor` ignores one sensor for this reset, e.g. distanceReset(NAN, true, true, 5, &leftSensor).
bool distanceReset(double heading = NAN, bool resetX = true, bool resetY = true, int samples = 5,
                   const pros::Distance* skipSensor = nullptr);
/// Starts a task that keeps printing the distance-sensor x/y on brain line 7 (doesn't move odom).
void startWallEstimatePrinter();
/// Starts a task that blends the distance sensors into LemLib's odom with a per-axis Kalman gain.
/// Odom stays the default; the gain grows the farther the robot drives without a wall fix.
/// Prints the gains and odom sigma on brain line 7, so don't also run startWallEstimatePrinter().
void startPoseFusion();
/// Set false to pause the blending (odom uncertainty still grows while paused).
extern bool poseFusionEnabled;

/// Localization: true = the UKF (src/localization/ukf_task.cpp), false = startPoseFusion() above.
inline constexpr bool USE_UKF = true;
/// Starts the UKF task: tracking wheels + IMU + distance sensors -> chassis.setPose() every 10 ms.
/// A chassis.setPose() from anywhere else (auton start pose, distanceReset()) restarts the filter there.
/// Prints its uncertainty on brain line 7 and logs to /usd/ukf_log.csv if an SD card is in.
/// Robot geometry and tuning: include/localization/robot_config.hpp.
void startUkf();
/// Set false to pause the UKF; LemLib's odometry runs alone and the UKF picks up from there.
extern bool ukfEnabled;

void EastWestAuton();
void SkillsAuton();
void NorthSouthAuton();
