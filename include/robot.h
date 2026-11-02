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
extern pros::Distance claw_sensor;


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

extern pros::MotorGroup cascade;
extern pros::Rotation cascade_sensor;
extern lemlib::PID cascade_pid;
//void moveCascadeTo(double target);
void moveCascadeTo(double target, int timeoutMs = 2000);

extern pros::MotorGroup arm_motor;
extern pros::Rotation arm_sensor;
extern lemlib::PID arm_pid;
extern lemlib::PID arm_hold_pid;
void moveArmTo(double target, int timeoutMs = 2000, float maxSpeed = 127.0f);
void moveArmToHold(double target, int timeoutMs = 2000, float maxSpeed = 127.0f);
inline constexpr double downArmDegPinAndCup = 65;
inline constexpr double downArmDegPinAndCupClamping = -10;
inline constexpr double downArmDegJustPin = 0;
inline constexpr double allianceGoalWithNothing = 5;
inline constexpr double normalAllianceGoal = 5;

constexpr float HS_AXLE_DIAMETER = 0.25f; 
// cascade stages multiply travel; calibrated so brain-screen inches match a ruler (8.5 in real move)
constexpr float CASCADE_TRAVEL_SCALE = 2.355f;
constexpr float INCHES_PER_DEGREE = (HS_AXLE_DIAMETER * 3.14159f) / 360.0f * CASCADE_TRAVEL_SCALE;

// cascade heights are in inches
inline constexpr double cascadeRestHeight = 0;  // was 2.517. 2.017
inline constexpr double cascadeClampingHeight = -0.0;   
inline constexpr double cascadeLiftOffset = 5.137;     // extra lift before moving the arm (was 1000 deg)

float getCascadeInches();

extern pros::adi::DigitalOut clawShut;
extern bool flipBool;
extern bool clawBool;
extern bool armNeedsDown;
extern bool armPosition;
extern pros::adi::DigitalOut leftToggle;
extern pros::adi::DigitalOut rightToggle;


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
/// Thread-safe brain screen print for lines 5-7 (the screen task draws it).
/// Use this instead of pros::lcd::print anywhere outside the screen task.
void screenPrint(int line, const char* fmt, ...);

// Read by the screen task in main.cpp to show the spin test readout.
extern bool spinTestActive;
extern double spinTestRotation;
extern double spinTestPeakRate;

void moveArmToFront();
void moveArmToBack(bool async = false);
void primeArm();
void scoreArm();

void L1Button();
void L2Button();
void R1Button();
void RightArrowButton();
void toggleMech();
void toggleMechUp();
void toggleMechDown();


void rotationChassisPID();
void linearChassisPID();

void closeClaw(int timeoutMs = 3000);
void openClaw();

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

void EastWestAuton();
void SkillsAuton();
void NorthSouthAuton();

void clampClaw();