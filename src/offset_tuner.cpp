// Distance sensor offset tuner. Put the robot's center (the tracking center LemLib uses, not
// necessarily the middle of the frame) exactly on x = 0 or y = 0, square to the walls. For the
// selected sensor it works out what offset would make the reading put the robot exactly on that line.
//
// Run it by calling offsetTuner() from opcontrol() (like spinTest()). Controller:
//   A   next sensor
//   Y   heading used for the math: IMU -> fixed 0 -> 90 -> 180 -> 270 -> IMU
//   B   exit; prints the table with the new offsets to the terminal
//
// A sensor pointing at the left/right wall assumes x = 0; one pointing at the top/bottom wall
// assumes y = 0. With the robot square, only the offset along the beam changes the answer
// (forward for front/back sensors, right for side sensors), so that's the one it calculates.
//
// The math is the same as distance_reset.cpp, run backwards: sensor position + reading * beam
// direction = wall, solved for the offset instead of the robot's center. Starting values come from
// include/localization/robot_config.hpp. Copy the results into littleX() there; the UKF,
// distanceReset() and this tuner all read it.

#include <cmath>
#include <cstdio>

#include "localization/robot_config.hpp"
#include "robot.h"

bool tuningToolActive = false;

namespace {

constexpr int kAverageLoops = 50; // ~1 s of readings (the sensor refreshes every ~33 ms)
constexpr double DEG_TO_RAD = M_PI / 180.0;
// Heading choices for Y. -1 = use the IMU (chassis heading).
constexpr double kHeadings[] = {-1, 0, 90, 180, 270};
constexpr int kHeadingCount = sizeof(kHeadings) / sizeof(kHeadings[0]);

struct Solution {
    bool valid = false;
    bool isForward = false; // which offset was calculated
    bool assumesX = false;  // true: robot on x = 0, false: on y = 0
    double offset = 0;
};

// The offset that puts the robot's center exactly on the line, for reading d (inches) at heading
// headingDeg. Robot frame: +right, +forward; heading clockwise from +Y (LemLib).
//   left/right wall:  x = wall - d*dirX - (right*cos t + forward*sin t)    = 0
//   top/bottom wall:  y = wall - d*dirY - (-right*sin t + forward*cos t)   = 0
// Solved for whichever offset the heading makes count (the other one stays as configured).
Solution solveOffset(const loc::DistanceSensorConfig& s, double d, double headingDeg, double fieldHalf) {
    Solution out;
    double t = headingDeg * DEG_TO_RAD;
    double c = std::cos(t), sn = std::sin(t);
    double a = t + s.angleDeg * DEG_TO_RAD;
    double dirX = std::sin(a), dirY = std::cos(a);
    out.assumesX = std::fabs(dirX) > std::fabs(dirY); // the wall the beam points at most directly
    double rhs = out.assumesX ? (dirX > 0 ? fieldHalf : -fieldHalf) - d * dirX
                              : (dirY > 0 ? fieldHalf : -fieldHalf) - d * dirY;
    // rhs = (x wall)  right*c + forward*sn      (y wall)  -right*sn + forward*c
    double rightCoef = out.assumesX ? c : -sn;
    double forwardCoef = out.assumesX ? sn : c;
    if (std::fabs(forwardCoef) >= std::fabs(rightCoef)) {
        out.isForward = true;
        out.offset = (rhs - s.right * rightCoef) / forwardCoef;
    } else {
        out.isForward = false;
        out.offset = (rhs - s.forward * forwardCoef) / rightCoef;
    }
    out.valid = std::isfinite(out.offset);
    return out;
}

} // namespace

void offsetTuner() {
    loc::RobotConfig cfg = loc::littleX();
    int count = (int)cfg.sensors.size();
    if (count == 0) return;

    tuningToolActive = true; // pauses the controller pose line
    int sensor = 0;
    int headingChoice = 0;
    Solution best[16]; // last solution per sensor, for the table at the end

    double history[kAverageLoops] = {};
    int historyCount = 0, historyNext = 0;
    uint32_t lastPrint = 0;
    int printLine = 0;
    controller.clear();
    pros::delay(60);

    while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        uint32_t now = pros::millis();

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            sensor = (sensor + 1) % count;
            historyCount = historyNext = 0;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            headingChoice = (headingChoice + 1) % kHeadingCount;
        }

        double heading = kHeadings[headingChoice];
        bool fixedHeading = heading >= 0;
        if (!fixedHeading) {
            heading = std::fmod(chassis.getPose().theta, 360.0);
            if (heading < 0) heading += 360.0;
        }

        // Read and average the selected sensor.
        loc::DistanceSensorConfig& s = cfg.sensors[sensor];
        pros::Distance* device = distanceSensorFor(s.name);
        int mm = device ? device->get() : PROS_ERR;
        if (mm > 0 && mm < 9999 && mm != PROS_ERR) {
            history[historyNext] = mm / 25.4;
            historyNext = (historyNext + 1) % kAverageLoops;
            if (historyCount < kAverageLoops) historyCount++;
        }
        double d = -1;
        if (historyCount > 0) {
            d = 0;
            for (int i = 0; i < historyCount; i++) d += history[i];
            d /= historyCount;
        }

        Solution sol;
        if (d >= 0) sol = solveOffset(s, d, heading, cfg.fieldHalf);
        if (sol.valid && sensor < 16) best[sensor] = sol;
        double current = sol.isForward ? s.forward : s.right;

        // The controller drops prints sent less than ~50 ms apart, so update one line at a time.
        if (now - lastPrint >= 60) {
            lastPrint = now;
            if (printLine == 0) {
                if (sol.valid) controller.print(0, 0, "%-5s %s %7.3f ", s.name, sol.isForward ? "fwd" : "rgt", sol.offset);
                else controller.print(0, 0, "%-5s no reading  ", s.name);
            } else if (printLine == 1) {
                controller.print(1, 0, "h%5.1f %s %s=0   ", heading, fixedHeading ? "FIX" : "imu",
                                 sol.assumesX ? "x" : "y");
            } else {
                controller.print(2, 0, "d%6.2f was%6.2f%s", d, current, d > s.maxRange ? "!" : " ");
            }
            printLine = (printLine + 1) % 3;
        }
        if (sol.valid) {
            screenPrint(5, "TUNE %s: %s offset %.3f (was %.3f)", s.name, sol.isForward ? "forward" : "right",
                        sol.offset, current);
            screenPrint(6, "assumes %s=0  heading %.1f %s  reading %.2f in%s", sol.assumesX ? "x" : "y", heading,
                        fixedHeading ? "(fixed)" : "(IMU)", d, d > s.maxRange ? " PAST maxRange" : "");
        } else {
            screenPrint(5, "TUNE %s: no reading", s.name);
            screenPrint(6, "");
        }

        pros::delay(20);
    }

    std::printf("\n--- offsets: paste into littleX() in include/localization/robot_config.hpp ---\n");
    for (int i = 0; i < count; i++) {
        loc::DistanceSensorConfig& s = cfg.sensors[i];
        bool tuned = i < 16 && best[i].valid;
        if (tuned) (best[i].isForward ? s.forward : s.right) = best[i].offset;
        std::printf("  %-6s right %7.3f  forward %7.3f  angle %5.1f  %s\n", s.name, s.right, s.forward, s.angleDeg,
                    tuned ? (best[i].isForward ? "(forward tuned)" : "(right tuned)") : "(not tuned)");
    }
    screenPrint(5, "offset tuner done, table in terminal");
    screenPrint(6, "");
    controller.clear();
    tuningToolActive = false;
}
