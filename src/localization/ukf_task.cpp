// UKF localization task: reads the sensors, runs loc::Localizer, and writes the pose into LemLib.
// Started by startUkf() in initialize() when USE_UKF is true (robot.h).

#include <cmath>
#include <cstdio>

#include "localization/localizer.hpp"
#include "robot.h"

namespace {

// Someone else calling chassis.setPose() (the auton start pose, distanceReset()) shows up as
// LemLib's pose moving away from what this task last wrote by more than the robot moved.
// When that happens the filter restarts from the new pose instead of overwriting it.
// LemLib's odometry runs on its own 10 ms timer, so it can be up to a loop ahead or behind
// (0.65 in at full speed, ~4 deg in a fast spin); the thresholds sit above that.
constexpr double kExternalJumpIn = 2.0;
constexpr double kExternalJumpDeg = 10.0;
// How unsure the filter starts out about a pose someone else set.
constexpr double kResetSigmaIn = 1.0;
constexpr double kResetSigmaDeg = 2.0;

constexpr const char* kLogPath = "/usd/ukf_log.csv";

double wrap180(double deg) { return std::remainder(deg, 360.0); }

void logHeader(FILE* log, const loc::Localizer& localizer) {
    std::fprintf(log, "# boot\nt_ms,vertical_in,horizontal_in,imu_rot_deg");
    for (int i = 0; i < localizer.sensorCount(); i++) {
        const char* n = localizer.robot().sensors[i].name;
        std::fprintf(log, ",%s_mm,%s_conf,%s_status", n, n, n);
    }
    std::fprintf(log, ",x,y,heading,var_x,var_y,var_heading,event\n");
}

void logLine(FILE* log, const loc::RawInputs& in, const loc::Localizer& localizer, const char* event) {
    std::fprintf(log, "%u,%.4f,%.4f,%.4f", (unsigned)in.timeMs, in.verticalInches, in.horizontalInches,
                 in.imuRotationDeg);
    const loc::StepReport& r = localizer.report();
    for (int i = 0; i < localizer.sensorCount(); i++)
        std::fprintf(log, ",%d,%d,%s", in.distanceMm[i], in.confidence[i], loc::statusName(r.status[i]));
    const double(&P)[3][3] = localizer.filter().covariance();
    std::fprintf(log, ",%.3f,%.3f,%.3f,%.5g,%.5g,%.5g,%s\n", localizer.x(), localizer.y(), localizer.headingDeg(),
                 P[0][0], P[1][1], P[2][2], event);
}

} // namespace

bool ukfEnabled = true;

void startUkf() {
    pros::Task task([]() {
        loc::RobotConfig cfg = loc::littleX();
        // LemLib already has the tracking wheel offsets (main.cpp); use its numbers so the two
        // can't disagree.
        cfg.odom.verticalWheelRight = vertical_tracking_wheel.getOffset();
        cfg.odom.horizontalWheelForward = horizontal_tracking_wheel.getOffset();
        loc::Localizer localizer(cfg, loc::Tuning{});

        pros::Distance* devices[loc::kMaxSensors] = {};
        for (int i = 0; i < localizer.sensorCount(); i++) {
            devices[i] = distanceSensorFor(cfg.sensors[i].name);
            if (!devices[i])
                std::printf("UKF: no pros::Distance for \"%s\", add it to distanceSensorFor() in main.cpp\n",
                            cfg.sensors[i].name);
        }

        FILE* log = pros::usd::is_installed() ? std::fopen(kLogPath, "a") : nullptr;
        if (log) logHeader(log, localizer);

        lemlib::Pose written = chassis.getPose();
        localizer.reset(written.x, written.y, written.theta, kResetSigmaIn, kResetSigmaDeg);

        loc::RawInputs prev;
        bool havePrev = false;
        int loops = 0, usedRecently = 0;
        std::uint32_t wake = pros::millis();
        while (true) {
            pros::Task::delay_until(&wake, cfg.loopMs);

            loc::RawInputs in;
            in.timeMs = pros::millis();
            in.verticalInches = vertical_tracking_wheel.getDistanceTraveled();
            in.horizontalInches = horizontal_tracking_wheel.getDistanceTraveled();
            in.imuRotationDeg = imu.get_rotation();
            for (int i = 0; i < localizer.sensorCount(); i++) {
                in.distanceMm[i] = devices[i] ? devices[i]->get() : loc::kDistanceNothing;
                in.confidence[i] = devices[i] ? devices[i]->get_confidence() : 0;
            }

            // Where LemLib should be now if nobody else touched it: what this task wrote last
            // loop, plus this loop's wheel and IMU motion (LemLib's own odometry adds that).
            double expectX = written.x, expectY = written.y, expectH = written.theta;
            if (havePrev) {
                double dV = in.verticalInches - prev.verticalInches;
                double dH = in.horizontalInches - prev.horizontalInches;
                double dRot = in.imuRotationDeg - prev.imuRotationDeg;
                if (std::isfinite(dV) && std::isfinite(dH) && std::isfinite(dRot)) {
                    double t = loc::degToRad(written.theta);
                    double right = cfg.odom.horizontalPositiveIsRight ? dH : -dH;
                    // LemLib heading: 0 = +Y, clockwise. Forward is (sin t, cos t), right is (cos t, -sin t).
                    expectX += dV * std::sin(t) + right * std::cos(t);
                    expectY += dV * std::cos(t) - right * std::sin(t);
                    expectH += dRot;
                }
            }
            prev = in;
            havePrev = true;

            localizer.step(in);
            const char* event = "";

            lemlib::Pose lem = chassis.getPose();
            if (!ukfEnabled) {
                // Paused: LemLib's odometry runs on its own. Keep the filter sitting on LemLib's
                // pose so it picks up from there when it's turned back on.
                localizer.reset(lem.x, lem.y, lem.theta, kResetSigmaIn, kResetSigmaDeg);
                written = lem;
                event = "paused";
            } else if (std::hypot(lem.x - expectX, lem.y - expectY) > kExternalJumpIn ||
                       std::fabs(wrap180(lem.theta - expectH)) > kExternalJumpDeg) {
                localizer.reset(lem.x, lem.y, lem.theta, kResetSigmaIn, kResetSigmaDeg);
                written = lem;
                event = "external setPose";
            } else {
                // Keep LemLib's turn count (its heading isn't wrapped to 0-360).
                double theta = lem.theta + wrap180(localizer.headingDeg() - lem.theta);
                chassis.setPose(localizer.x(), localizer.y(), theta);
                written = {(float)localizer.x(), (float)localizer.y(), (float)theta};
                if (localizer.report().recovered) event = "lost recovery";
            }

            if (log) {
                logLine(log, in, localizer, event);
                if (loops % 100 == 0) std::fflush(log); // once a second
            }

            usedRecently += localizer.report().used;
            if (++loops % 10 == 0) {
                const double(&P)[3][3] = localizer.filter().covariance();
                screenPrint(7, "UKF %s s%.1fin %.1fdeg used %d/100ms", ukfEnabled ? "on" : "off",
                            std::sqrt(P[0][0] + P[1][1]), loc::radToDeg(std::sqrt(P[2][2])), usedRecently);
                usedRecently = 0;
            }
        }
    });
}
