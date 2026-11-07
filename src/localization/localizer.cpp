#include "localization/localizer.hpp"

#include <algorithm>
#include <cmath>

namespace loc {

const char* statusName(SensorStatus s) {
    switch (s) {
        case SensorStatus::Used: return "used";
        case SensorStatus::NoReading: return "no reading";
        case SensorStatus::LowConfidence: return "low confidence";
        case SensorStatus::OutOfRange: return "out of range";
        case SensorStatus::Waiting: return "waiting";
        case SensorStatus::Turning: return "turning";
        case SensorStatus::Steep: return "steep";
        case SensorStatus::Gated: return "gated";
    }
    return "?";
}

Ukf::Params Localizer::filterParams(const RobotConfig& robot, const Tuning& tuning) {
    Ukf::Params p;
    p.alpha = tuning.alpha;
    p.beta = tuning.beta;
    p.kappa = tuning.kappa;
    p.fieldHalf = robot.fieldHalf;
    // The center can't get closer to a wall than half the robot's narrower side.
    p.positionBound = robot.fieldHalf - 0.5 * std::min(robot.robotWidth, robot.robotLength);
    p.gateSigmas = tuning.gateSigmas;
    p.maxIncidenceRad = degToRad(tuning.maxIncidenceDeg);
    p.beamHalfAngleRad = degToRad(tuning.beamHalfAngleDeg);
    return p;
}

Localizer::Localizer(const RobotConfig& robot, const Tuning& tuning)
    : robot_(robot), tuning_(tuning), ukf_(filterParams(robot, tuning)) {
    sensorCount_ = std::min<int>(robot_.sensors.size(), kMaxSensors);
    for (int i = 0; i < sensorCount_; i++) {
        const DistanceSensorConfig& s = robot_.sensors[i];
        mounts_[i] = mountFromRobotFrame(s.right, s.forward, s.angleDeg);
    }
    ukf_.setMounts(mounts_, sensorCount_);
    reset(0, 0, 0);
}

void Localizer::reset(double x, double y, double headingDeg, double positionSigma, double headingSigmaDeg) {
    double ps = positionSigma * positionSigma;
    double hs = degToRad(headingSigmaDeg) * degToRad(headingSigmaDeg);
    double P[3][3] = {{ps, 0, 0}, {0, ps, 0}, {0, 0, hs}};
    ukf_.reset(x, y, headingToTheta(headingDeg), P);
    losing_ = false;
}

void Localizer::step(const RawInputs& in) {
    report_ = StepReport();
    if (!havePrev_) {
        // First call: nothing to diff against yet.
        prev_ = in;
        havePrev_ = true;
        for (int i = 0; i < sensorCount_; i++) report_.status[i] = SensorStatus::Waiting;
        return;
    }

    double dt = (in.timeMs - prev_.timeMs) / 1000.0;
    if (!(dt > 0)) dt = robot_.loopMs / 1000.0;

    // ---------------------------------------------------------------- predict
    double dV = in.verticalInches - prev_.verticalInches;
    double dH = in.horizontalInches - prev_.horizontalInches;
    double dRot = in.imuRotationDeg - prev_.imuRotationDeg;
    // A jump no drivetrain can make in one loop means something re-zeroed the sensors
    // (chassis.calibrate(), imu.reset()). Treat it as no motion and carry on from the new zero.
    if (!std::isfinite(dV) || std::fabs(dV) > 20) dV = 0;
    if (!std::isfinite(dH) || std::fabs(dH) > 20) dH = 0;
    if (!std::isfinite(dRot) || std::fabs(dRot) > 90) dRot = 0;

    double dTheta = -degToRad(dRot); // IMU is clockwise degrees, filter is counterclockwise radians
    double vRight = robot_.odom.verticalWheelRight;
    double hForward = robot_.odom.horizontalWheelForward;
    double hLeft = robot_.odom.horizontalPositiveIsRight ? -dH : dH;

    // A wheel off the center of rotation gets swept along when the robot turns:
    //   vertical wheel reads   forward + verticalWheelRight * dTheta
    //   horizontal wheel reads lateral + horizontalWheelForward * dTheta   (lateral = +left)
    // Take that back out to get how the chassis itself moved.
    double forward = dV - vRight * dTheta;
    double lateral = hLeft - hForward * dTheta;

    // Noise grows with motion, not time: a wheel that isn't turning makes no error. Variance is
    // proportional to distance so the total doesn't depend on how often this runs. The offset
    // correction above multiplies the IMU's error by the wheel offsets, so that's added too.
    double drift = degToRad(tuning_.imuDriftDegPerSqrtS);
    double varTurn = tuning_.imuVarPerRad * std::fabs(dTheta);
    double varTheta = varTurn + drift * drift * dt;
    double floor2 = tuning_.wheelFloorSigma * tuning_.wheelFloorSigma;
    double varForward = tuning_.wheelVarPerInch * std::fabs(dV) + floor2 + vRight * vRight * varTurn;
    double varLateral = tuning_.wheelVarPerInch * std::fabs(dH) + floor2 + hForward * hForward * varTurn;
    double before[3] = {ukf_.state()[0], ukf_.state()[1], ukf_.state()[2]};
    ukf_.predict(forward, lateral, dTheta, std::sqrt(varForward), std::sqrt(varLateral), std::sqrt(varTheta));

    report_.turnRateDegPerS = std::fabs(dRot) / dt;
    bool turning = report_.turnRateDegPerS > tuning_.maxTurnRateDegPerS;

    // ----------------------------------------------------------------- update
    Measurement z[kMaxSensors];
    int sensorOf[kMaxSensors];
    int count = 0;
    for (int i = 0; i < sensorCount_; i++) {
        report_.reading[i] = -1;
        int mm = in.distanceMm[i];
        if (mm == kDistanceError || mm <= 0 || mm >= kDistanceNothing) {
            report_.status[i] = SensorStatus::NoReading;
            continue;
        }
        if (mm > 200 && in.confidence[i] < tuning_.minConfidence) {
            report_.status[i] = SensorStatus::LowConfidence;
            continue;
        }
        double inches = mm / 25.4;
        report_.reading[i] = inches;
        if (inches < tuning_.minRange || inches > robot_.sensors[i].maxRange) {
            report_.status[i] = SensorStatus::OutOfRange;
            continue;
        }
        if (everUsed_[i] && in.timeMs - lastUsedMs_[i] < (std::uint32_t)tuning_.sensorPeriodMs) {
            report_.status[i] = SensorStatus::Waiting;
            continue;
        }
        if (turning) {
            report_.status[i] = SensorStatus::Turning;
            continue;
        }

        double sigma = std::max(tuning_.distSigmaMin, tuning_.distSigmaFrac * inches);
        if (tuning_.lagCorrection) {
            // The reading is on average half a refresh old (plus any internal lag). Driving at the
            // wall at 60 in/s that's about 1 in, and turning it sweeps the beam along the wall.
            // Shift it to now using how fast this sensor's expected reading changed over the last
            // step (covers driving and turning), and count the unknown part of the age as extra
            // noise (uniform over one refresh period: variance = (rate*T)^2 / 12).
            double rangeRate = (ukf_.expected(ukf_.state(), i).distance - ukf_.expected(before, i).distance) / dt;
            rangeRate = std::clamp(rangeRate, -300.0, 300.0); // the beam jumped to another wall
            double period = tuning_.sensorPeriodMs / 1000.0;
            double meanAge = 0.5 * period + tuning_.sensorLatencyMs / 1000.0;
            inches += rangeRate * meanAge;
            double ageNoise = rangeRate * period;
            sigma = std::sqrt(sigma * sigma + ageNoise * ageNoise / 12.0);
        }
        z[count] = {i, inches, sigma};
        sensorOf[count] = i;
        count++;
    }

    UpdateStats stats = ukf_.update(z, count);
    for (int k = 0; k < count; k++) {
        int i = sensorOf[k];
        lastUsedMs_[i] = in.timeMs;
        everUsed_[i] = true;
        report_.predicted[i] = stats.predicted[k];
        switch (stats.verdict[k]) {
            case Verdict::Used: report_.status[i] = SensorStatus::Used; break;
            case Verdict::Gated: report_.status[i] = SensorStatus::Gated; break;
            case Verdict::Steep: report_.status[i] = SensorStatus::Steep; break;
        }
    }
    report_.used = stats.used;

    // Lost-filter recovery. One blocked sensor gets gated while the others agree, which is fine.
    // But if most readings keep failing for lostMs, the filter is probably the one that's wrong,
    // and a plain gate would reject the truth forever. Widen the uncertainty so it can re-lock.
    if (stats.used + stats.gated > 0) {
        if (stats.gated > stats.used) {
            if (!losing_) {
                losing_ = true;
                losingSinceMs_ = in.timeMs;
            } else if (in.timeMs - losingSinceMs_ >= (std::uint32_t)tuning_.lostMs) {
                ukf_.inflate(tuning_.lostPositionInflation, tuning_.lostHeadingInflation);
                losing_ = false;
                report_.recovered = true;
            }
        } else {
            losing_ = false;
        }
    }

    prev_ = in;
}

} // namespace loc
