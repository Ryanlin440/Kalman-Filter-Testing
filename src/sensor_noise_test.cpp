// Distance sensor noise test. Park the robot (completely still) with the sensors facing walls,
// press A, and it records ~5 s of readings from every sensor at once. For each sensor it shows
// the average reading and the spread (standard deviation, sigma).
//
// Do it at a few distances (close ~10 in, middle ~30 in, far ~60 in), then use the numbers for
// distSigmaMin / distSigmaFrac in include/localization/robot_config.hpp. The filter currently
// assumes sigma = max(distSigmaMin, distSigmaFrac * reading) = max(0.6 in, 5% of the reading).
//
// Run it by calling sensorNoiseTest() from opcontrol(). Controller:
//   A   measure (keep the robot still for 5 s)
//   X   forget the runs so far
//   B   exit; prints every run and a suggested distSigmaMin / distSigmaFrac to the terminal

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "localization/robot_config.hpp"
#include "robot.h"

namespace {

constexpr int kSampleMs = 40; // sensors refresh about every 33 ms; 40 avoids counting one reading twice
constexpr int kSamples = 125; // 5 s
constexpr int kMaxSensors = 8;
constexpr int kMaxRuns = 40;
// Runs closer than this set distSigmaMin; farther ones set distSigmaFrac.
constexpr double kCloseIn = 12;

struct Stats {
    int n = 0, invalid = 0;
    double mean = 0, m2 = 0, min = 1e9, max = 0, confSum = 0;
    void add(double v, int conf) { // Welford's running mean and variance
        n++;
        double delta = v - mean;
        mean += delta / n;
        m2 += delta * (v - mean);
        min = std::min(min, v);
        max = std::max(max, v);
        confSum += conf;
    }
    double sigma() const { return n > 1 ? std::sqrt(m2 / (n - 1)) : 0; }
};

struct Run {
    const char* name;
    double mean, sigma, min, max, conf;
    int n, invalid;
};

void printRun(const Run& r, const loc::Tuning& t) {
    double assumed = std::max(t.distSigmaMin, t.distSigmaFrac * r.mean);
    std::printf("  %-6s mean %6.2f in  sigma %6.3f in (%4.2f%%)  min %6.2f  max %6.2f  conf %4.1f  n %3d  "
                "invalid %3d  | filter assumes sigma %.2f\n",
                r.name, r.mean, r.sigma, 100 * r.sigma / r.mean, r.min, r.max, r.conf, r.n, r.invalid, assumed);
}

} // namespace

void sensorNoiseTest() {
    loc::RobotConfig cfg = loc::littleX();
    loc::Tuning tuning;
    int count = std::min((int)cfg.sensors.size(), kMaxSensors);
    pros::Distance* devices[kMaxSensors] = {};
    for (int i = 0; i < count; i++) devices[i] = distanceSensorFor(cfg.sensors[i].name);

    Run runs[kMaxRuns];
    int runCount = 0;

    tuningToolActive = true; // pauses the controller pose line
    controller.clear();
    pros::delay(60);
    controller.print(0, 0, "NOISE TEST     ");
    pros::delay(60);
    controller.print(1, 0, "A measure B exit");
    screenPrint(5, "NOISE TEST: A measure, X clear, B exit");
    screenPrint(6, "keep the robot still while measuring");

    while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            runCount = 0;
            std::printf("noise test: runs cleared\n");
            screenPrint(6, "runs cleared");
        }
        if (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::delay(20);
            continue;
        }

        // ---- measure
        Stats stats[kMaxSensors];
        bool aborted = false;
        for (int k = 0; k < kSamples && !aborted; k++) {
            for (int i = 0; i < count; i++) {
                if (!devices[i]) continue;
                int mm = devices[i]->get();
                int conf = devices[i]->get_confidence();
                if (mm > 0 && mm < 9999 && mm != PROS_ERR) stats[i].add(mm / 25.4, conf);
                else stats[i].invalid++;
            }
            if (k % 10 == 0) {
                double left = (kSamples - k) * kSampleMs / 1000.0;
                controller.print(0, 0, "measuring %.1fs  ", left);
                screenPrint(6, "measuring... %.1f s left (B aborts)", left);
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) aborted = true;
            pros::delay(kSampleMs);
        }
        if (aborted) break;

        // ---- results
        std::printf("\nnoise test run:\n");
        char sigmaLine[48] = "sigma in:", pctLine[48] = "% of dist:";
        int shown = 0;
        for (int i = 0; i < count; i++) {
            const Stats& s = stats[i];
            const char* name = cfg.sensors[i].name;
            bool usable = s.n >= 10;
            if (usable && runCount < kMaxRuns) {
                runs[runCount] = {name, s.mean, s.sigma(), s.min, s.max, s.confSum / s.n, s.n, s.invalid};
                printRun(runs[runCount], tuning);
                runCount++;
            } else if (!usable) {
                std::printf("  %-6s no reading (%d invalid)\n", name, s.invalid);
            }

            // Brain: one short entry per sensor, by first letter.
            char entry[16];
            std::snprintf(entry, sizeof(entry), usable ? " %c %.3f" : " %c --", name[0], s.sigma());
            std::strncat(sigmaLine, entry, sizeof(sigmaLine) - std::strlen(sigmaLine) - 1);
            std::snprintf(entry, sizeof(entry), usable ? " %c %.2f" : " %c --", name[0], 100 * s.sigma() / s.mean);
            std::strncat(pctLine, entry, sizeof(pctLine) - std::strlen(pctLine) - 1);

            // Controller: one line per sensor (first three), e.g. "front 45.2 s0.08".
            if (shown < 3) {
                pros::delay(60); // the controller drops prints sent closer than ~50 ms
                bool pastRange = usable && s.mean > cfg.sensors[i].maxRange;
                if (usable) controller.print(shown, 0, "%-5.5s%5.1f s%.2f%s", name, s.mean, s.sigma(), pastRange ? "!" : " ");
                else controller.print(shown, 0, "%-5.5s no read   ", name);
                shown++;
            }
        }
        screenPrint(5, "%s", sigmaLine);
        screenPrint(6, "%s", pctLine);
    }

    // ---- summary
    std::printf("\n--- sensor noise test: %d runs ---\n", runCount);
    for (int i = 0; i < runCount; i++) printRun(runs[i], tuning);

    double worstClose = -1, worstFrac = -1;
    for (int i = 0; i < runCount; i++) {
        if (runs[i].mean <= kCloseIn) worstClose = std::max(worstClose, runs[i].sigma);
        else worstFrac = std::max(worstFrac, runs[i].sigma / runs[i].mean);
    }
    std::printf("\nnow in robot_config.hpp: distSigmaMin %.2f in, distSigmaFrac %.3f\n", tuning.distSigmaMin,
                tuning.distSigmaFrac);
    if (worstClose >= 0) std::printf("measured: worst sigma closer than %.0f in = %.3f in  -> distSigmaMin\n", kCloseIn, worstClose);
    else std::printf("measured: no runs closer than %.0f in, so nothing to say about distSigmaMin\n", kCloseIn);
    if (worstFrac >= 0) std::printf("measured: worst sigma / distance past %.0f in = %.4f  -> distSigmaFrac\n", kCloseIn, worstFrac);
    else std::printf("measured: no runs past %.0f in, so nothing to say about distSigmaFrac\n", kCloseIn);
    std::printf("These are with the robot still. Driving adds vibration, angled beams and lag, so set the\n"
                "filter's values somewhat above these (1.5-2x is a reasonable start).\n");

    screenPrint(5, "noise test done, summary in terminal");
    screenPrint(6, "");
    controller.clear();
    tuningToolActive = false;
}
