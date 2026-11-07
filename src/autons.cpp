#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include <algorithm>



void movementRightSide(){
    //Go score the preloaded pin
    chassis.turnToPoint(47, 18, 500, {});//uncommented to make faster

    chassis.moveToPoint(47, 17.8, 1000);

    // Get out of the way and face towards the first cup/pin
    chassis.turnToPoint(47, 10, 200, {.forwards = false});
    chassis.moveToPoint(47,10, 400, {.forwards = false});

    chassis.turnToPoint(29.5, 19, 700);

    // Get the first cup/pin and get it
    chassis.moveToPoint(29.5, 19, 2000);

    pros::delay(250);
    
    // Move to goal, move arm to back, and score
    chassis.turnToPoint(35, 30, 500, {.forwards = false, .maxSpeed= 60});
    chassis.moveToPoint(35, 30, 500, {.forwards = false});

    
    chassis.moveToPoint(44, 25, 800, {.forwards = false}, false);
    pros::delay(850);
    

    // second cup/pin and get it
    chassis.turnToPoint(47, 38, 2000);
    chassis.moveToPoint(47, 38, 2000);
    // pros::delay(1000);

    // // chassis.turnToHeading(315, 1000, {.maxSpeed = 30});
    chassis.turnToPoint(47, 30, 1000, {.forwards=false});
    chassis.moveToPoint(47, 30, 2000, {.forwards=false});
    pros::delay(500);
    //make cascade go higher here
}

void EastWestAuton() {
    // Toggle flip
    //move this back when there is wall - testing without wall
    // chassis.setPose(61.5,0,90);
    // TIMING: ms since auton start, on brain line 5 and the terminal. Remove once the start pause is found.
    distanceReset(90,true,false);
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);

    chassis.moveToPoint(47, 0, 1000, {.forwards = false});
    movementRightSide();

}

void NorthSouthAuton() {
    // Toggle flip
    //move this back when there is wall - testing without wall

    distanceReset(0,false,true);
    // pros::delay(10000);
    chassis.moveToPoint(0, 58, 400, {.forwards = false});
    chassis.moveToPoint(0, 72, 300);
    chassis.moveToPoint(0, 58, 400, {.forwards = false});
    chassis.moveToPoint(0,72, 300);

    //getting ready to score preload
    chassis.moveToPoint(0, 47, 1000, {.forwards = false});
    //Go score the preloaded pin
    chassis.turnToPoint(18, 47, 500, {});//uncommented to make faster
    chassis.moveToPoint(18, 47, 1000);

    //score
    pros::delay(500);

    chassis.turnToPoint(10, 47, 200, {.forwards = false});
    pros::delay(150);

    // Get out of the way and face towards the first cup/pin
    chassis.moveToPoint(10,47, 400, {.forwards = false});

    //move so that we can swing to position
    chassis.turnToPoint(13, 58.5, 1000);
    chassis.moveToPoint(13, 58.5, 2000);           //can probably motion change this part
    chassis.turnToPoint(36, 58.5, 2000);
    chassis.moveToPoint(36, 58.5, 2000, {.earlyExitRange=1});
    chassis.swingToPoint(23, 60, lemlib::DriveSide::RIGHT, 700, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    //grab the middle stack
    chassis.turnToPoint(23, 59, 1000);
    chassis.moveToPoint(23, 59.1, 1000, {}, false);
    // pros::delay(10000);
    // chassis.arcade(20, 0);//make it keep going forward?
    chassis.arcade(30, 0);//make it keep going forward?
    pros::delay(250);
    chassis.moveToPoint(24, 50, 1000, {.forwards = false});
    //score the stack
    // pros::delay(250);
    pros::delay(2500);
    
    //go to the ohter side
    chassis.swingToPoint(-32, 56, lemlib::DriveSide::LEFT, 1000);

    chassis.moveToPoint(10, 56, 1000, {.minSpeed=70, .earlyExitRange=3});
    chassis.moveToPoint(-35, 56, 2000);

    //swing to correct heading
    chassis.swingToPoint(-24, 60, lemlib::DriveSide::LEFT, 700, {.direction =lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.turnToPoint(-24, 58, 500);
    chassis.moveToPoint(-24, 58, 1000);

    // pros::delay(10000);
    chassis.arcade(30, 0);//make it keep going forward?
    pros::delay(250);
    //go into goal and score 
    chassis.moveToPoint(-24, 50, 1000, {.forwards = false});
    pros::delay(250);


}


void SkillsAuton() {

    distanceReset(90,true,false);
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    // pros::delay(1000000);
    chassis.moveToPoint(72, 0, 300);
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);

    chassis.moveToPoint(47, 0, 1000, {.forwards = false});

    //Go score the preloaded pin
    chassis.turnToPoint(47, -16, 500, {});//uncommented to make faster

    chassis.moveToPoint(47, -16, 1000);
    pros::delay(500);//             do we need this one?

    pros::delay(350);

    // Get out of the way and go towards match loader
    chassis.turnToPoint(47, -10, 500, {.forwards = false});
    chassis.moveToPoint(47,-10, 500, {.forwards = false});
    chassis.turnToPoint(58, -17, 500);
    chassis.moveToPoint(58,-17, 500);
    chassis.turnToPoint(60, -30, 500);
    chassis.moveToPoint(60,-30, 500);

    // CLEARING
    chassis.turnToPoint(35, -59, 1000, {.forwards = false});
    chassis.moveToPoint(35, -59, 2000, {.forwards = false});
    chassis.turnToHeading(90, 1000);
    distanceReset(NAN, true, true, 5, &leftSensor); // left sensor skipped 
    // Getting to matchloader
    chassis.turnToPoint(58.5, -58, 2000);
    chassis.moveToPoint(58.5, -58, 2000);
    // chassis.turnToPoint(58.5, -58, 2000);
    

    for(int i = 1; i <= 4; i++){
        // Moving to goal
        chassis.turnToPoint(47, -58, 1000, {.forwards = false});
        chassis.moveToPoint(47,-58, 1000, {.forwards = false});
        pros::delay(200);
        chassis.turnToPoint(47, -28, 1000, {.forwards = false});
        chassis.moveToPoint(47,-28, 1000, {.forwards = false}, false);

        // Scoring
        distanceReset(NAN, true, true, 5, &rightSensor); // right sensor skipped
        pros::delay(250);
        if(i == 4) break;

        // Get back in front of matchloader
        chassis.turnToPoint(47, -58, 1000);
        chassis.moveToPoint(47, -58, 1000);

        // Go into matchloader and get the next stack
        chassis.turnToPoint(59, -58, 1000);
        chassis.moveToPoint(59, -58, 1000);
        // chassis.turnToPoint(59, -58, 1000);

        // chassis.arcade(40,0);
        
        pros::delay(250);

    }

    //go park
    chassis.turnToPoint(47, -38, 1000);
    chassis.moveToPoint(47,-38, 1000);
    chassis.turnToPoint(6, -5, 1000, {.forwards=false});
    chassis.moveToPoint(6,-5, 2000, {.forwards=false});


    // // Get out of way.
    // chassis.turnToPoint(47, -40, 1000);
    // chassis.moveToPoint(47, -40, 1000);

    // // move to clear second batch
    // chassis.turnToPoint(28, -24, 1000, {.forwards = false});
    // chassis.moveToPoint(28,-24, 1000, {.forwards = false});

    // // get stack from mid
    // chassis.turnToPoint(24, -8, 1000);
    // chassis.moveToPoint(24, -8, 1000);
    // searchForStack(1000, cascadeRestHeight);
    // pros::delay(250);

    // seperatePin();
    
    // chassis.moveToPoint(47, 0, 1000, {.forwards = false}, false);

    // movementRightSide();

    // for (int i = 1; i <= 2; i++){
    //     // traveling to matchload
    //     chassis.turnToPoint(47, 58.5, 1000);
    //     chassis.moveToPoint(47, 58.5, 1000);
    //     chassis.turnToPoint(61, 58.5, 1000);
    //     chassis.moveToPoint(61, 58.5, 1000);
    //     searchForStack(1000, 2 * i);
    //     pros::delay(250);

    //     // Going to goal
    //     chassis.turnToPoint(47, 58.5, 1000, {.forwards = false});
    //     chassis.moveToPoint(47, 58.5, 1000, {.forwards = false});
    //     moveCascadeTo((i+2)*2);     //prob need to change this, i+2 cause there should be two cups already right?
    //     chassis.turnToPoint(47, 31, 1000, {.forwards = false});
    //     chassis.moveToPoint(47, 31, 1000, {.forwards = false});
    //     moveArmToBack();
        
    //     openClawAndLiftThenPutBackFront(true);
    //     pros::delay(250);
    // }
    // // prepare for middle
    // chassis.turnToPoint(47, 47, 1000);
    // chassis.moveToPoint(47, 47, 1000);

    // // going to middle
    // chassis.turnToPoint(8, 8, 1000, {.forwards = false});
    // chassis.moveToPoint(8, 8, 1000, {.forwards = false});
    // pros::delay(1000);
}
// ---------------------------------------------------------------------------
// Test / tuning routines
// ---------------------------------------------------------------------------

void darwin_test() {
    int counter = 0;
    chassis.setPose(0, 0, 0);
    int secondVar = 0;
    while (frontSensor.get() > 500) {
        chassis.arcade(127, 0);
        counter++;
        screenPrint(5, "%d sec: %f", counter, chassis.getPose().y);
        pros::delay(100);
    }
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.arcade(0, 0);
}

void frictiontest() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.arcade(127, 0);
    pros::delay(500);
    chassis.arcade(0, 0);
    screenPrint(6, "Final Velocity: %.4f", vertical_rotation_sensor.get_velocity());
    int count = 0;
    while (vertical_rotation_sensor.get_velocity() > 1) {
        count++;
        pros::delay(50);
    }
    screenPrint(7, "Time till stop: %.4f seconds",
                     vertical_rotation_sensor.get_velocity() * 20); // 20 = 1000/50 or 1 second / milli
}

void torquetest() {
    int torqueCount = 47;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            torqueCount += 10;
            if (torqueCount > 127) torqueCount = 127;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            torqueCount -= 10;
            if (torqueCount < 0) torqueCount = 0;
        }
        screenPrint(5, "Current voltage: %d", torqueCount);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            left_motors.move(torqueCount);
        } else {
            left_motors.move(0);
        }
    }
}

void tuningPID(){
    chassis.turnToHeading(90, 2000);
    pros::delay(4000);
    chassis.turnToHeading(180, 2000);
    pros::delay(4000);
    chassis.turnToHeading(270, 2000);
    pros::delay(4000);
    chassis.turnToHeading(0, 2000);

    // chassis.moveToPoint(0,12, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,24, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,48, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,0, 2000, {.forwards=false});


}

// IMU scale test. Blocks opcontrol until B is pressed: only right-stick
// turning works while it runs. Line the robot up against a wall/tile edge,
// press A to zero, spin in place with the right stick (or by hand), and line
// it back up after some whole number of turns. The screen task in main.cpp
// shows the rotation and the scale factor (true degrees / IMU degrees).
// This never prints to the brain screen itself: LVGL isn't thread-safe, and
// printing from here and the screen task at once crashes the brain.
bool spinTestActive = false;
double spinTestRotation = 0;
double spinTestPeakRate = 0; // fastest turn rate seen since the last zero, deg/s

void spinTest() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // Use an offset instead of tare_rotation() so LemLib's odom isn't disturbed.
    double start = imu.get_rotation();
    spinTestRotation = 0;
    spinTestPeakRate = 0;
    spinTestActive = true;
    while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            start = imu.get_rotation();
            spinTestPeakRate = 0;
        }
        chassis.arcade(0, controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        spinTestRotation = imu.get_rotation() - start;
        double rate = std::fabs(imu.get_gyro_rate().z);
        if (std::isfinite(rate) && rate > spinTestPeakRate) spinTestPeakRate = rate;
        pros::delay(20);
    }
    chassis.arcade(0, 0);
    spinTestActive = false;
}

// Scratch route for trying things out; the "Testing" entry in the selector runs it.
void testing(){
    // tuningPID();
}

// ---------------------------------------------------------------------------
// Auton selector -- shown on controller line 2 and brain line 6. Left/Right on
// the controller or the brain's left/right buttons to choose; both stay in sync.
//   1st X: locks in the route and sets its start pose. Odometry keeps
//          tracking from there, so the robot can be pushed into place.
//   2nd X: starts the route (only without a competition switch/field; on a
//          switch/field, autonomous() starts it instead).
// To add a route, write the function above and add a line here with its start
// pose (keep Testing last).
// ---------------------------------------------------------------------------

struct AutonRoute {
    const char* name;
    void (*run)(); // nullptr = no auton, straight to driver control
    float x, y, theta; // start pose set on the first X press
};

AutonRoute autons[] = {
    {"Driver", nullptr, 0, 0, 0},
    {"EastWest Auton", EastWestAuton, 61.5, 0, 90},//in the future change this value, cause this is not the correct starting position
    {"NorthSouth Auton", NorthSouthAuton, 0, 61.5, 0},//in the future change this value, cause this is not the correct starting position
    {"Skills Auton", SkillsAuton, 61.5, 0, 90},
    {"Testing", testing, 0, 0, 0},
};
constexpr int autonCount = sizeof(autons) / sizeof(autons[0]);

int selectedAuton = 0; // starts on Driver
bool pickingAuton = true; // true from boot so nothing runs before the picker is done
bool autonLocked = false; // set once the route is locked in or started; selection stops changing

// Moves the selection, clamped to the list, and shows it on the brain. Called from
// the controller selector and the brain button callbacks, which run in different tasks.
void selectAuton(int index) {
    if (autonLocked) return;
    selectedAuton = std::clamp(index, 0, autonCount - 1);
    screenPrint(6, "Auton: %s", autons[selectedAuton].name);
}

// Brain buttons: left/right step through the list, center does nothing for now.
// These only change the selection; the screen task does the drawing (LVGL isn't thread-safe).
static void brainLeftButton() { selectAuton(selectedAuton - 1); }
static void brainRightButton() { selectAuton(selectedAuton + 1); }

void registerBrainAutonSelector() {
    pros::lcd::register_btn0_cb(brainLeftButton);
    pros::lcd::register_btn2_cb(brainRightButton);
    selectAuton(selectedAuton); // show the starting choice
}

// Controller Left/Right change the selection, and line 2 follows it (including
// changes from the brain buttons). The controller drops prints sent faster than
// ~50ms apart, so only print on a change, plus a slow refresh in case one got dropped.
static void pollControllerSelector(int& shown, uint32_t& lastPrint) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        selectAuton(selectedAuton - 1);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        selectAuton(selectedAuton + 1);
    }
    if (selectedAuton != shown || pros::millis() - lastPrint > 250) {
        controller.print(2, 0, "Auton: %-12s", autons[selectedAuton].name);
        shown = selectedAuton;
        lastPrint = pros::millis();
    }
}

void autonSelector() {
    pickingAuton = true; // screen task stops printing to controller line 2
    pros::delay(100);

    // step 1: pick the route, first X locks it in
    int shown = -1;
    uint32_t lastPrint = 0;
    while (true) {
        // on a competition switch/field, competition_initialize() and autonomous()
        // take over, so don't hold up initialize()
        if (pros::competition::is_connected()) {
            pickingAuton = false;
            return;
        }
        pollControllerSelector(shown, lastPrint);
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            break;
        }
        pros::delay(20);
    }

    autonLocked = true;
    AutonRoute& route = autons[selectedAuton];
    screenPrint(6, "Auton: %s (locked)", route.name);

    // step 2: set the start pose, then wait for the second X to start.
    if (route.run) {
        chassis.setPose(route.x, route.y, route.theta);
        pros::delay(60);
        controller.print(2, 0, "X to start    ");
        uint32_t lastPrint = pros::millis();
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // plugged into a switch/field after locking in: autonomous() starts the route
            if (pros::competition::is_connected()) break;
            if (pros::millis() - lastPrint > 250) {
                controller.print(2, 0, "X to start    ");
                lastPrint = pros::millis();
            }
            pros::delay(20);
        }
    }

    pickingAuton = false;
}

void competitionAutonSelector() {
    int shown = -1;
    uint32_t lastPrint = 0;
    // PROS ends this task when the robot is enabled
    while (true) {
        pollControllerSelector(shown, lastPrint);
        pros::delay(20);
    }
}

bool autonRunning = false;

void runAuton() {
    autonRunning = true; // keeps the driver button task off the arm/claw
    if (autons[selectedAuton].run) autons[selectedAuton].run();
    autonRunning = false;
}

void runSelectedAuton() {
    AutonRoute& route = autons[selectedAuton];
    // if it was locked in with X, the pose was already set there and odom has tracked since
    if (!autonLocked && route.run) chassis.setPose(route.x, route.y, route.theta);
    autonLocked = true;
    runAuton();
}
