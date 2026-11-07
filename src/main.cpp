#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "robot.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup left_motors({-20, -19, 18}, pros::MotorGearset::blue); // right motors use 200 RPM cartridges
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.6, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// pros::Distance backSensor(20);
pros::Distance frontSensor(15);
pros::Distance leftSensor(17);
pros::Distance rightSensor(8);

// Which device goes with each sensor name in include/localization/robot_config.hpp (positions live
// there). Used by the UKF, distanceReset() and offsetTuner(). Add a line when you add a sensor.
pros::Distance* distanceSensorFor(const char* name) {
    if (std::strcmp(name, "front") == 0) return &frontSensor;
    if (std::strcmp(name, "right") == 0) return &rightSensor;
    if (std::strcmp(name, "left") == 0) return &leftSensor;
    return nullptr;
}

ScaledImu imu(5, 1.0032, 1.0047); // left, right scales from spinTest(); right averages half + full speed runs
// ScaledImu imu(5, 1.0000, 1.0000); // left, right scales from spinTest(); right averages half + full speed runs
pros::Rotation vertical_rotation_sensor(-21);
pros::Rotation horizontal_rotation_sensor(4);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor, lemlib::Omniwheel::NEW_2, -.55);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor, lemlib::Omniwheel::NEW_2, -1.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(7.5, // proportional gain (kP)
                                              -.0005, // integral gain (kI)
                                              35, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
//                                               0.01, // integral gain (kI)
//                                               30, // derivative gain (kD)
//                                               5, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// angular PID controller
lemlib::ControllerSettings angular_controller(1.6, // proportional gain (kP)
                                              .00, // integral gain (kI)
                                              9, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                            3, // large error range, in degrees
                                              300, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.025 // expo curve gain
);
//both previously 1.019
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.025 // expo curve gain
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// The brain screen (LVGL) isn't thread-safe: calling pros::lcd from two tasks
// at once corrupts it and can crash the brain (DATA ABORT in lv_obj_style.c).
// So only the screen task in initialize() draws. Everything else calls
// screenPrint(), which saves the text for the screen task to draw on lines 5-7.
pros::Mutex screenMutex;
char screenMessages[3][48] = {};

void screenPrint(int line, const char* fmt, ...) {
    if (line < 5 || line > 7) return;
    char buf[48];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    screenMutex.take();
    std::strcpy(screenMessages[line - 5], buf);
    screenMutex.give();
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// Cleared by turnTunerAuton() so the screen task stops overwriting its readout.
bool printingDistances = true;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    registerBrainAutonSelector(); // brain left/right buttons pick the auton
    chassis.calibrate(); // calibrate sensors




    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            if (not printingDistances) {
                pros::delay(40);
                continue;
            }
            if (spinTestActive) {
                double rotation = spinTestRotation;
                double turns = std::round(std::fabs(rotation) / 360.0);
                pros::lcd::print(0, "SPIN TEST  A=zero B=exit");
                pros::lcd::print(1, "Rotation: %.2f", rotation);
                pros::lcd::print(2, "Turns: %d", (int)turns);
                if (turns > 0) {
                    pros::lcd::print(3, "Scale: %.5f", turns * 360.0 / std::fabs(rotation));
                } else {
                    pros::lcd::print(3, "Scale: spin 1+ turns");
                }
                pros::lcd::print(4, "Peak rate: %.0f deg/s", spinTestPeakRate);
                pros::delay(100);
                continue;
            }
            lemlib::Pose pose = chassis.getPose();
            double heading = std::fmod(pose.theta, 360.0);
            if (heading < 0) heading += 360.0;
            // while disabled on a switch/field, competitionAutonSelector() owns controller line 2
            bool fieldDisabled = pros::competition::is_connected() && pros::competition::is_disabled();
            // pose as "x y heading"; the controller drops prints sent less than ~50 ms apart
            static uint32_t lastControllerPrint = 0;
            if (!pickingAuton && !fieldDisabled && !tuningToolActive &&
                pros::millis() - lastControllerPrint >= 60) {
                controller.print(2, 0, "%5.1f %5.1f %3.0f ", pose.x, pose.y, heading);
                lastControllerPrint = pros::millis();
            }
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", heading); // heading
            pros::lcd::print(3, "Front sensor: %.2f", (double)(frontSensor.get() / 25.4));
            char messages[3][48];
            screenMutex.take();
            std::memcpy(messages, screenMessages, sizeof(messages));
            screenMutex.give();
            for (int i = 0; i < 3; i++) pros::lcd::print(5 + i, "%s", messages[i]);
            // pros::lcd::print(4, "asdl %d", count);
            // count+=2;
            // delay to save resources
            pros::delay(40);
            //
        }
    });

    if (USE_UKF) startUkf(); // UKF owns the pose; status on line 7
    else startPoseFusion(); // blends distance sensors into odom; gains on line 7
    // startWallEstimatePrinter(); // raw distance-sensor x/y on line 7 instead (turn off fusion first)

    

    autonSelector(); 
    if (!pros::competition::is_connected()) {
        runAuton();
    }

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    competitionAutonSelector();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() { 

    poseFusionEnabled = true;
    ukfEnabled = true;
    // runs whatever the brain/controller selector shows, from its start pose in autons[]
    runSelectedAuton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void rotationChassisPID(){
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000);
    pros::delay(4000);
    chassis.turnToHeading(180, 1000);
    pros::delay(4000);
    chassis.turnToHeading(360, 1000);
    pros::delay(4000);
}

void linearChassisPID(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 2000);
    pros::delay(5000);
    chassis.moveToPoint(0, 48, 2000);
    pros::delay(5000);
    chassis.moveToPoint(0, 0, 2000, {.forwards=false});
    pros::delay(5000);
}

void opcontrol() {
    // the field kills autonomous() at 15 s, so it never gets to clear this itself
    autonRunning = false;

    poseFusionEnabled=false;
    ukfEnabled = true; // UKF keeps running in driver control
       // autonomous();
    while (true) {

        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
            
        // delay to save resources
        pros::delay(25);
    }
}

