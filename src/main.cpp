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

pros::MotorGroup cascade ({1, -2}, pros::v5::MotorGears::blue);//2 11w
pros::Rotation cascade_sensor(-3);
lemlib::PID cascade_pid(250,-.1,150,10); // old (5,0,20) per degree, scaled to per inch
float getCascadeInches() {
    return (cascade_sensor.get_position() / 100.0f) * INCHES_PER_DEGREE;
}
void moveCascadeTo(double targetInches, int timeoutMs) {
    cascade_pid.reset();
    uint32_t start = pros::millis();
    
    while (true) {
        float current_pos_inches = getCascadeInches();
        float error = targetInches - current_pos_inches;
        float output = cascade_pid.update(error);

        output = std::clamp(output, -127.0f, 127.0f);
        cascade.move(output);

        // stop moving if the target reached or if timer run out
        if (std::abs(error) < .3 || pros::millis() - start > (uint32_t)timeoutMs) {
            cascade.set_brake_mode(pros::MotorBrake::hold);
            cascade.move_velocity(0);
            break;
        }
        pros::delay(10);
    }
}

// TODO: placeholder port/gearset -- update to the real toggles port(s)
bool clawBool = false;
pros::adi::DigitalOut clawShut(7, clawBool);
// TODO: placeholder ADI ports -- update to the real toggle ports
bool flipBool = false;
pros::adi::DigitalOut toggles(8, false);

pros::Distance claw_sensor(9);
// pros::Distance backTopSensor(8);

pros::MotorGroup arm_motor ({6, -7}, pros::v5::MotorGears::green);//2 5.5
pros::Rotation arm_sensor(10);
lemlib::PID arm_pid(8,0,150,0);     
lemlib::PID arm_hold_pid(7,0,50,0); 

void moveArmTo(double target, int timeoutMs, float maxSpeed) {
    arm_pid.reset();
    uint32_t start = pros::millis();
    while (true) {
        float current_pos = arm_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = arm_pid.update(error);
        
        output = std::clamp(output, -maxSpeed, maxSpeed); 
        
        arm_motor.move(output);
        if (std::abs(error) < 15.0 || pros::millis() - start > (uint32_t)timeoutMs) {
            arm_motor.move(0);
            break;
        }
        pros::delay(10);
    }
}

void moveArmToHold(double target, int timeoutMs, float maxSpeed) {
    arm_hold_pid.reset();
    uint32_t start = pros::millis();
    while (true) {
        float current_pos = arm_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = arm_hold_pid.update(error);
        
        output = std::clamp(output, -maxSpeed, maxSpeed); 

        arm_motor.move_velocity(output);
        if (std::abs(error) < 15.0 || pros::millis() - start > (uint32_t)timeoutMs) {
            arm_motor.set_brake_mode(pros::MotorBrake::hold);
            arm_motor.move_velocity(0);
            break;
        }
        pros::delay(10);
    }
}


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
int pressNum = 0;
// Cleared by turnTunerAuton() so the screen task stops overwriting its readout.
bool printingDistances = true;
bool armNeedsDown = true;
bool isScoring = false;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    registerBrainAutonSelector(); // brain left/right buttons pick the auton
    chassis.calibrate(); // calibrate sensors
    cascade_sensor.reset_position();
    arm_sensor.reset_position();
    arm_motor.set_brake_mode(pros::MotorBrake::hold);
    cascade.set_brake_mode(pros::MotorBrake::hold);

    arm_motor.move_velocity(0);
    cascade.move_velocity(0);

    // Start with claw clamped so we can have a pin.
    clawBool = true;
    clawShut.set_value(true);

    // So we aren't touching the toggle when match starts
    toggleMechUp();

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
            if (!pickingAuton && !fieldDisabled && !offsetTunerActive &&
                pros::millis() - lastControllerPrint >= 60) {
                controller.print(2, 0, "%5.1f %5.1f %3.0f ", pose.x, pose.y, heading);
                lastControllerPrint = pros::millis();
            }
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", heading); // heading
            pros::lcd::print(3, "Front sensor: %.2f", (double)(frontSensor.get() / 25.4));
            pros::lcd::print(4, "Cascade (in): %.3f", (double)getCascadeInches());
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

    pros::Task armTask([&]() {
        while (pickingAuton) pros::delay(20); // driver buttons do nothing until X
        while (true) {
            // auton drives the arm itself; R1Button's arm hold would fight it.
            // Also skip while disabled on a switch/field: the controller arrows are picking
            // the auton then, and Right would otherwise open the claw and drop the preload.
            // Same for offsetTuner(), which uses the controller buttons.
            bool fieldDisabled = pros::competition::is_connected() && pros::competition::is_disabled();
            if (autonRunning || fieldDisabled || offsetTunerActive) {
                pros::delay(20);
                continue;
            }
            L1Button();

            L2Button();

            R1Button();

            RightArrowButton();
            

            pros::delay(50);
            }
        });

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

    /*
    // loop forever
    // autonomous();

    // rotationChassisPID();
    // linearChassisPID();
    
    // return;
    // while(true){
    //     arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //     arm_motor.move_velocity(0);
    // }
    */

    // spinTest();
    // offsetTuner(); // distance sensor offsets: robot square on x = 0 or y = 0, B to exit

    // tuningPID();
    // pros::delay(10000);
    distanceReset();
    poseFusionEnabled=false;
    ukfEnabled = true; // UKF keeps running in driver control
    // tuningPID();
    // pros::delay(1000000);
    moveArmTo(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight, 800);
    
    // autonomous();
    while (true) {

        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        //     moveCascadeTo(10);
        // }else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        //     moveCascadeTo(0);
        // }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            distanceReset();
            // moveCascadeTo(6,5000);
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            distanceReset();
            // moveCascadeTo(0,5000);
        }

        if (!isScoring){
            double cascadePos = getCascadeInches();
            
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
                cascade.move(127); // Move Up
            }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
                // Only allow reverse if it is above 0
                if (cascadePos > 0.011) {
                    cascade.move(-127);
                } else {
                    cascade.move(0); // Lock it at the bottom
                }
            }
            else{
                cascade.move(0); // Brake mode HOLD kicks in here
            }
        }
        
        toggleMech();
        
        // delay to save resources
        pros::delay(25);
    }
}

void L1Button() {
    static bool l1WasPressed = false;
    static bool autoClampedThisPress = false;

    bool l1CurrentlyPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    // check if need to move arm back down after prime
    if (l1CurrentlyPressed && !l1WasPressed) {
        if (pressNum == 1) {
            // abort the score priming
            pressNum = 0; 
            
            moveCascadeTo(cascadeRestHeight, 600); 
            
            moveArmTo(downArmDegPinAndCup, 2000, 127.0f); 
            
            armNeedsDown = true;
            
            autoClampedThisPress = true; 
        } else {
            autoClampedThisPress = false; 
        }
    }
    
    // auto-clamp when held down
    if (l1CurrentlyPressed) {
        if (clawBool == false && !autoClampedThisPress) {
            if (claw_sensor.get_distance() < 50) {
                autoClampedThisPress = true;
                isScoring = true;
                
                // Lower cascade and arm
                clampClaw();
                pros::delay(150);
                
                // return to resting height
                moveCascadeTo(cascadeRestHeight, 600); 
                moveArmTo(downArmDegPinAndCup, 600, 127.0f); 
                armNeedsDown = true; 
                
                isScoring = false;
            }
        }
    }

    // Manual clamp/release
    if (!l1CurrentlyPressed && l1WasPressed) {
        if (!autoClampedThisPress) {
            
            // if it's open then manual clamp
            if (clawBool == false) {
                isScoring = true;
                
                clampClaw();
                pros::delay(150); 
                // return to resting height
                moveCascadeTo(cascadeRestHeight, 600); 
                moveArmTo(downArmDegPinAndCup, 600, 127.0f); 
                armNeedsDown = true;
                
                isScoring = false;
            } 
            // If closed then manually drop object (single pin scoring)
            else {
                isScoring = true;
                
                // open claw
                clawBool = false;
                clawShut.set_value(false);
                
                isScoring = false;
            }
        }
    }

    l1WasPressed = l1CurrentlyPressed;
}

void L2Button() {
    if (clawBool == true && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        isScoring = true;

        // lower cascade to 0
        moveCascadeTo(cascadeClampingHeight, 600);
        moveArmTo(downArmDegJustPin, 600);

        // open claw for a small interval to release cup
        clawShut.set_value(false);
        pros::delay(75);
        clawShut.set_value(true);
        pros::delay(150);
        
        // return arm to resting height
        moveCascadeTo(cascadeRestHeight, 600);
        moveArmTo(downArmDegPinAndCup, 600);
        armNeedsDown = true;
        
        isScoring = false;
    }
}

void R1Button(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){//ARM movement

        if(pressNum == 0){
            // arm_hold_pid.reset();
            armNeedsDown = false;
            pressNum = 1;
            moveArmToHold(450);
            screenPrint(5, "priming...");
        }else if (pressNum==1){
            isScoring = true;
            screenPrint(5, "scoring...");
            cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            //moveCascadeTo(660);
            cascade.move_velocity(0);
            if (getCascadeInches() < 3) {
                moveArmTo(650, 1000, 95.0f);
            } else {
                moveArmTo(600, 1000, 95.0f);
            }
            
            pressNum=0;
            clawShut.set_value(false);
            clawBool=false;
            armNeedsDown = true;
            cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            pros::delay(100);
            moveCascadeTo(getCascadeInches() + cascadeLiftOffset, 500);
            pros::delay(150);
            pros::Task armReturn([]() {
                moveArmToHold(downArmDegPinAndCup);
            });
            if (getCascadeInches() > 15) {
                pros::delay(250);
            } else {
                pros::delay(150);
            }
            moveCascadeTo(cascadeRestHeight);
            // wait for the arm so the armNeedsDown hold below doesn't stop it mid-move
            armReturn.join();
            isScoring = false;
        }
        //if(armNeedsDown) moveArmTo(downArmDegPinAndCup+50);
    }
     if(armNeedsDown) {
        arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        arm_motor.move_velocity(0);
     }
};

void RightArrowButton(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){//ARM movement
        isScoring = true;
                
        // lower cascade
        moveCascadeTo(cascadeClampingHeight, 600);
        
        // open claw
        clawBool = false;
        clawShut.set_value(false);

        // wait for driver to back away
        double startPosition = left_motors.get_position(); 
        uint32_t waitStart = pros::millis();
        
        while (true) {
            double currentPosition = left_motors.get_position();
            double degreesMoved = std::abs(currentPosition - startPosition);
            
            // timeout if takes too long
            if (degreesMoved > 150.0 || pros::millis() - waitStart > 1000) {
                break;
            }
            pros::delay(10); 
        }
        
        // return to resting height
        moveCascadeTo(cascadeRestHeight, 600);
        armNeedsDown = true;
        
        isScoring = false;
    }
};

void toggleMech(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        toggles.set_value(true);
        // pros::delay(1000);
    }else{
        toggles.set_value(false);
    }
};

void toggleMechUp(){
    toggles.set_value(true);
};

void toggleMechDown(){
    toggles.set_value(false);
};
void clampClaw(){
    // lower the cascade
    
    pros::Task armTask([&]() {
        moveCascadeTo(cascadeClampingHeight, 400); 
        moveArmTo(downArmDegPinAndCupClamping, 600, 30.0f);
    });
    pros::delay(300);
    // grab object
    clawBool = true;
    clawShut.set_value(true);
    // pros::delay(150); 
}