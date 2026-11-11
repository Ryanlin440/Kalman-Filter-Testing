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
////////////////////////////////////////////////////////////////////////////////////
pros::MotorGroup right_motors({11, 12, -13}, 
    pros::MotorGearset::blue); 
pros::MotorGroup left_motors({-20, -19, 18},
     pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&left_motors, 
                              &right_motors,
                              12.6,
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2
);
////////////////////////////////////////////////////////////////////////////////////
pros::Distance frontSensor(15);
pros::Distance leftSensor(17);
pros::Distance rightSensor(8);
pros::Distance claw_sensor(9);
////////////////////////////////////////////////////////////////////////////////////

ScaledImu imu(5, 1.0032, 1.0047); // left, right scales from spinTest(); right averages half + full speed runs
// ScaledImu imu(5, 1.0000, 1.0000); // left, right scales from spinTest(); right averages half + full speed runs

////////////////////////////////////////////////////////////////////////////////////
pros::Rotation vertical_rotation_sensor(-21);
pros::Rotation horizontal_rotation_sensor(4);
lemlib::TrackingWheel vertical_tracking_wheel(
    &vertical_rotation_sensor, 
    lemlib::Omniwheel::NEW_2, -.55);
lemlib::TrackingWheel horizontal_tracking_wheel(
    &horizontal_rotation_sensor, 
    lemlib::Omniwheel::NEW_2, -1.5);
////////////////////////////////////////////////////////////////////////////////////
pros::MotorGroup cascade ({1, -2}, pros::v5::MotorGears::blue);
pros::Rotation cascade_sensor(-3);
lemlib::PID cascade_pid(250,-.1,150,10);
////////////////////////////////////////////////////////////////////////////////////

float getCascadeInches() {
    return (
        cascade_sensor.get_position() / 100.0f
    ) * INCHES_PER_DEGREE;
}
void moveCascadeTo(double targetInches, int timeoutMs) {
    cascade_pid.reset();
    uint32_t start = pros::millis();
    
    while (true) {
        float current_pos_inches = getCascadeInches();
        float error = targetInches - current_pos_inches;
        float output = cascade_pid.update(error);

        output = std::clamp(
            output, -127.0f, 127.0f
        );
        cascade.move(output);

        // Stop moving if the target reached
        // or if timer runs out
        if (std::abs(error) < .3 || 
            pros::millis() - start > (uint32_t)timeoutMs) {

            cascade.set_brake_mode(
                pros::MotorBrake::hold
            );
            cascade.move_velocity(0);
            break;
        }
        pros::delay(10);
    }
}
////////////////////////////////////////////////////////////////////////////////////
bool clawBool = false;
bool flipBool = false;
pros::adi::DigitalOut clawShut(7, clawBool);
pros::adi::DigitalOut toggles(8, flipBool);
////////////////////////////////////////////////////////////////////////////////////
pros::MotorGroup arm_motor ({6, -7},
     pros::v5::MotorGears::green);
pros::Rotation arm_sensor(10);

lemlib::PID arm_pid(8,0,150,0);     
lemlib::PID arm_hold_pid(7,0,50,0); 
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
void moveArmTo(double target, int timeoutMs, float maxSpeed) {
    arm_pid.reset();
    uint32_t start = pros::millis();
    while (true) {
        float current_pos = arm_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = arm_pid.update(error);
        
        output = std::clamp(output, -maxSpeed, maxSpeed); 
        
        arm_motor.move(output);
        if (std::abs(error) < 15.0 || 
            pros::millis() - start > (uint32_t)timeoutMs) {
            arm_motor.move(0);
            break;
        }
        pros::delay(10);
    }
}
////////////////////////////////////////////////////////////////////////////////////

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
////////////////////////////////////////////////////////////////////////////////////
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    7.5,
    -.0005,
    35,
    0,
    1, 
    100,
    3,
    500, 
    0
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    1.6, 
    .00,
    9,
    0,
    1,
    100,
    3,
    300,
    0
);
////////////////////////////////////////////////////////////////////////////////////
lemlib::ExpoDriveCurve throttle_curve(3, 
                                     10, 
                                     1.025 
);

lemlib::ExpoDriveCurve steer_curve(3,
                                  10,
                                  1.025
);
////////////////////////////////////////////////////////////////////////////////////
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
////////////////////////////////////////////////////////////////////////////////////


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
bool printingDistances = true;
bool armNeedsDown = true;
bool isScoring = false;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
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
            /*
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
            */
            double heading = std::fmod(chassis.getPose().theta, 360.0);
            if (heading < 0) heading += 360.0;
            if (!pickingAuton) controller.print(2, 0, "%.4f", heading);
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

    startPoseFusion(); // blends distance sensors into odom; gains on line 7

    pros::Task armTask([&]() {
        while (pickingAuton) pros::delay(20); // driver buttons do nothing until X
        while (true) {
            // auton drives the arm itself; R1Button's arm hold would fight it
            if (autonRunning) {
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
    autonRunning = true; // keeps the driver button task off the arm (R1Button's hold fights moveArmTo)

    //north south
    // chassis.setPose(0, 61.5, 0); //make sure to change this everything
    // NorthSouthAuton(); //make sure to run correct one

    // //east west 
    chassis.setPose(61.5, 0, 90);  //make sure to change this everything
    EastWestAuton();//make sure to run correct one

    // //Skills 
    // chassis.setPose(61.5, 0, 90); //make sure to change this everything
    // SkillsAuton(); //make sure to run correct one
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

    // tuningPID();
    // pros::delay(10000);
    distanceReset();
    poseFusionEnabled=false;
    // tuningPID();
    // pros::delay(1000000);
    moveArmTo(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight, 800);
    
    // autonomous();
    while (true) {

    //     // get left y and right y positions
    //     int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    //     int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    //     // move the robot
    //     chassis.arcade(leftY, rightX);
    //     // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
    //     //     moveCascadeTo(10);
    //     // }else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    //     //     moveCascadeTo(0);
    //     // }

    //     if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
    //         distanceReset();
    //         // moveCascadeTo(6,5000);
    //     }
    //     if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
    //         distanceReset();
    //         // moveCascadeTo(0,5000);
    //     }

            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                clawBool = true;
                clawShut.set_value(true);
            }

            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
                clawBool = false;
                clawShut.set_value(false);
            }


    //     if (!isScoring){
    //         double cascadePos = getCascadeInches();
            
    //         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    //             cascade.move(127); // Move Up
    //         }
    //         else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    //             // Only allow reverse if it is above 0
    //             if (cascadePos > 0.011) {
    //                 cascade.move(-127);
    //             } else {
    //                 cascade.move(0); // Lock it at the bottom
    //             }
    //         }
    //         else{
    //             cascade.move(0); // Brake mode HOLD kicks in here
    //         }
    //     }
        
    //     toggleMech();
        
    //     // delay to save resources
    //     pros::delay(25);
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
        }else if (pressNum==1){
            isScoring = true;
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



////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////


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