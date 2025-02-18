#include "main.h"
#include "drivercontrol.hpp"
#include "devices.hpp"
#include "autons.hpp"
#include "gif-pros/gifclass.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "display/lvgl.h" 

/* 12Z ez-template copied from 2145Z 2/5/25

______/\\\____/\\\\\\\\\______/\\\\\\\\\\\\\\\____________________________________/\\\\\\\\\\\\\\\________________________________________         
 __/\\\\\\\__/\\\///////\\\___\////////////\\\____________________________________\////////////\\\_________________________________________        
  _\/////\\\_\///______\//\\\____________/\\\/_______________________________________________/\\\/____________________/\\\\\\\\\____________       
   _____\/\\\___________/\\\/___________/\\\/________________/\\\\\\\\\\\___________________/\\\/______/\\\\\\\\\_____/\\\/////\\\___________      
    _____\/\\\________/\\\//___________/\\\/_________________\///////////__________________/\\\/_______\////////\\\___\/\\\\\\\\\\____________     
     _____\/\\\_____/\\\//____________/\\\/_______________________________________________/\\\/___________/\\\\\\\\\\__\/\\\//////_____________    
      _____\/\\\___/\\\/_____________/\\\/_______________________________________________/\\\/____________/\\\/////\\\__\/\\\___________________   
       _____\/\\\__/\\\\\\\\\\\\\\\__/\\\\\\\\\\\\\\\____________________________________/\\\\\\\\\\\\\\\_\//\\\\\\\\/\\_\/\\\___________________  
        _____\///__\///////////////__\///////////////____________________________________\///////////////___\////////\//__\///____________________ 
                     
*/


// Enter your autons here!
AutonFunction autonFunctions[] = {
    {"Test 24", testAuto},
    {"$Blue Rings", $blue_neg_rings},
    {"diff setup - (no stake) - Blue Rings", blue_neg_rings},
    {"red ", red_neg_rings},
    {"Solo AWP Red", soloAwpSafeRed},
    {"Solo AWP Blu", soloAwpSafeBlue},
    // {"Test 24", testAuto},
    {"Nothing", doNothingAuto},
    {"Skills2 JAR", skills2_jar}
};

// this is needed for LVGL displaying! Do not touch!
size_t autonCount = sizeof(autonFunctions) / sizeof(autonFunctions[0]);

void initialize() {
    pros::delay(750); // Stop the user from doing anything while legacy ports configure.

	// screen init
    calibrationScreenInit();

    calibrateChassis();
    
    autonSelectorScreenInit(
        // your auton funcitons
        autonFunctions, 
        // auton count needed for LVGL displaying
        autonCount, 
        // customizable color scheme, play around with it!
        LV_COLOR_MAKE(0xD2, 0x46, 0x8C)
    );
    default_constants();
    chassisInits();

    armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    armSensor.reset();

    // -- Piston Inits --
    goalClamp1.set_value(true);
    goalClamp2.set_value(true);
    doinker.set_value(false);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
} 

/**
 * Runs after initialize(), and before autonomous when connected to the Field

 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 */
void autonomous() {
    set_drive_to_hold();
    default_constants();
    runSelectedAuton(autonFunctions, autonCount);
}

/**
 * Runs the operator control code. This function will be started in its own task
 */
void opcontrol() {
    set_drive_to_coast();
    // task to make sure all motors are plugged in and check the temperature of the drivetrain
    pros::Task motorCheck(checkMotorsAndPrintTemperature);

    // Bools for toggles
    bool lastAState = false;
    bool lastBState = false;    
    bool clampEngaged = false;
    bool lastR2State = false;
    
    
	while (true) { // --------------
        // chassis.opcontrol_tank();
        chassis.opcontrol_arcade_standard(ez::SPLIT);

        // -- Arm Control --
        // Variables to store the previous states of the buttons
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !lastAState) {
            toggleA = !toggleA;
            if (toggleA) {
                moveToPositionPID(midPosition);
            } else {
                armMotor.move_velocity(80);  
                while (armSensor.get_angle() > homePosition - 300) {  
                    pros::delay(10);
                }
                disengageMotor(); 
            } }
        lastAState = master.get_digital(pros::E_CONTROLLER_DIGITAL_A); // Update last state

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !lastBState) {
            toggleB = !toggleB;
            if (toggleB) {
                moveToPositionPID(highPosition);
            } else {
                armMotor.move_velocity(80);  
                while (armSensor.get_angle() > homePosition - 500 + threshold) {  // Stop 5 degrees before
                    pros::delay(10);
                }
                disengageMotor(); 
            } }
        lastBState = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);  // Update last state

        
        // Manual control, if R1 && R2 pressing then L1 and L2
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            doinker.set_value(false);
            goalClamp1.set_value(false);
            goalClamp2.set_value(false);
            armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                armMotor.move_velocity(-127);  // Move up
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                armMotor.move_velocity(127); // Move down
            } else {
                armMotor.move_velocity(0);    // Stop if neither L1 nor L2 is pressed 
            }
        } else {
            armMotor.move_velocity(0); // Ensure motor stops if R1 and R2 are not both pressed
            
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !lastR2State) { clampEngaged = !clampEngaged; } // Toggle clamp state
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {doinker.set_value(true); }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {intake1 = 127; intake2 = 127; doinker.set_value(false); }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { intake1 = -127; intake2 = -127; doinker.set_value(false); } 
            else {intake1 = 0; intake2 = 0; doinker.set_value(false); } 
        }
           
        goalClamp1.set_value(clampEngaged);
        goalClamp2.set_value(clampEngaged);
        lastR2State = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2); // Update last state
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) { disengageMotor(); motorCheck.suspend();} // Stop remote from buzzing if motor disconnect
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) { armSensor.reset(); }

		pros::delay(10);
	}
}

// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
//     // intakeRaise.set_value(false);
//     doinker.set_value(false);

//     liftControl();
// } else {           
//     liftAutoControl(-1);
//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { intake1 = 127; intake2 = 127; doinker.set_value(true); } 
//     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {intake1 = 127; intake2 = 127; doinker.set_value(false); }
//     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { intake1 = -127; intake2 = -127; doinker.set_value(false); } 
//     else {intake1 = 0; intake2 = 0; doinker.set_value(false); }   
// }
