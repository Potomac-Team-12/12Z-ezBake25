#include "main.h"
#include "drivercontrol.hpp"
#include "devices.hpp"
#include "pros/rtos.hpp"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// . . .
// 
//  DRIVER CONTROL
//
// . . .

void set_drive_to_coast() {
  left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

  right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void set_drive_to_hold() {
  left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

  right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
}

double left_curve_function(double x, double left_curve_scale) {
  if (left_curve_scale != 0) {
    // if (CURVE_TYPE)
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
    // else
    // return powf(2.718, ((abs(x)-127)*RIGHT_CURVE_SCALE)/100)*x;
  }
  return x;
}

void set_tank(int l_stick, int r_stick) {
  left_side_motors.move_voltage(l_stick * (12000.0 / 127.0));

  right_side_motors.move_voltage(r_stick * (12000.0 / 127.0));
}

void tank_drive(double curve /* default is 7 in hpp file */) {
    // Put the joysticks through the curve function
    double lYcord = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double rYcord = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    double lXCord;
    double rXCord;

    lYcord > 0 ? lXCord = abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) : lXCord = -abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    rYcord > 0 ? rXCord = abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) : rXCord = -abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    int l_stick = left_curve_function(lYcord + lXCord, curve);
    int r_stick = left_curve_function(rYcord + rXCord, curve);

    // Set robot to l_stick and r_stick, check joystick threshold, set active brake
    set_tank(l_stick, r_stick);
}


// -- Arm/Lift PID 
// Constants for lift positions
double FIRST_RING_LIFT_VALUE = 0.077 * 360 * 100; 
const double MAX_LIFT_VALUE = 0.45 * 360 * 100;

// PID Control Constants
double kP = 3.0;  // Adjust this if needed
double kI = 0.0;  // Currently no integral term
double kD = 9.0;  // Add some derivative control for damping

// PID variables
double error = 0, last_error = 0, integral = 0, derivative = 0;
double targetLiftValue = FIRST_RING_LIFT_VALUE;

// Function to stop the lift and reset PID variables
void stopLift() {
    armMotor.move_velocity(0);
    integral = 0;
    last_error = 0;
}

void liftAutoControl(double targetLiftValue = FIRST_RING_LIFT_VALUE) {
    double resetValue;

    // Control loop for lift to reach target position
      double currentLiftValue = armSensor.get_position();  // Get current lift position
      if (targetLiftValue == -1) {
          double resetValue = 0.00 * 360 * 100;  // Set lift to a reset position
          error = resetValue - currentLiftValue;
      } else {
          error = targetLiftValue - currentLiftValue;
      }
      
      // Check if the error is small enough to stop
      if (abs(error) < 1) {  
          // Stop the lift if the target position is reached
          stopLift();
          return;
      }

      // PID calculations for motor control
      integral += error;
      derivative = error - last_error;
      double motorSpeed =  ((error * kP) + (integral * kI) + (derivative * kD));  // PID control
      
      // Apply motor speed with a max speed limit
      motorSpeed = std::clamp(motorSpeed, -12000.0, 12000.0);  // Prevent overspeeding
      armMotor.move_voltage(motorSpeed);

      last_error = error;
}

ez::PID liftPID{0.055, 0, 0.375, 0, "Lift"};

void set_lift(int input) {
  armMotor.move(input);
}

void manualLiftControl() {
    double currentLiftPosition = armSensor.get_position();  // Get current lift position

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
      intake1 = 127;
      intake2 = 127;
      liftAutoControl(0.12 * 360 * 100);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        // Move the lift down, allowing movement as long as it's above 0 degrees
            armMotor.move_velocity(200);  // Move lift down
            intake1.move_velocity(0);     
            intake2.move_velocity(0);       
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            armMotor.move_velocity(-200);  // Move lift up
            intake1.move_velocity(0);  
            intake2.move_velocity(0);       
    } else {
          armMotor.move_velocity(0); 
          intake1.move_velocity(0);  
          intake2.move_velocity(0);   
    }
}

void liftControl() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
    intake1.move_velocity(600);
    intake2.move_velocity(600);
    liftAutoControl(FIRST_RING_LIFT_VALUE);
  } else {
    manualLiftControl();
  }
}

double globalLiftTarget = FIRST_RING_LIFT_VALUE;

void liftLoop() {
  for (int i; i <200; i++) {
    liftAutoControl(targetLiftValue);
    pros::delay(10);
  }
  stopLift();
}

void startLiftTask(double targetLiftValue) {
  globalLiftTarget = targetLiftValue;
  pros::Task liftTask(liftLoop);
}

