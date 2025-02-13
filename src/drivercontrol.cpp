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

// // -- Bake Arm/Lift PID --
// // Constants for lift positions
// double FIRST_RING_LIFT_VALUE = 0.077 * 360 * 100; 
// const double MAX_LIFT_VALUE = 0.45 * 360 * 100;

// // PID Control Constants
// double kP = 3.0;  // Adjust this if needed
// double kI = 0.0;  // Currently no integral term
// double kD = 9.0;  // Add some derivative control for damping

// // PID variables
// double error = 0, last_error = 0, integral = 0, derivative = 0;
// double targetLiftValue = FIRST_RING_LIFT_VALUE;

// // Function to stop the lift and reset PID variables
// void stopLift() {
//     armMotor.move_velocity(0);
//     integral = 0;
//     last_error = 0;
// }

// void liftAutoControl(double targetLiftValue = FIRST_RING_LIFT_VALUE) {
//     double resetValue;

//     // Control loop for lift to reach target position
//       double currentLiftValue = armSensor.get_position();  // Get current lift position
//       if (targetLiftValue == -1) {
//           double resetValue = 0.00 * 360 * 100;  // Set lift to a reset position
//           error = resetValue - currentLiftValue;
//       } else {
//           error = targetLiftValue - currentLiftValue;
//       }
      
//       // Check if the error is small enough to stop
//       if (abs(error) < 1) {  
//           // Stop the lift if the target position is reached
//           stopLift();
//           return;
//       }

//       // PID calculations for motor control
//       integral += error;
//       derivative = error - last_error;
//       double motorSpeed =  ((error * kP) + (integral * kI) + (derivative * kD));  // PID control
      
//       // Apply motor speed with a max speed limit
//       motorSpeed = std::clamp(motorSpeed, -12000.0, 12000.0);  // Prevent overspeeding
//       armMotor.move_voltage(motorSpeed);

//       last_error = error;
// }

// ez::PID liftPID{0.055, 0, 0.375, 0, "Lift"};

// void set_lift(int input) {
//   armMotor.move(input);
// }

// void manualLiftControl() {
//     double currentLiftPosition = armSensor.get_position();  // Get current lift position

//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
//       intake1 = 127;
//       intake2 = 127;
//       liftAutoControl(0.12 * 360 * 100);
//     } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
//         // Move the lift down, allowing movement as long as it's above 0 degrees
//             armMotor.move_velocity(200);  // Move lift down
//             intake1.move_velocity(0);     
//             intake2.move_velocity(0);       
//     } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
//             armMotor.move_velocity(-200);  // Move lift up
//             intake1.move_velocity(0);  
//             intake2.move_velocity(0);       
//     } else {
//           armMotor.move_velocity(0); 
//           intake1.move_velocity(0);  
//           intake2.move_velocity(0);   
//     }
// }

// void liftControl() {
//   if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
//     intake1.move_velocity(600);
//     intake2.move_velocity(600);
//     liftAutoControl(FIRST_RING_LIFT_VALUE);
//   } else {
//     manualLiftControl();
//   }
// }

// double globalLiftTarget = FIRST_RING_LIFT_VALUE;

// void liftLoop() {
//   for (int i; i <200; i++) {
//     liftAutoControl(targetLiftValue);
//     pros::delay(10);
//   }
//   stopLift();
// }

// void startLiftTask(double targetLiftValue) {
//   globalLiftTarget = targetLiftValue;
//   pros::Task liftTask(liftLoop);
// }


// ---- 12Z custom ----
bool goalClamp_toggle = true;
void goalClamp () {
  goalClamp_toggle = !goalClamp_toggle;
  if (goalClamp_toggle) {
    goalClamp1.set_value(true);
    goalClamp2.set_value(true);
  } else {
    goalClamp1.set_value(false);
    goalClamp2.set_value(false);
  } }



// --- Arm/Lift PID Control ---
// Positions (based on screen angles)
double homePosition = 169.5 * 100;  // Off position (motor disengaged)  (in centidegrees)
double midPosition = 156.0 * 100;   // Mid position 
double highPosition = 50.0 * 100;   // High position

// Flags
bool toggleA = false;             // Tracks Button A toggle state
bool toggleB = false;             // Tracks Button B toggle state
bool disengageRequested = false;  // Tracks if disengage is requested
bool armEngaged = false;

// PID constants
double kP = 0.8;    
double kI = 0.005;   
double kD = 0.4;    
double threshold = 100.0;  // Adjusted for centidegrees

// Function to disengage the motor
void disengageMotor() {
    disengageRequested = true;
    armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    armMotor.brake();
    armEngaged = false;
}

// Function to clamp a value
double clamp(double value, double minVal, double maxVal) {
    return std::max(minVal, std::min(value, maxVal));
}

// PID control function
void moveToPositionPID(double targetPosition) {
  double error = 0, prevError = 0, integral = 0, derivative = 0;
  double output = 0;
  double maxPower = 110.0;  // Reduce max power to avoid overshoot
  double minPower = 20.0;  // Minimum power to prevent stalling

  armEngaged = true;
  disengageRequested = false;

  while (!disengageRequested) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) || disengageRequested) {
          disengageMotor(); // Stop the motor
          return; // Exit the PID loop
      }

      // Get current position in centidegrees (ensure correct units)
      double currentPosition = armSensor.get_position() * 100; // Convert degrees to centidegrees

      // Compute error
      error = targetPosition - currentPosition;

      // Handle wraparound correction
      if (error > 18000.0) error -= 36000.0;
      else if (error < -18000.0) error += 36000.0;

      // Integral Windup Prevention
      if (std::abs(error) < 2000) { 
          integral += error;
      } else {
          integral = 0; 
      }

      derivative = error - prevError;
      // Compute PID output
      output = (kP * error) + (kI * integral) + (kD * derivative);
      // Clamp output to prevent excessive power
      output = clamp(output, -maxPower, maxPower);
      // Prevent stalling by ensuring minimum power is applied
      if (std::abs(output) < minPower && std::abs(error) > threshold) {
          output = (output > 0) ? minPower : -minPower;
      }
      // Move motor with precise control
      armMotor.move_voltage(output * 12000.0 / 127.0);
      // Stop and hold position when within threshold
      if (std::abs(error) <= threshold) {
          armMotor.move_velocity(0);
          armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          return;
      }

      prevError = error;
      pros::delay(20);
  }
}