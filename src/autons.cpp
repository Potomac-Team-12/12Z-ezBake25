#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "devices.hpp"
#include "autons.hpp"
#include "drivercontrol.hpp"
#include "pros/rtos.hpp"

// function definitions

//       chassis.setPose(x, y, theta); 
//       chassis.turnTo(x, y, timeout);    
//       chassis.follow(path file, timeout, lookahead distance, isAsynchronous, forwards (false makes the robot run the path backwards));
//       chassis.moveToPose(x, y, theta, timeout, {maxSpeed, minSpeed, earlyExitRange});
//       chassis.moveToPoint(x, y, timeout, {maxSpeed, minSpeed, earlyExitRange, forwards});
//
//       chassis.waitUntil(inches); use this to have your bot wait until the path is finished
//       a really big number means just wait until the path has finished

const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;
void safe_exit_conditions() {
  chassis.pid_turn_exit_condition_set(150_ms, 3_deg, 300_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(150_ms, 3_deg, 300_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(150_ms, 1_in, 300_ms, 3_in, 500_ms, 500_ms);
}

void testAuto() {
  chassis.drive_angle_set(0);

  chassis.pid_drive_set(24, 100, false, true);

}

void soloAwpSafe(bool isRed) { 
  double sign = isRed ? 1 : -1;

  chassis.drive_angle_set(-50 * sign);

  // intakeRaise.set_value(true);
  pros::delay(200);
  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(8, 40, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(200);
  // intakeRaise.set_value(false);
  pros::delay(200);

  intake1 = 0;
  intake2 = 0;

  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  //turn to go to alliance stake
  chassis.pid_turn_set(-90 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(14, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(0 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-3, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  pros::delay(750);
  intake1 = 0;
  intake2 = 0;

  // move forward 
  chassis.pid_drive_set(8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // turn to goal
  chassis.pid_turn_set(-140 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(true);
  goalClamp2.set_value(true);
  chassis.pid_drive_set(-36, 70, true);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false);
  goalClamp2.set_value(false);  
  pros::delay(300);

  chassis.pid_turn_set(90 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(28, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(180 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(38, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-38, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-45 * sign, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
}

void soloAwpSafeBlue() { soloAwpSafe(false); }

void soloAwpSafeRed() { soloAwpSafe(true); }




void doNothingAuto() {
  // do nothing
}