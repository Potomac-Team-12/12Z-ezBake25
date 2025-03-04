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
  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 100_ms, 7_deg, 50_ms, 100_ms);
  chassis.pid_swing_exit_condition_set(100_ms, 3_deg, 100_ms, 7_deg, 50_ms, 100_ms);
  chassis.pid_drive_exit_condition_set(100_ms, 1_in, 100_ms, 3_in, 50_ms, 100_ms);
}


void testAuto() {
  chassis.drive_angle_set(0);

  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  pros::delay(50);
  chassis.pid_wait();
  // chassis.pid_turn_set(180, TURN_SPEED);
  // pros::delay(50);
  // chassis.pid_wait();
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


void $blue_neg_rings(){ //$ not working fully
  chassis.drive_angle_set(180);

  chassis.pid_drive_set(-11.45, 70, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-4.35, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  pros::delay(700);

  chassis.pid_drive_set(20, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 0;
  intake2 = 0;

  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-10, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  
  chassis.pid_turn_set(-45, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-14, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false);
  goalClamp2.set_value(false);

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(16, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7.5, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(75, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
}

void blue_neg_rings(){ //one tile over, no wall stake
  chassis.drive_angle_set(180);
  goalClamp1.set_value(true);
  goalClamp2.set_value(true);
  
  chassis.pid_drive_set(6, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(240, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-29, DRIVE_SPEED, true); //was -30
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-3, DRIVE_SPEED, true); 
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false); //clamp down
  goalClamp2.set_value(false);
  // pros::delay(100);

  chassis.pid_drive_set(-5, DRIVE_SPEED, true); 
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(170, TURN_SPEED); //was 180
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  // intake1 = -127;
  // intake2 = -127;

  pros::delay(100);

  // intake1 = 127;
  // intake2 = 127;

  chassis.pid_turn_set(80, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(16.5, DRIVE_SPEED, true); //first ring
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  // intake1 = -127;
  // intake2 = -127;
  // pros::delay(200);
  // intake1 = 127;
  // intake2 = 127;
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(110, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(8.35, DRIVE_SPEED, true); //second ring
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  // intake1 = -127;
  // intake2 = -127;
  //pros::delay(200);
  // intake1 = 127;
  // intake2 = 127;
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(30, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
}

void redcopy(){
  chassis.drive_angle_set(0);
  goalClamp1.set_value(true);
  goalClamp2.set_value(true);
  
  chassis.pid_drive_set(6, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-60, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-29, DRIVE_SPEED, true); //was -30
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-3, DRIVE_SPEED, true); 
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false); //clamp down
  goalClamp2.set_value(false);
  // pros::delay(100);

  chassis.pid_drive_set(-5, DRIVE_SPEED, true); 
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(10, TURN_SPEED); //was 180
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  // intake1 = -127;
  // intake2 = -127;

  pros::delay(100);

  // intake1 = 127;
  // intake2 = 127;

  chassis.pid_turn_set(80, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(16.5, DRIVE_SPEED, true); //first ring
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  // intake1 = -127;
  // intake2 = -127;
  // pros::delay(200);
  // intake1 = 127;
  // intake2 = 127;
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(110, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(8.35, DRIVE_SPEED, true); //second ring
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  // intake1 = -127;
  // intake2 = -127;
  //pros::delay(200);
  // intake1 = 127;
  // intake2 = 127;
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(185, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(34, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
}

void red_neg_rings(){
  chassis.drive_angle_set(0);
  
  chassis.pid_drive_set(8, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-50, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-29.75, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false);
  goalClamp2.set_value(false);

  intake1 = -127;
  intake2 = -127;
  pros::delay(200);
  intake1 = 0;
  intake2 = 0;

  chassis.pid_turn_set(2, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = -127;
  intake2 = -127;
  pros::delay(200);
  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(19, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_turn_set(72, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(13.5, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(95, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(9.5, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(28, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

}

void red_pos(){
  chassis.drive_angle_set(180);
  
  chassis.pid_drive_set(-15, DRIVE_SPEED, true);
  chassis.pid_wait();

  goalClamp1.set_value(true); //clamp down
  goalClamp2.set_value(true);

  chassis.pid_turn_set(270, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-1.5, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  armMotor.move(127);

  pros::delay(500);

  chassis.pid_drive_set(-5, DRIVE_SPEED, true); //was 1.5
  pros::delay(100);
  chassis.pid_wait();

  armMotor.move(-127);
  pros::delay(100);

  chassis.pid_drive_set(3.5, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(42, DRIVE_SPEED, true);
  chassis.pid_wait();

  armMotor.move(0);

  chassis.pid_turn_set(240, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  //armMotor.move(0);
  
  chassis.pid_drive_set(-26, DRIVE_SPEED, true); //was -30
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-6, DRIVE_SPEED, true); 
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false); //clamp down
  goalClamp2.set_value(false);
  
  intake1.move(127);
  intake2.move(127);

  chassis.pid_drive_set(-5, DRIVE_SPEED, true); 
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(178, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(24, DRIVE_SPEED, true); //was -30
  pros::delay(100);
  chassis.pid_wait();
}

// NEW SKILLS!!! -- -- -- -- -- -- -- -- -- --
void newskills(){
  chassis.drive_angle_set(-90);

  armMotor.move(127);
  pros::delay(500);

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait();

  goalClamp1.set_value(true);
  goalClamp2.set_value(true);

  armMotor.move(-127);

  chassis.pid_drive_set(-22, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-2, DRIVE_SPEED, true);
  chassis.pid_wait();

  goalClamp1.set_value(false); //clamp down
  goalClamp2.set_value(false);

  intake1.move(127);
  intake2.move(127);

  chassis.pid_drive_set(-2, DRIVE_SPEED, true);
  //armMotor.move(0);
  chassis.pid_wait();

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(26, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(300); //slight delay to make sure first ring makes it on before turning

  chassis.pid_turn_set(142, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(39, DRIVE_SPEED, true);
  intake2.move(-127);
  pros::delay(200);
  intake2.move(127);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(268, TURN_SPEED);
  intake2.move(-127);
  pros::delay(100);
  intake2.move(127);
  chassis.pid_wait();

  chassis.pid_drive_set(36, DRIVE_SPEED, true); //first long run in the line
  chassis.pid_wait();

  chassis.pid_drive_set(20, 70, false); //second long run in the line
  chassis.pid_wait();

  chassis.pid_drive_set(-3, DRIVE_SPEED, true); //backs up to not touch the wall
  intake2.move(-127);
  pros::delay(100);
  intake2.move(127);
  chassis.pid_wait();

  chassis.pid_turn_set(145, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(17, DRIVE_SPEED, true);
  intake2.move(-127);
  pros::delay(100);
  intake2.move(127);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  intake2.move(-127);
  pros::delay(100);
  intake2.move(127);
  chassis.pid_wait();

  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-5, DRIVE_SPEED, true);
  intake1.move(0);
  intake2.move(0);
  pros::delay(100);
  intake2.move(-127);
  chassis.pid_wait();

  goalClamp1.set_value(true); //release clamp
  goalClamp2.set_value(true);

  //next side

  chassis.pid_drive_set(8, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(179, TURN_SPEED);
  intake1.move(0);
  intake2.move(0);
  chassis.pid_wait();

  chassis.pid_drive_set(-66, DRIVE_SPEED, true); //was 58
  chassis.pid_wait();

  chassis.pid_drive_set(-3, DRIVE_SPEED, true);
  chassis.pid_wait();

  // chassis.pid_turn_set(180, TURN_SPEED);
  // pros::delay(100);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-10, DRIVE_SPEED, true);
  // pros::delay(100);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-2, DRIVE_SPEED, true);
  // pros::delay(100);
  // chassis.pid_wait();

  goalClamp1.set_value(false); //clamp down
  goalClamp2.set_value(false);

  intake1.move(127);
  intake2.move(127);

  chassis.pid_drive_set(-13, DRIVE_SPEED, true);
  chassis.pid_wait();

  //ring 1
  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(26, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(350); //slight delay to make sure first ring makes it on before turning

  intake2.move(-127);
  pros::delay(300);
  intake2.move(127);

  chassis.pid_turn_set(32, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(39, DRIVE_SPEED, true);
  pros::delay(100);
  //intake2.move(-127);
  chassis.pid_wait();

  chassis.pid_drive_set(-11, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  //intake2.move(127);

  pros::delay(100);

  chassis.pid_turn_set(-90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake2.move(-127);
  pros::delay(200);
  intake2.move(127);

  chassis.pid_drive_set(36, DRIVE_SPEED, true); //first long run in the line
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(20, 70, false); //second long run in the line
  pros::delay(100);
  chassis.pid_wait();

  intake2.move(-127);
  pros::delay(400);
  intake2.move(127);

  chassis.pid_drive_set(-3, DRIVE_SPEED, true); //backs up to not touch the wall
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(35, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(17, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(200);

  chassis.pid_drive_set(-7, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake2.move(-127);
  pros::delay(200);
  intake2.move(127);

  chassis.pid_turn_set(135, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake1.move(0);
  intake2.move(0);

  chassis.pid_drive_set(-5, DRIVE_SPEED, true);
  pros::delay(100);
  intake2.move(-127);
  chassis.pid_wait();

  goalClamp1.set_value(true); //release clamp
  goalClamp2.set_value(true);

  intake1.move(0);
  intake2.move(0);






  //the other side
  chassis.pid_drive_set(8, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(105, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(44, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(270, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-44, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(315, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-34, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(190, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-42, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(14, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(24, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(40, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-13, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false); //clamp
  goalClamp2.set_value(false);

  chassis.pid_drive_set(-4, DRIVE_SPEED, true);
  chassis.pid_wait();

}


void skills2_jar() {
  chassis.drive_angle_set(0);
  chassis.pid_drive_exit_condition_set(100_ms, 1_in, 100_ms, 3_in, 50_ms, 100_ms);
  
  goalClamp1.set_value(true);
  goalClamp2.set_value(true);
  
  intake1 = 127;
  intake2 = 127;

  // chassis.pid_drive_set(-1, 50, true, true);
  // pros::delay(1000);
  // intake1 = 0;
  // intake2 = 0;

  // // Ring on goal
  // chassis.pid_drive_set(14.75, DRIVE_SPEED, true, true);
  // chassis.pid_wait();
  
  chassis.pid_turn_set(315, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-6.75, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-2.45, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  goalClamp1.set_value(false);
  goalClamp2.set_value(false);
  pros::delay(250);

  // Ring 1
  chassis.pid_turn_set(182, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  intake1 = 127;
  intake2 = 127;
  
  chassis.pid_drive_set(24.5, DRIVE_SPEED, true);
  pros::delay(650);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-10, DRIVE_SPEED, true);
  chassis.pid_wait();

  // Ring 2
  chassis.pid_turn_set(230, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(15, DRIVE_SPEED, true);
  pros::delay(650);
  chassis.pid_wait();

  // Ring 3
  chassis.pid_turn_set(140, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(18.5, DRIVE_SPEED, true);
  pros::delay(150);
  chassis.pid_wait();

  // Ring 4
  chassis.pid_turn_set(92, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(44, DRIVE_SPEED, true);
  chassis.pid_wait();

  // Ring 5
  chassis.pid_turn_set(60, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(-45, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(45, DRIVE_SPEED, true);
  pros::delay(450);
  chassis.pid_wait();

  // Ring 6
  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(28, DRIVE_SPEED, true);
  pros::delay(450);
  chassis.pid_wait();

  // Back to goal
  chassis.pid_turn_set(75, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-26, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = -127;
  intake2 = -127;
  pros::delay(200);
  
  goalClamp1.set_value(true);
  goalClamp2.set_value(true);
  
  intake1.move_velocity(0);
  intake2.move_velocity(0);

  // Next stack
  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_turn_set(8, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(42, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_turn_set(182, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-6, DRIVE_SPEED, true);
  pros::delay(650);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-3.25, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  
  goalClamp1.set_value(false);
  goalClamp2.set_value(false);
  pros::delay(250);

  intake1 = 127;
  intake2 = 127;

  // Ring 1
  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(26, DRIVE_SPEED, true);
  chassis.pid_wait();

  // Ring 2
  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(26, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // Ring 3
  chassis.pid_turn_set(65, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(21.5, DRIVE_SPEED, true);
  pros::delay(1000);
  chassis.pid_wait();

  chassis.pid_drive_set(-20, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();

  // Rings 4 & 5
  chassis.pid_turn_set(270, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(38, DRIVE_SPEED, true);
  chassis.pid_wait();

  // Ring 6
  chassis.pid_drive_set(-12, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(14, DRIVE_SPEED, true);
  chassis.pid_wait();

  // Put goal in corner
  chassis.pid_drive_set(-9, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(135, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-9, DRIVE_SPEED, true);
  chassis.pid_wait();

  goalClamp1.set_value(true);
  goalClamp2.set_value(true);
  pros::delay(250);

  intake1 = -127;
  intake2 = -127;

  // Go to other side
  chassis.pid_drive_set(12, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  // Other side
  chassis.pid_drive_set(104, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(110, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-110, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(110, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-110, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
}

void off_line(){
  chassis.pid_drive_set(423120, DRIVE_SPEED, true);
  pros::delay(100);
  chassis.pid_wait();
}