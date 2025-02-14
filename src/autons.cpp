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

  chassis.pid_drive_set(24, DRIVE_SPEED, true, true);
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
  
  chassis.pid_drive_set(-10, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  
  chassis.pid_turn_set(-45, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-14, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false);
  goalClamp2.set_value(false);

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(24, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_turn_set(90, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(16, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7.5, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(75, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(8, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();
}

void blue_neg_rings(){ //one tile over
  chassis.drive_angle_set(180);
  
  chassis.pid_drive_set(8, DRIVE_SPEED, true, true);
  chassis.pid_wait();

  chassis.pid_turn_set(240, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-29, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false);
  goalClamp2.set_value(false);

  pros::delay(200);

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(24, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_turn_set(82.5, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(16, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7.5, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(62, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(8, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-10, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(30, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();
}

void red_neg_rings(){
  chassis.drive_angle_set(0);
  
  chassis.pid_drive_set(8, DRIVE_SPEED, true, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-50, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-29, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  goalClamp1.set_value(false);
  goalClamp2.set_value(false);

  pros::delay(200);

  chassis.pid_turn_set(0, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  intake1 = 127;
  intake2 = 127;

  chassis.pid_drive_set(18, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_turn_set(85, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(16, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(70, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();
  
  chassis.pid_drive_set(12, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_turn_set(180, TURN_SPEED);
  pros::delay(100);
  chassis.pid_wait();

  chassis.pid_drive_set(30, DRIVE_SPEED, true, true);
  pros::delay(100);
  chassis.pid_wait();

}


// void skills2_jar() {
//   //set up stuff
//   chassis.drive_angle_set(0);
//   goalClamp1.set_value(true);
//   goalClamp2.set_value(true);
//   //armMotor.spin(reverse);

//   intake1 = 127;
//   intake2 = 127;

//   chassis.pid_drive_set(-1, 50, true, true);

//   //ring

//   pros::delay(1000);
//   intake1 = 0;
//   intake2 = 0;

//   //ring on goal 
//   chassis.drive_distance(14.75); //goes forward 
//   //chassis.turn_to_angle(1); //face goal 
//   chassis.turn_to_angle(0); //face goal 
//   chassis.drive_distance(-16.75); //goes back into the goal
//   pros::delay(100); //wait
//   chassis.drive_distance(-2.45);
//   goalClamp1.set(false); //close clamp
//   goalClamp2.set(false);
//   pros::delay(250); 


//   //ring 1
//   chassis.turn_to_angle(182);
//   intake1 = 127;
//   intake2 = 127;
//   chassis.drive_distance(24.5); 
//   pros::delay(650); 
//   chassis.drive_distance(-10); 
//   //ring 2
//   chassis.turn_to_angle(230);
//   chassis.drive_distance(15); 
//   pros::delay(650); 
//   //ring 3
//   chassis.turn_to_angle(140);
//   chassis.drive_distance(18.5); //16.5
//   pros::delay(150); 

//   //ring 4
//   chassis.turn_to_angle(92);
//   chassis.drive_distance(44); 
//   //ring 5
//   chassis.turn_to_angle(60); 
//   chassis.drive_distance(12); 
//   chassis.turn_to_angle(-45); 
//   chassis.drive_distance(45); 
//   pros::delay(450); 
//   //ring 6 
//   chassis.turn_to_angle(180);
//   chassis.drive_distance(28); 
//   pros::delay(450); 
//   //back to goal
//   chassis.turn_to_angle(75);
//   chassis.drive_distance(-26); 
//   // chassis.turn_to_angle(88);
//   // chassis.drive_distance(-3); 
//   //old spot for spotting intakes
//   intake1 = -127;
//   intake2 = -127;
//   pros::delay(200); 
//   goalClamp1.set(true); //open clamp
//   goalClamp2.set(true);
//   //add
//   //chassis.drive_distance(4);
//   intakeS1.stop();
//   intakeS2.stop(); 

//   //next stack
//   chassis.turn_to_angle(0); //15
//   chassis.drive_distance(20); //38
//   //chassis.drive_distance(36); //38
//   chassis.turn_to_angle(8); //15
//   chassis.drive_distance(42);
//   chassis.turn_to_angle(182);
//   chassis.drive_distance(-6);
//   pros::delay(650); 
//   chassis.drive_distance(-3.25);
//   goalClamp1.set(false); //close clamp
//   goalClamp2.set(false);
//   pros::delay(250); 
//   intakeS1.spin(fwd);
//   intakeS2.spin(fwd); 

//   //ring 1
//   chassis.turn_to_angle(90);
//   chassis.drive_distance(26); 
//   //ring 2 
//   chassis.turn_to_angle(0);
//   chassis.drive_distance(26); 
//   //ring 3
//   chassis.turn_to_angle(65);
//   chassis.drive_distance(21.5); 
//   pros::delay(1000); 
//   chassis.drive_distance(-20);
//   //ring 4 & 5
//   chassis.turn_to_angle(270);
//   chassis.drive_distance(38); 
//   //ring 6
//   chassis.drive_distance(-12); 
//   chassis.turn_to_angle(0);
//   chassis.drive_distance(14); 
//   //put goal in corner
//   chassis.drive_distance(-9); 
//   chassis.turn_to_angle(135);
//   chassis.drive_distance(-9); 
//   goalClamp1.set(true); //open clamp
//   goalClamp2.set(true);
//   pros::delay(250); 
//   intakeS1.spin(reverse);
//   intakeS2.spin(reverse); 
//   //go to other side
//   chassis.drive_distance(12); 
//   chassis.turn_to_angle(90);

//   //other side
//   chassis.drive_distance(104); 
//   chassis.turn_to_angle(180);
//   chassis.drive_distance(110); 
//   chassis.drive_distance(-110); 
//   chassis.drive_distance(110); 
//   chassis.drive_distance(-110); 
// }