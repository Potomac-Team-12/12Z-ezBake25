#include "main.h"
#include "devices.hpp"
#include "drivercontrol.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
bool wingsOut = false;
bool liftUp = false;
bool goalClampToggle = false;

pros::Controller master (pros::E_CONTROLLER_MASTER);

pros::ADIDigitalOut doinker('G');
pros::ADIDigitalOut goalClamp1('A');
pros::ADIDigitalOut goalClamp2('B');

pros::Rotation armSensor (9);
pros::Motor armMotor (7, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor intake1 (5, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake2 (6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor leftFront(1, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor leftMiddle(2, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor leftBack(3, pros::E_MOTOR_GEARSET_06, false); 
pros::Motor rightFront(11, pros::E_MOTOR_GEARSET_06, false); 
pros::Motor rightMiddle(12, pros::E_MOTOR_GEARSET_06, false); 
pros::Motor rightBack(13, pros::E_MOTOR_GEARSET_06, true); 

pros::MotorGroup left_side_motors({leftFront, leftMiddle, leftBack,});
pros::MotorGroup right_side_motors({rightFront, rightMiddle, rightBack});

pros::Imu imu(4); 

Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{11, 12, -13} 

  // IMU Port
  ,4

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  // eg. 48:36 = 1.3333
  ,1.3333


  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);



void checkMotorsAndPrintTemperature() {
    std::vector<pros::Motor> motors = {
      leftFront, leftBack, leftMiddle, rightFront, rightBack, rightMiddle, intake1, intake2, armMotor,
    };

    int totalTime = 105; // 1 minute 45 seconds
    int startTime = pros::millis();

    while (true) {
        double totalTemp = 0.0;
        int count = 0;

        for (auto& motor : motors) {
            double temp = motor.get_temperature();
            if (temp == PROS_ERR_F) { // PROS_ERR_F is returned when the motor is unplugged
                master.set_text(0, 0, "Motor " + std::to_string(motor.get_port()) + " unplugged.");
                pros::delay(250);
                master.rumble("---");
            }

            if (count < 6) {
                totalTemp += temp;
            }
            ++count;
        }

        if (count == 0) master.set_text(0, 0, "No motors found.");

        double averageTempCelsius = totalTemp / 6;
        double averageTempFahrenheit = averageTempCelsius * 9.0 / 5.0 + 32.0;
        master.set_text(0, 0, "Avg Temp: " + std::to_string(imu.get_heading()));

        pros::delay(250);
    }
}

void driverControlCountdown() {
    if (COMPETITION_CONNECTED != true) { return; }
    
    int totalTime = 105; // 1 minute 45 seconds
    int startTime = pros::millis();

    while (true) {
        int currentTime = pros::millis();
        int elapsedTime = (currentTime - startTime) / 1000; // elapsed time in seconds
        int timeLeft = totalTime - elapsedTime;

        if (timeLeft == 30) {
            pros::Controller master(pros::E_CONTROLLER_MASTER);
            master.rumble(".");
        } else if (timeLeft == 15) {
            pros::Controller master(pros::E_CONTROLLER_MASTER);
            master.rumble("..");
        } else if (timeLeft <= 5 && timeLeft > 0) {
            pros::Controller master(pros::E_CONTROLLER_MASTER);
            master.rumble("-");
        } else if (timeLeft <= 0) {
            break;
        }

        pros::delay(20);
    }
}

void default_constants() {
  chassis.pid_heading_constants_set(4, 0, 35);
  // -- Tune these --
  chassis.pid_drive_constants_set(7, 0.02, 25);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30);

  chassis.pid_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);

  chassis.slew_drive_constants_set(7_in, 80);
}

void modified_exit_conditions() {
  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 150_ms, 7_deg, 150_ms, 100_ms);
  chassis.pid_swing_exit_condition_set(100_ms, 3_deg, 150_ms, 7_deg, 150_ms, 100_ms);
  chassis.pid_drive_exit_condition_set(100_ms, 1_in, 150_ms, 3_in, 150_ms, 100_ms);
}

void calibrateChassis() {
    // calibrate the chassis with imu
    chassis.initialize();
}

void chassisInits() {
    // EZ temp inits
    chassis.opcontrol_curve_buttons_toggle(false); // Enables modifying the controller curve with buttons on the joysticks
    chassis.opcontrol_drive_activebrake_set(0); // Sets the active brake kP. We recommend 0.1.
    chassis.opcontrol_curve_default_set(7, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
    default_constants(); // Set the drive to your own constants from autons.cpp!
    modified_exit_conditions(); // Set the exit conditions to your own from autons.cpp!
}