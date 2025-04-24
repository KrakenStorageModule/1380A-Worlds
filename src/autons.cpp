#include "autons.hpp"

#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 110.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(7.0, 0.0, 20.0);         // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}
int intakeState = 0;  // 0 = off, 1 = intake, 2 = outtake

// call this to set the full intake to intake indefinitely until another intake function is called
void autoIntake() {
  intake.move_voltage(-1200000);
  intakeState = 1;
}

// call this to set the full intake to outtake indefinitely until another intake function is called
void outtake() {
  intake.move_voltage(1200000);
  intakeState = 2;
}

// call this to stop the full intake indefinitely until another intake function is called
void Intakekill() {
  intake.move_voltage(0);
  intakeState = 0;
}

int color = 0;
// antijam and colorsort, meant to be run as a task during autos.
void intakeExtrasAuto() {
  int timer = 0;
  while (true) {
    if (currState != 1) {
      // antijam
      if (intakeState == 1) {
        if (intake.get_actual_velocity() <= 10) {
          timer++;
          if (timer >= 5) {
            outtake();
            pros::delay(50);
            autoIntake();
            timer = 0;
          }
        }
        // RED alliance mode: EJECT BLUE rings
        else if (color == 0 && (vision.get_hue() <= 270 && vision.get_hue() >= 200) &&
                 vision.get_proximity() > 100) {
          timer = 0;        // reset timer when antijam is not triggered
          pros::delay(70);  // Delay to allow ring to reach apex
          Intakekill();
          pros::delay(70);  // allows ring to fly out fully
          autoIntake();
        }

        // BLUE alliance mode: EJECT RED rings
        else if (color == 1 &&
                 (vision.get_hue() >= 300 && vision.get_hue() <= 400) &&
                 vision.get_proximity() > 100) {
          timer = 0;        // reset timer when antijam is not triggered
          pros::delay(70);  // Delay to allow ring to reach apex
          Intakekill();
          pros::delay(70);  // allows ring to fly out fully
          autoIntake();
        }

        pros::delay(5);
      }
    }
    pros::delay(20);  // delay to prevent overloading the CPU
  }
}

// mogo
void autonMogo() {
  mogoToggle = !mogoToggle;
  mogo.set_value(mogoToggle);
}

// doinker
void autoDoinkerLeft() {
  leftToggle = !leftToggle;
  leftDoinker.set_value(leftToggle);
}
void autoDoinkerRight() {
  rightToggle = !rightToggle;
  rightDoinker.set_value(rightToggle);
}

// intake lift
void autonIntakeLift() {
  intakeLiftToggle = !intakeLiftToggle;
  intakeLift.set_value(intakeLiftToggle);
}

// THESE ARE EXAMPLES!!!
///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 30;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
  ez::screen_print("Left: " + util::to_string_with_precision(l_offset) +
                       "\nRight: " + util::to_string_with_precision(r_offset) +
                       "\nBack: " + util::to_string_with_precision(b_offset) +
                       "\nFront: " + util::to_string_with_precision(f_offset),
                   1);  // Don't override the top Page line
}

// . . .
// Make your own autonomous functions here!
// . . .

void NegativeRedSafeQual() {
  //copied from
  // this puts the arm in loading phase
  nextState();
  // arm task
  pros::Task arm(armDriver);

  // antijam and colorsort
  pros::Task intakeExtras(intakeExtrasAuto);

  // setting position
  chassis.odom_xyt_set(-51, 8, 240);
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();

  // scoring motion for AWS
  // untipState();
  // pros::delay(500);

  // move backwards into mogo and clamp
  //chassis.pid_odom_set({{-26, 23}, rev, 80});
  // outtake();
  // //chassis.pid_wait_quick_chain();
  // untipState();
  // pros::delay(200);
  // // wait and clamp
  // autonMogo();
  // pros::delay(200);

  // // turn to face the opposing alliance to make next movements easier
  // chassis.pid_turn_set(0, TURN_SPEED);
  // autoIntake();
  // chassis.pid_wait();

  // //"arc move into the middle rings"
  // chassis.pid_odom_set({{{-3.5, 44, 0}, fwd, DRIVE_SPEED},
  //                       {{-3.5, 52, 0}, fwd, DRIVE_SPEED}},
  //                      false);
  // chassis.pid_wait_quick();

  // // turn to the ring stack
  // chassis.pid_turn_set(260, TURN_SPEED, true);
  // chassis.pid_wait();
  // // move into ring stack
  // chassis.pid_odom_set({{-24, 48}, fwd, 127});
  // chassis.pid_wait_quick();

  // // move near the corner + arm parallel to the ground
  // chassis.pid_odom_set({{-57, 57}, fwd, DRIVE_SPEED});
  // tippingState();
  // chassis.pid_wait_quick_chain();

  // // move into the corner
  // chassis.pid_odom_set({{-65, 65}, fwd, 60});
  // chassis.pid_wait();

  // // back up and lift intake
  // chassis.pid_odom_set({{-55, 55}, rev, DRIVE_SPEED}, true);
  // autonIntakeLift();
  // chassis.pid_wait_quick_chain();

  // // move gently into corner again
  // chassis.pid_odom_set({{-65, 65}, fwd, 60});
  // chassis.pid_wait();

  // // back up and retract arm
  // chassis.pid_odom_set({{-47, 47}, rev, DRIVE_SPEED});
  // tippingState();
  // chassis.pid_wait();

  // // turn to face alliance wallstake's ring stack
  // chassis.pid_turn_set(170, TURN_SPEED);
  // chassis.pid_wait();

  // // move to stack quickly
  // chassis.pid_odom_set({{-48, 0}, fwd, 127});
  // chassis.pid_wait_quick();

  // // touch ladder
  // chassis.pid_odom_set({{-24, 0}, fwd, 60});
  // nextState();
  // nextState();
  // autonIntakeLift();
  // chassis.pid_wait();
}

void NegativeRedSafeElim() {
  // this puts the arm in loading phase
  nextState();
  // arm task
  pros::Task arm(armDriver);

  // antijam and colorsort
  pros::Task intakeExtras(intakeExtrasAuto);

  // setting position
  chassis.odom_xyt_set(-59, 10, 230);

  // scoring motion for AWS
  untipState();
  pros::delay(500);
  untipState();

  // turn to mogo
  chassis.pid_turn_set(250, TURN_SPEED);
  chassis.pid_wait();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-24, 24}, rev, 100});
  outtake();
  chassis.pid_wait();
  // wait and clamp
  autonMogo();
  pros::delay(200);

  // turn to face the opposing alliance to make next movements easier
  chassis.pid_turn_set(0, TURN_SPEED);
  autoIntake();
  chassis.pid_wait();

  //"arc move into the middle rings"
  chassis.pid_odom_set({{{-3.5, 44, 0}, fwd, DRIVE_SPEED},
                        {{-3.5, 52, 0}, fwd, DRIVE_SPEED}},
                       false);
  chassis.pid_wait_quick();

  // turn to the ring stack
  chassis.pid_turn_set(260, TURN_SPEED, true);
  chassis.pid_wait();
  // move into ring stack
  chassis.pid_odom_set({{-24, 48}, fwd, 127});
  chassis.pid_wait_quick();

  // move near the corner + arm parallel to the ground
  chassis.pid_odom_set({{-57, 57}, fwd, DRIVE_SPEED});
  tippingState();
  chassis.pid_wait_quick_chain();

  // move into the corner
  chassis.pid_odom_set({{-65, 65}, fwd, 60});
  chassis.pid_wait();

  // back up and lift intake
  chassis.pid_odom_set({{-55, 55}, rev, DRIVE_SPEED}, true);
  autonIntakeLift();
  chassis.pid_wait_quick_chain();

  // move gently into corner again
  chassis.pid_odom_set({{-65, 65}, fwd, 60});
  chassis.pid_wait();

  // back up and retract arm
  chassis.pid_odom_set({{-47, 47}, rev, DRIVE_SPEED});
  tippingState();
  chassis.pid_wait();

  // turn to face alliance wallstake's ring stack
  chassis.pid_turn_set(170, TURN_SPEED);
  chassis.pid_wait();

  // move to stack quickly
  chassis.pid_odom_set({{-48, 0}, fwd, 127});
  chassis.pid_wait_quick();

  // turn around
  chassis.pid_turn_set(195, TURN_SPEED);
  chassis.pid_wait();

  // move into positive corner
  chassis.pid_odom_set({{-65, -65}, rev, 127});
  autonIntakeLift();
  chassis.pid_wait();
}

void PositiveRedSafeQual() {
  // copied from: https://www.youtube.com/shorts/7Vq0jS8sU_w
  //  arm task
  nextState();
  pros::Task arm(armDriver);

  // antijam and colorsort
  pros::Task intakeExtras(intakeExtrasAuto);

  // setting position
  chassis.odom_xyt_set(-59, -10, 300);

  // scoring motion for AWS
  untipState();
  pros::delay(500);

  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-20, -20}, rev, 127});
  outtake();
  chassis.pid_wait_quick_chain();
  // slowed backup into mogo
  chassis.pid_odom_set({{-26, -26}, rev, 70});
  outtake();
  chassis.pid_wait();
  untipState();
  pros::delay(200);
  // wait and clamp
  autonMogo();
  pros::delay(200);

  // move to the AWS ring stack and get the top ring
  chassis.pid_odom_set({{-50, 2}, fwd, 127});
  autonIntakeLift();
  autoIntake();
  chassis.pid_wait();

  // move to the ladder and doink a ring with right doinker + drop intake
  chassis.pid_odom_set({{-12, -12}, fwd, 60});
  chassis.pid_wait_until({-6, -6});
  autonIntakeLift();
  chassis.pid_wait();
  autoDoinkerRight();
  pros::delay(200);

  // back up and retract doinker
  chassis.pid_odom_set({{-32, -32}, rev, 127});
  chassis.pid_wait();
  autoDoinkerRight();

  // move into the ring
  chassis.pid_odom_set({{-24, -24}, fwd, 127});
  chassis.pid_wait_quick_chain();

  // rotate around and drop mogo
  chassis.pid_turn_set(270, 127);
  Intakekill();
  chassis.pid_wait_until(180);
  autonMogo();
  chassis.pid_wait();

  // clamp that last mogo (the line mogo)
  chassis.pid_odom_set({{-6, -48}, rev, 60});
  chassis.pid_wait();
  pros::delay(200);
  autonMogo();
  pros::delay(200);

  // intake and turn to ladder
  autoIntake();
  chassis.pid_turn_set(180, TURN_SPEED, true);
  chassis.pid_wait();

  // drive into ladder
  chassis.pid_odom_set({{-6, -20}, fwd, 80});
  nextState();
  nextState();
  chassis.pid_wait();
}

void RedSAWP() {
  //copied from https://www.youtube.com/watch?v=h_V-uvW3cuI
 // this puts the arm in loading phase
 nextState();
 // arm task
 pros::Task arm(armDriver);

 // antijam and colorsort
 pros::Task intakeExtras(intakeExtrasAuto);

 // setting position
 chassis.odom_xyt_set(-51, 8, 240);
 chassis.pid_drive_set(2_in, DRIVE_SPEED);
 chassis.pid_wait();

 // scoring motion for AWS
 untipState();
 pros::delay(500);

 // move backwards into mogo and clamp
 chassis.pid_odom_set({{-20, 20}, rev, 127});
 outtake();
 chassis.pid_wait_quick_chain();
 // slowed backup into mogo
 chassis.pid_odom_set({{-26, 26}, rev, 70});
 chassis.pid_wait();
 untipState();
 pros::delay(200);
 // wait and clamp
 autonMogo();
 pros::delay(200);





}