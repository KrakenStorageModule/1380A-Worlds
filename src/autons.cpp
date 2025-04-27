#include "autons.hpp"

#include <string>

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
bool sortingBool = false;
// call this to stop the full intake indefinitely until another intake function is called
void Intakekill() {
  intake.move_voltage(0);
  intakeState = 0;
}

int color = 0;

void antiJam() {
  const int checkInterval = 5;  // ms per loop
  const int stallTime = 500;    // ms of sustained low velocity to count as jam
  const int stallTicks = stallTime / checkInterval;

  int stallCounter = 0;
  int cooldown = 0;

  while (true) {
    if (intakeState == 1 && sortingBool == false) {
      // If velocity is very low, count it as a potential jam
      if (std::abs(intake.get_actual_velocity()) <= 10 && cooldown == 0) {
        stallCounter++;
        if (stallCounter >= stallTicks) {
          // Jam confirmed, do anti-jam action
          outtake();
          pros::delay(100);  // reverse for 100ms
          autoIntake();
          stallCounter = 0;
          cooldown = 500 / checkInterval;  // cooldown for 500ms
        }
      } else {
        stallCounter = 0;
      }
    } else {
      stallCounter = 0;
    }

    if (cooldown > 0) cooldown--;
    pros::delay(checkInterval);
  }
}

// --- Helper to check if wrong color ring ---
bool isWrongRing() {
  int hue = 0;
  for (int i = 0; i < 3; i++) {
    hue += vision.get_hue();
    pros::delay(3);  // tiny delay
  }
  hue /= 3;

  // Red alliance ejects blue rings
  if (color == 0 && (hue >= 180 && hue <= 240)) {
    return true;
  }

  // Blue alliance ejects red rings
  if (color == 1 && (hue >= 340 || hue <= 20)) {
    return true;
  }

  return false;
}

// --- Main colorSort task ---
int ringsEjected = 0;  // Number of rings ejected
void colorSort() {
  const int ejectRotation = 20;  // degrees to spin intake to eject (tune this!)
  bool ejecting = false;
  int ejectTarget = 0;

  while (true) {
    ez::screen_print(std::to_string(intake.get_position()));
    ez::screen_print(std::to_string(ringsEjected), 2);
    ez::screen_print(std::to_string(vision.get_proximity()), 3);
    if (currState != 1 && intakeState == 1) {
      if (!ejecting && isWrongRing() && vision.get_proximity() > 100) {
        ejecting = true;
        sortingBool = true;
        ejectTarget = intake.get_position() - ejectRotation;
      } else if (ejecting) {
        // Eject only based on motor position
        if (intake.get_position() <= ejectTarget) {
          sortingBool = true;
          Intakekill();
          pros::delay(100);  // Allow ring to fly out
          autoIntake();
          ejecting = false;
          ringsEjected++;
        }
      }
    } else {
      sortingBool = false;
      ejecting = false;
    }

    pros::delay(10);
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

void NegativeRedSafeQual() {
  // copied from 2011B
  // CHANGELOG


  //  this puts the arm in loading phase
  nextState();
  // arm task
  pros::Task arm(armDriver);

  // antijam
  pros::Task sort(colorSort);

  // colorsort
  color = 0;  // 0 = red, 1 = blue
  pros::Task jamming(antiJam);

  // setting position
  chassis.odom_xyt_set(-51, 8, 240);
  chassis.pid_drive_set(2_in, 127);
  chassis.pid_wait_quick_chain();

  // scoring motion for AWS
  untipState();
  pros::delay(400);

  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-26, 20}, rev, 60});
  outtake();
  chassis.pid_wait();
  pros::delay(200);  // allow mogo to settle into bot
  // wait and clamp
  autonMogo();
  untipState();
  pros::delay(200);  // delay to allow mogo to clamp

  // // turn to face the opposing alliance to make next movements easier
  chassis.pid_turn_behavior_set(ez::shortest);
  chassis.pid_turn_set(80, 127);
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(-24, 24, 80);
  //"arc move into the middle rings"
  chassis.pid_odom_set({{{-6, 45, 0}, fwd, 127},
                        {{-6, 62, 0}, fwd, 80}},
                       false);
  autoIntake();
  chassis.pid_wait_quick();

  // back up + align to the ring stack
  chassis.pid_odom_set({{{-7, 45}, rev, 127},
                        {{-25, 24, 0}, rev, 127}},
                       false);
  chassis.pid_wait_quick_chain();
  // // move into ring stack
  chassis.pid_odom_set({{-25, 57}, fwd, 127});
  chassis.pid_wait_quick_chain();

  // turn to face corner
  chassis.pid_swing_set(ez::RIGHT_SWING, 270, 127);
  target = 32000;
  chassis.pid_wait_quick_chain();
  // move near the corner + arm parallel to the ground
  chassis.pid_odom_set({{-57., 57}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 330, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  // into corner
  chassis.pid_odom_set({{-67, 67}, fwd, 127});
  autoIntake();
  chassis.pid_wait();
  pros::delay(150);
  // GASLIGHT
  chassis.odom_xyt_set(-65, 65, 330);
  // // back up and lift intake
  chassis.pid_odom_set({{-55, 55}, rev, 127}, false);
  chassis.pid_wait_quick();

  // move into corner again
  chassis.pid_odom_set({{-67, 67}, fwd, 127});
  chassis.pid_wait_until({-55, 55});
  autonIntakeLift();
  chassis.pid_wait();
  pros::delay(150);

  // back up and retract arm
  chassis.pid_odom_set({{-47, 47}, rev, 127});
  target = 14500;
  chassis.pid_wait();

  // turn to face alliance wallstake's ring stack
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait();

  // move to stack quickly
  chassis.pid_odom_set({{-48, 0}, fwd, 127});
  chassis.pid_wait_quick();
  autonIntakeLift();
  pros::delay(150);
  chassis.pid_turn_set(90_deg, 127);
  chassis.pid_wait_quick_chain();

  // touch ladder
  nextState();
  nextState();
  chassis.pid_odom_set({{-32, 0}, fwd, 127});
  chassis.pid_wait();
}

void NegativeRedSafeElim() {
  // this puts the arm in loading phase
  nextState();
  // arm task
  pros::Task arm(armDriver);

  // antijam and colorsort
  // pros::Task intakeExtras(intakeExtrasAuto);

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
  pros::Task jamming(antiJam);
  color = 0;  // 0 = red, 1 = blue
  // colorsort
  pros::Task sort(colorSort);
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
  // copied from https://www.youtube.com/watch?v=h_V-uvW3cuI
  //  this puts the arm in loading phase
  nextState();
  // arm task
  pros::Task arm(armDriver);

  // antijam
  pros::Task jamming(antiJam);

  // colorsort
  color = 0;  // 0 = red, 1 = blue
  pros::Task sort(colorSort);

  // setting position
  chassis.odom_xyt_set(-51, 8, 240);
  chassis.pid_drive_set(6_in, 127);
  chassis.pid_wait_quick_chain();

  // scoring motion for AWS
  untipState();
  pros::delay(400);

  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-26, 20}, rev, 90});
  outtake();
  chassis.pid_wait();
  pros::delay(100);  // allow mogo to settle into bot
  // wait and clamp
  autonMogo();
  untipState();
  pros::delay(20);  // delay to allow mogo to clamp

  // abusing colorsort here
  // ring middle line
  //  // turn to face the opposing alliance to make next movements easier
  chassis.pid_turn_behavior_set(ez::shortest);
  chassis.pid_turn_set(80, 127);
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(-24, 24, 80);
  //"arc move into the middle rings"
  chassis.pid_odom_set({{{-7, 44, 0}, fwd, 127},
                        {{-7, 62, 0}, fwd, 80}},
                       false);
  autoIntake();
  chassis.pid_wait_quick();

  // swing into the ring stack
  chassis.pid_swing_set(ez::RIGHT_SWING, 230, 127);
  chassis.pid_wait_quick_chain();

  // move into the ring stack
  chassis.pid_odom_set({{-25, 40}, fwd, 127});
  chassis.pid_wait_quick();
}

void RedRushNeg() {
  // arm task
  pros::Task arm(armDriver);

  // antijam
  pros::Task jamming(antiJam);

  // colorsort
  color = 0;  // 0 = red, 1 = blue
  pros::Task sort(colorSort);

  // setting position
  chassis.odom_xyt_set(-51, 30, 70);

  //rush
  chassis.pid_odom_set(48_in, 127, false);
  autoDoinkerLeft();
  autoIntake();
  chassis.pid_wait_quick_chain();

  // //backup into mogo and clamp
  // chassis.pid_odom_set({{-26, 18}, rev, 60});
  // chassis.pid_wait_until(2_in);
  // Intakekill();
  // chassis.pid_wait();
  // pros::delay(200);  // allow mogo to settle into bot
  // // wait and clamp
  // autonMogo();
  // pros::delay(200);  // delay to allow mogo to clamp
  // autoIntake();

  // // turn towards the ring stack, raise the doinker, and move towards the ring stack
  // chassis.pid_turn_set(0, 127);
  // chassis.pid_wait();
  // // retract doinker
  // autoDoinkerLeft();
  // // move into the 2-ring line
  // chassis.pid_odom_set({{-26, 55}, fwd, 127});
  // chassis.pid_wait_quick();

  // // move into the corner + arm parallel to the ground
  // chassis.pid_odom_set({{-65, 65, 330}, fwd, 127});
  // target = 33000;
  // chassis.pid_wait();
  // pros::delay(200);  // allow intake to pick up ring

  // //back it up back it up
  // chassis.pid_odom_set({{-55, 55}, rev, 127});
  // chassis.pid_wait_quick();
  // autonIntakeLift();

  // //slam into the corner again
  // chassis.pid_odom_set({{-65, 65}, fwd, 127});
  // autoIntake();
  // chassis.pid_wait();


  

}














