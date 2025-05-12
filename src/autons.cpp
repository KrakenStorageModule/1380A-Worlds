#include "autons.hpp"

#include <algorithm>
#include <string>

#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

// THESE WEBSITES WILL HELP SO MUCH!
//  https://docs.ez-robotics.com/ez-template/
// https://path.jerryio.com/
// Look at the bottom of this file for an example of every single possible movement

// These are out of 127.
// Personal Note: I never used these. I generally defaulted to 127 for any point A to point B movement
// and around 80 for any movement that involved intaking
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;  // without odom pods, try to avoid going 127 for turns as you will lose accuracy
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // Need to tune to make accurate, consistent, and speedy movements
  chassis.pid_drive_constants_set(20.0, 0.0, 110.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(7.0, 0.0, 20.0);         // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions -> useful to tune for time reasons, but not needed
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants -> can mostly be ignored
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // for this section, probbaly best to leave alone
  // - if you have tracking wheels, you can run this higher.  1.0 is the max

  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at. Further goes faster ->
                                               // ->but loses accuracy.
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// SET UP GLOBAL AUTO VARIABLES HERE
int intakeState = 0;       // 0 = off, 1 = intake, 2 = outtake. used for other functions to check what the intake is doing
bool ringStored = false;   // true if ring is stored in intake
int color = 0;             // alliance color, 0 = red, 1 = blue
bool sortingBool = false;  // true if sorting is active, used for antijam to check if the intake is running

// INTAKE FUNCTIONS GO HERE!

// call this to set the intake to intake indefinitely until another intake function is called
void autoIntake() {
  intake.move_voltage(-1200000);  // only neccessary part
  intakeState = 1;
  bool ringStored = false;  // true if ring is stored in intake
}

// call this to set the intake to outtake indefinitely until another intake function is called
void outtake() {
  intake.move_voltage(1200000);  // only neccessary part
  intakeState = 2;
}
// call this to stop the intake indefinitely until another intake function is called
void Intakekill() {
  intake.move_voltage(0);  // only neccessary part
  intakeState = 0;
}

// autonomous version of the antijam. seperate from driver because i found it "cleaner" that way due to differences in conditions checked
// for annotations, refer to subsystems.cpp -> antiJamDriverControl();
void antiJam() {
  const int checkInterval = 20;  // ms per loop
  const int stallTime = 400;     // ms of sustained low velocity to count as jam
  const int stallTicks = stallTime / checkInterval;

  int stallCounter = 0;
  int cooldown = 0;

  while (true) {
    if (intakeState == 1 && sortingBool == false && currState != 1) {
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
  // Red alliance ejects blue rings
  if (color == 0 && (vision.get_hue() >= 180 && vision.get_hue() <= 240)) {
    return true;
  }

  // Blue alliance ejects red rings
  if (color == 1 && (vision.get_hue() >= 340 || vision.get_hue() <= 20)) {
    return true;
  }

  return false;
}
int ringsEjected = 0;  // Number of rings ejected

void colorSort() {
  const int ejectRotation = 25;  // degrees to spin intake to eject (tune this!)
  bool ejecting = false;
  int ejectTarget = 0;

  // --- LVGL visual color indicator setup ---
  // no touch this here
  static lv_obj_t* colorIndicator = nullptr;
  static lv_obj_t* colorLabel = nullptr;

  if (colorIndicator == nullptr) {
    colorIndicator = lv_obj_create(lv_scr_act());
    lv_obj_set_size(colorIndicator, 400, 120);                  // controls size of rectangle. feel free to alter
    lv_obj_align(colorIndicator, LV_ALIGN_BOTTOM_MID, 0, -10);  // aligns it to bottom center. feel free to alter
    lv_obj_clear_flag(colorIndicator, LV_OBJ_FLAG_SCROLLABLE);  // disables scrolling. no touch

    colorLabel = lv_label_create(colorIndicator);  // creates label. no touch
    lv_obj_center(colorLabel);                     // centers label in rectangle. no touch
  }

  while (true) {
    // --- Logging to screen ---
    ez::screen_print("Rings Sorted " + std::to_string(ringsEjected), 0);
    ez::screen_print("Intake Temp " + std::to_string(int((intake.get_temperature() * 9 / 5) + 32)), 1);
    ez::screen_print("LB Temp " + std::to_string(int((lb.get_temperature() * 9 / 5) + 32)), 2);
    ez::screen_print("X: " + std::to_string(int(chassis.odom_x_get())) + " Y: " + std::to_string(int(chassis.odom_y_get())) + " T: " + std::to_string(int(chassis.odom_theta_get())), 3);  // this should display odom values ez::screen_print("Y: " + std::to_string(int(chassis.odom_y_get())))//this should display odom values
    // --- Color-based sorting logic ---
    // refer to the driver version of this for annotation
    if (currState != 1 && intakeState == 1) {  // check for intaking + lady brown not in loading state
      // I used this in auto by changing the value of a bool. it stopped a ring halfway through the intake to store it
      if (ringStored == true) {
        if (distanceSensor.get_distance() < 50) {
          Intakekill();
        }
      } else if (!ejecting && isWrongRing() && vision.get_proximity() > 100) {  // actual sorting logic
        ejecting = true;
        sortingBool = true;
        ejectTarget = intake.get_position() - ejectRotation;  // because the intake is negative
      } else if (ejecting) {
        // Eject only based on motor position
        if (intake.get_position() <= ejectTarget) {
          sortingBool = true;
          Intakekill();
          pros::delay(250);  // Allow ring to fly out
          autoIntake();
          ejecting = false;
          ringsEjected++;
        }
      }
    } else {
      sortingBool = false;
      ejecting = false;
    }

    // --- Update visual rectangle color ---
    int hueDisplay = std::clamp(int(vision.get_hue()), 0, 360);

    if (hueDisplay >= 340 || hueDisplay <= 20 && vision.get_proximity() > 100) {
      lv_obj_set_style_bg_color(colorIndicator, lv_color_hex(0xFF0000), LV_PART_MAIN);  // red
      lv_label_set_text(colorLabel, "RED RING");
    } else if (hueDisplay >= 180 && hueDisplay <= 240 && vision.get_proximity() > 100) {
      lv_obj_set_style_bg_color(colorIndicator, lv_color_hex(0x0000FF), LV_PART_MAIN);  // blue
      lv_label_set_text(colorLabel, "BLUE RING");
    } else {
      lv_obj_set_style_bg_color(colorIndicator, lv_color_hex(0x5d5d5d), LV_PART_MAIN);  // neutral
      lv_label_set_text(colorLabel, "NO RING");
    }

    pros::delay(20);
  }
}

// PNEUMATIC AUTONOMOUS FUNCTIONS GO HERE!
// Call these to toggle pistons in auto

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

// WRITE UR AUTOS HERE! GOOD LUCK POLARIS/SAMURAI. Johan Out ;)
void NegativeRedSafeQual() {
  // copied from 2011B
  // CHANGELOG

  // arm task
  pros::Task arm(armDriver);  // this is a "task". It lets you run a function in the background so it can repeat without interrupting ->
  // -> the rest of the auto

  // colorsort
  color = 0;  // 0 = red, 1 = blue

  pros::Task sort(colorSort);

  // antijam
  pros::Task jamming(antiJam);

  // setting intitial position. this is needed for odometry movements to work
  chassis.odom_xyt_set(-53, 13, 270);

  chassis.pid_turn_set(220, 127);
  chassis.pid_wait_quick_chain();

  // scoring motion for AWS
  target = 33500;    // makes arm move
  pros::delay(500);  // wait for arm to move

  chassis.pid_odom_set(-7, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-27.8, 21}, rev, 127});
  outtake();

  chassis.pid_wait_quick_chain();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-18, 23}, rev, 60});
  chassis.pid_wait();
  // wait and clamp
  pros::delay(100);  // delay to allow mogo to clamp
  autonMogo();
  pros::delay(100);  // delay to allow mogo to clamp
  target = 14500;
  // // turn to face the opposing alliance to make next movements easier
  chassis.pid_turn_set(80, 127, false);
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(-24, 24, 80);
  //"arc move into the middle rings"
  chassis.pid_odom_set({{{-9, 50, 0}, fwd, 127},
                        {{-9, 54, 0}, fwd, 80}},
                       false);
  autoIntake();
  chassis.pid_wait_quick_chain();

  // swerve
  chassis.pid_swing_set(ez::RIGHT_SWING, 220, 127);
  chassis.pid_wait_quick_chain();
  // chassis.pid_odom_set({{-25, 30}, rev, 127});
  // chassis.pid_wait_quick_chain();

  // // move into the ring stack
  // chassis.pid_odom_set({{-25, 47}, fwd, 127});
  // chassis.pid_wait_quick_chain();
  // // chassis.pid_swing_set(ez::RIGHT_SWING, 240, 127);  // added at night after tuning autos, so could fuck it up
  // // chassis.pid_wait_quick_chain();

  // move to point near corner and align to corner
  chassis.pid_odom_set({{-40, 47}, fwd, 127});  // prolly need to tune this
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 320, 127);  // swing to align to corner
  // target = 33000;
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{-72, 72}, fwd, 127});
  chassis.pid_wait_quick();
  pros::delay(100);
  // back it up back it up
  chassis.pid_odom_set(-20, 80, false);
  chassis.pid_wait_quick_chain();

  // slam into the corner again
  chassis.pid_odom_set(14_in, 80, false);
  autoIntake();
  chassis.pid_wait_quick_chain();

  // GASLIGHT
  chassis.odom_xyt_set(-62, 62, 320);

  // reverse and retract arm
  chassis.pid_odom_set({{-60, 60}, rev, 127});
  nextState();
  nextState();
  chassis.pid_wait_quick_chain();

  // turn to AWS ring stack
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(-45, 45, 180);

  // move to aws ring stack
  chassis.pid_odom_set({{-40, 10}, fwd, 127});

  autoIntake();
  chassis.pid_wait_quick();

  chassis.pid_turn_set(120, 127, false);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(20_in, 127);
  // // ladder movement
  // chassis.pid_swing_set(ez::RIGHT_SWING, 90, 127);
  // chassis.pid_wait_quick_chain();
  // chassis.drive_set(127, 127);
}

void NegativeRedSafeElim() {
  // copied from 2011B
  // CHANGELOG

  // arm task
  pros::Task arm(armDriver);

  // colorsort
  color = 0;  // 0 = red, 1 = blue

  pros::Task sort(colorSort);

  // antijam
  pros::Task jamming(antiJam);

  // setting position
  chassis.odom_xyt_set(-53, 13, 270);

  chassis.pid_turn_set(240, 127);
  chassis.pid_wait_quick_chain();

  // scoring motion for AWS
  target = 31500;
  pros::delay(400);
  target = 14500;

  chassis.pid_odom_set(-5_in, 127, false);  // move off of AWS
  autoIntake();

  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{-27.8, 21}, rev, 127});
  outtake();
  chassis.pid_wait_quick_chain();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-18, 23}, rev, 60});
  chassis.pid_wait();
  // wait and clamp
  pros::delay(100);  // delay to allow mogo to clamp
  autonMogo();
  target = 14500;
  pros::delay(100);  // delay to allow mogo to clamp
  // // turn to face the opposing alliance to make next movements easier
  chassis.pid_turn_set(80, 127, false);
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(-24, 24, 80);
  //"arc move into the middle rings"
  chassis.pid_odom_set({{{-9, 50, 0}, fwd, 127},
                        {{-9, 60, 0}, fwd, 127}},
                       false);
  chassis.pid_wait_until(7_in);
  autoIntake();
  chassis.pid_wait_quick();

  chassis.pid_odom_set({{-25, 30}, rev, 127});
  chassis.pid_wait_quick_chain();

  // move into the ring stack
  chassis.pid_odom_set({{-25, 47}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 240, 127);  // added at night after tuning autos, so could fuck it up
  chassis.pid_wait_quick_chain();

  // move to point near corner and align to corner
  chassis.pid_odom_set({{-40, 45}, fwd, 127});  // prolly need to tune this
  target = 33000;

  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 320, 127);  // swing to align to corner
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{-70, 70}, fwd, 127});
  chassis.pid_wait_until(5_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait();
  pros::delay(50);
  // back it up back it up
  chassis.pid_odom_set(-15, 127, false);
  chassis.pid_wait_quick();

  // slam into the corner again
  chassis.pid_odom_set(12_in, 127, false);
  autoIntake();

  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(-62, 62, 320);

  // reverse and retract arm
  chassis.pid_odom_set({{-60, 60}, rev, 127});
  target = 30000;
  chassis.pid_wait_quick_chain();

  // turn to AWS ring stack
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(-45, 45, 180);

  // move to aws ring stack
  chassis.pid_odom_set({{-40, 0}, fwd, 127});
  autoIntake();
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-70, -70}, fwd, 127});
  chassis.pid_wait_quick_chain();
}

void PositiveBlueSafeQual() {
  // copied from: https://www.youtube.com/shorts/7Vq0jS8sU_w
  //  arm task
  pros::Task arm(armDriver);

  // antijam and colorsort
  pros::Task jamming(antiJam);
  color = 1;  // 0 = red, 1 = blue
  // colorsort
  pros::Task sort(colorSort);
  // setting position
  chassis.odom_xyt_set(53, -10, 90);

  chassis.pid_turn_set(40, 127);
  chassis.pid_wait();
  // scoring motion for AWS
  target = 33000;
  pros::delay(400);

  chassis.pid_odom_set(-5_in, 127, false);  // move off of AWS
  autoIntake();
  chassis.pid_wait_quick_chain();
  target = 14500;
  chassis.pid_odom_set({{27.8, -21}, rev, 127});
  chassis.pid_wait_quick_chain();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{18, -23}, rev, 60});
  chassis.pid_wait();
  // wait and clamp
  pros::delay(100);  // delay to allow mogo to clamp
  autonMogo();
  pros::delay(100);  // delay to allow mogo to clamp

  // ladder movement for middle rings
  chassis.pid_turn_set(325, 127, false);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{8, -8}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(300, 127, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  pros::delay(100);
  // next turn needs to be lower than 300
  // turn into the second middle ring
  chassis.pid_swing_set(ez::RIGHT_SWING, 270, 127, false);
  chassis.pid_wait();
  autoDoinkerRight();
  pros::delay(100);

  // //reverse out of ladder
  chassis.pid_odom_set({{31, -31, 320}, rev, 90});
  chassis.pid_wait_quick_chain();
  autoIntake();

  // turns to throw rings and then turns to move down to set up the swing (can prolly turn more and throw rings further to avoid needing swing at all)
  chassis.pid_turn_set(250, 80, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  autoDoinkerRight();
  pros::delay(400);  // let doinkers go up
  autoIntake();
  chassis.pid_turn_set(300, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 160, 127, 10, false);  // swing to align to line
  chassis.pid_wait_quick_chain();

  // move into the ring stack through the 2 doinked rings
  chassis.pid_odom_set({{30, -52}, fwd, 90});
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 60, 127);  // added at night after tuning
                                                    // turn to align rings
  chassis.pid_wait_quick_chain();

  // move to point near corner and align to corner
  // chassis.pid_odom_set({{30, -45}, fwd, 127});  // prolly need to tune this
  target = 33000;

  chassis.pid_swing_set(ez::LEFT_SWING, 140, 127);  // swing to align to corner
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{80, -80}, fwd, 127});
  // chassis.pid_wait_until(10_in);
  // chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  pros::delay(50);
  // back it up back it up
  chassis.pid_odom_set(-17_in, 127, false);
  chassis.pid_wait_quick();

  // slam into the corner again
  chassis.pid_odom_set(12_in, 127, false);
  autoIntake();
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(62, -62, 140);

  // ladder touch
  // if truly pressed for time just make it shoot backwards and slam into the hang or lb
  nextState();
  chassis.pid_odom_set({{12, -15}, rev, 127});
}

void NegativeBlueQual() {
  // copied from 2011B
  // CHANGELOG

  // arm task
  pros::Task arm(armDriver);

  // colorsort
  color = 1;  // 0 = red, 1 = blue

  pros::Task sort(colorSort);

  // antijam
  pros::Task jamming(antiJam);

  // setting position
  chassis.odom_xyt_set(53, 13, 90);

  chassis.pid_turn_set(135, 127);
  chassis.pid_wait_quick_chain();

  // scoring motion for AWS
  target = 33500;
  pros::delay(500);
  chassis.pid_odom_set(-7, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{27.8, 21}, rev, 127});
  outtake();

  chassis.pid_wait_quick_chain();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{18, 23}, rev, 60});
  chassis.pid_wait();
  // wait and clamp
  pros::delay(100);  // delay to allow mogo to clamp
  autonMogo();
  pros::delay(100);  // delay to allow mogo to clamp
  target = 14500;
  // // turn to face the opposing alliance to make next movements easier
  chassis.pid_turn_set(280, 127, false);
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(24, 24, 280);
  //"arc move into the middle rings"
  chassis.pid_odom_set({{{7, 50, 0}, fwd, 127},
                        {{7, 54, 0}, fwd, 80}},
                       false);
  autoIntake();
  chassis.pid_wait_quick_chain();

  // swerve
  chassis.pid_swing_set(ez::LEFT_SWING, 130, 127);
  chassis.pid_wait_quick_chain();

  // move to point near corner and align to corner
  chassis.pid_odom_set({{40, 47}, fwd, 127});  // prolly need to tune this
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 45, 127);  // swing to align to corner
  // target = 33000;
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{75, 75}, fwd, 127});
  chassis.pid_wait_quick();
  pros::delay(100);
  // back it up back it up
  chassis.pid_odom_set(-23, 70, false);
  chassis.pid_wait_quick_chain();

  // slam into the corner again
  chassis.pid_odom_set(17_in, 70, false);
  autoIntake();
  chassis.pid_wait_quick_chain();

  // GASLIGHT
  chassis.odom_xyt_set(62, 62, 45);

  // reverse and retract arm
  chassis.pid_odom_set({{60, 60}, rev, 127});
  nextState();
  nextState();
  chassis.pid_wait_quick_chain();

  // turn to AWS ring stack
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(45, 45, 180);

  // move to aws ring stack
  chassis.pid_odom_set({{40, 15}, fwd, 127});

  autoIntake();
  chassis.pid_wait_quick();

  chassis.pid_turn_set(270, 127, false);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(22_in, 127);
}

void PositiveRedQual() {
  // copied from: https://www.youtube.com/shorts/7Vq0jS8sU_w
  //  arm task
  pros::Task arm(armDriver);

  // antijam and colorsort
  pros::Task jamming(antiJam);
  color = 1;  // 0 = red, 1 = blue
  // colorsort
  pros::Task sort(colorSort);
  // setting position
  chassis.odom_xyt_set(-53, -10, 270);

  chassis.pid_turn_set(325, 127);
  chassis.pid_wait();

  // scoring motion for AWS
  target = 32500;
  pros::delay(400);
  target = 14500;

  chassis.pid_odom_set(-5_in, 127, false);  // move off of AWS
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-27.8, -21}, rev, 127});
  chassis.pid_wait_quick_chain();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-18, -23}, rev, 60});
  chassis.pid_wait();
  // wait and clamp
  pros::delay(100);  // delay to allow mogo to clamp
  autonMogo();
  pros::delay(100);  // delay to allow mogo to clamp

  // ladder movement for middle rings
  chassis.pid_turn_set(55, 127, false);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-8, -8}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60, 127, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  pros::delay(100);
  // next turn needs to be lower than 300
  // turn into the second middle ring
  chassis.pid_swing_set(ez::LEFT_SWING, 270, 127, false);
  chassis.pid_wait();
  autoDoinkerRight();
  pros::delay(100);

  // //reverse out of ladder
  chassis.pid_odom_set({{-31, -31, 320}, rev, 90});
  chassis.pid_wait_quick_chain();
  autoIntake();

  // turns to throw rings and then turns to move down to set up the swing (can prolly turn more and throw rings further to avoid needing swing at all)
  chassis.pid_turn_set(70, 127, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  autoDoinkerRight();
  pros::delay(200);  // let doinkers go up
  chassis.pid_turn_set(90, 127, false);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 160, 127, 20, false);  // swing to align to line
  chassis.pid_wait_quick_chain();

  // move into the ring stack through the 2 doinked rings
  chassis.pid_odom_set({{-30, -52}, fwd, 90});
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 110, 127);  // added at night after tuning
                                                    // turn to align rings
  chassis.pid_wait_quick_chain();

  // move to point near corner and align to corner
  chassis.pid_odom_set({{-38, -45}, fwd, 127});  // prolly need to tune this
  target = 33000;

  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 310, 127);  // swing to align to corner
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{-70, -70}, fwd, 127});
  // chassis.pid_wait_until(10_in);
  // chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  pros::delay(50);
  // back it up back it up
  chassis.pid_odom_set(-17_in, 127, false);
  chassis.pid_wait_quick();

  // slam into the corner again
  chassis.pid_odom_set(12_in, 127, false);
  autoIntake();
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(-62, -62, 140);

  // ladder touch
  // if truly pressed for time just make it shoot backwards and slam into the hang or lb
  nextState();
  chassis.pid_odom_set({{-12, -12}, rev, 127});
}

void PositiveRedElim() {
  // copied from: https://www.youtube.com/shorts/7Vq0jS8sU_w
  //  arm task
  pros::Task arm(armDriver);

  // antijam and colorsort
  pros::Task jamming(antiJam);
  color = 1;  // 0 = red, 1 = blue
  // colorsort
  pros::Task sort(colorSort);
  // setting position
  chassis.odom_xyt_set(-53, -10, 90);

  chassis.pid_turn_set(300, 127);
  chassis.pid_wait();

  // scoring motion for AWS
  target = 31500;
  pros::delay(400);
  target = 14500;

  chassis.pid_odom_set(-5_in, 127, false);  // move off of AWS
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-27.8, -21}, rev, 127});
  chassis.pid_wait_quick_chain();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{-18, -23}, rev, 60});
  chassis.pid_wait();
  // wait and clamp
  pros::delay(100);  // delay to allow mogo to clamp
  autonMogo();
  pros::delay(100);  // delay to allow mogo to clamp

  // ladder movement for middle rings
  chassis.pid_turn_set(55, 127, false);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-8, -8}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60, 127, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  pros::delay(100);
  // next turn needs to be lower than 300
  // turn into the second middle ring
  chassis.pid_swing_set(ez::LEFT_SWING, 270, 127, false);
  chassis.pid_wait();
  autoDoinkerRight();
  pros::delay(100);

  // //reverse out of ladder
  chassis.pid_odom_set({{-31, -31, 320}, rev, 90});
  chassis.pid_wait_quick_chain();
  autoIntake();

  // turns to throw rings and then turns to move down to set up the swing (can prolly turn more and throw rings further to avoid needing swing at all)
  chassis.pid_turn_set(70, 127, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  autoDoinkerRight();
  pros::delay(200);  // let doinkers go up
  chassis.pid_turn_set(90, 127, false);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 160, 127, 20, false);  // swing to align to line
  chassis.pid_wait_quick_chain();

  // move into the ring stack through the 2 doinked rings
  chassis.pid_odom_set({{-30, -52}, fwd, 90});
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 110, 127);  // added at night after tuning
                                                    // turn to align rings
  chassis.pid_wait_quick_chain();

  // move to point near corner and align to corner
  chassis.pid_odom_set({{-38, -45}, fwd, 127});  // prolly need to tune this
  target = 33000;

  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 310, 127);  // swing to align to corner
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{-70, -70}, fwd, 127});
  // chassis.pid_wait_until(10_in);
  // chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  pros::delay(50);
  // back it up back it up
  chassis.pid_odom_set(-17_in, 127, false);
  chassis.pid_wait_quick();

  // slam into the corner again
  chassis.pid_odom_set(12_in, 127, false);
  autoIntake();
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(-62, -62, 140);

  // mogo grab
  chassis.pid_odom_set({{-18, -48}, rev, 127});
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{-8, -48}, rev, 60});
  chassis.pid_wait();
  autonMogo();
}

void PositiveBlueElim() {
  // copied from: https://www.youtube.com/shorts/7Vq0jS8sU_w
  //  arm task
  pros::Task arm(armDriver);

  // antijam and colorsort
  pros::Task jamming(antiJam);
  color = 1;  // 0 = red, 1 = blue
  // colorsort
  pros::Task sort(colorSort);
  // setting position
  chassis.odom_xyt_set(53, -10, 90);

  chassis.pid_turn_set(60, 127);
  chassis.pid_wait();

  // scoring motion for AWS
  target = 31500;
  pros::delay(400);
  target = 14500;

  chassis.pid_odom_set(-5_in, 127, false);  // move off of AWS
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{27.8, -21}, rev, 127});
  chassis.pid_wait_quick_chain();
  // move backwards into mogo and clamp
  chassis.pid_odom_set({{18, -23}, rev, 60});
  chassis.pid_wait();
  // wait and clamp
  pros::delay(100);  // delay to allow mogo to clamp
  autonMogo();
  pros::delay(100);  // delay to allow mogo to clamp

  // ladder movement for middle rings
  chassis.pid_turn_set(325, 127, false);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{8, -8}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(300, 127, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  pros::delay(100);
  // next turn needs to be lower than 300
  // turn into the second middle ring
  chassis.pid_swing_set(ez::RIGHT_SWING, 270, 127, false);
  chassis.pid_wait();
  autoDoinkerRight();
  pros::delay(100);

  // //reverse out of ladder
  chassis.pid_odom_set({{31, -31, 320}, rev, 90});
  chassis.pid_wait_quick_chain();
  autoIntake();

  // turns to throw rings and then turns to move down to set up the swing (can prolly turn more and throw rings further to avoid needing swing at all)
  chassis.pid_turn_set(250, 127, false);
  chassis.pid_wait();
  autoDoinkerLeft();
  autoDoinkerRight();
  pros::delay(200);  // let doinkers go up
  chassis.pid_turn_set(270, 127, false);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 160, 127, 20, false);  // swing to align to line
  chassis.pid_wait_quick_chain();

  // move into the ring stack through the 2 doinked rings
  chassis.pid_odom_set({{30, -52}, fwd, 90});
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 60, 127);  // added at night after tuning
                                                    // turn to align rings
  chassis.pid_wait_quick_chain();

  // move to point near corner and align to corner
  chassis.pid_odom_set({{38, -45}, fwd, 127});  // prolly need to tune this
  target = 33000;

  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 140, 127);  // swing to align to corner
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{70, -70}, fwd, 127});
  // chassis.pid_wait_until(10_in);
  // chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  pros::delay(50);
  // back it up back it up
  chassis.pid_odom_set(-17_in, 127, false);
  chassis.pid_wait_quick();

  // slam into the corner again
  chassis.pid_odom_set(12_in, 127, false);
  autoIntake();
  chassis.pid_wait_quick();

  // GASLIGHT
  chassis.odom_xyt_set(62, -62, 140);

  // mogo grab
  chassis.pid_odom_set({{8, -48}, rev, 127});
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{8, -48}, rev, 60});
  chassis.pid_wait();
  autonMogo();
}

void exampleMovements() {


    /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */


  // this is how to set the initial position:
  chassis.odom_xyt_set(0, 0, 0);  // x, y, theta in degrees

  // these are the different ways to wait:
  chassis.pid_wait();                        // basic exit. Longest, most accurate version
  chassis.pid_wait_quick();                  // quicker exit. similar accuracy to above, but quicker
  chassis.pid_wait_quick_chain();            // inaccurate, but fastest exit. It carries momentum into the next movement
  chassis.pid_wait_until(5_in);              // waits until the robot moved a certain amount before allowing the next lines to occur.
  chassis.pid_wait_until_point({0, 0});      // waits until the robot is at a certain point before allowing the next lines to occur.
  chassis.pid_wait_until_index(0);           // waits until the robot is at a certain index before allowing the next lines to occur.
  chassis.pid_wait_until_index_started(90);  // waits until the robot starts at a certain index before allowing the next lines to occur.
  // note: pid_wait_until and all derivatives are used between a movement and a pid_wait or variation of the wait command.

  // there are two ways to move: relative and absolute.

  // Relative Movements: I discourage these slightly due to being more inaccurate than odom
  // If you change one movement, the rest of the movements will be affected.
  // These move relative to the current position of the robot.
  chassis.pid_drive_set(5_in, 127, false);                  // move forward 5 inches at 127 speed
  chassis.pid_turn_relative_set(90, 127);                   // turn 90 degrees from current angle at 127 speed
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 90, 127);  // swing left 90 degrees from current positionat 127 speed

  // Example of relative movement used in an auto ;) (Comment out the rest of the auto and run this to see it in action)
  chassis.pid_drive_set(15_in, 80, false);  // move forward 15 inches at 80 speed
  chassis.pid_wait();                       // wait
  chassis.pid_turn_relative_set(180, 60);   // turn 180 degrees from current angle at 60 speed
  chassis.pid_wait();                       // wait
  chassis.pid_drive_set(15_in, 80, false);  // move backwards 15 inches at 80 speed
  autoIntake();                             // turns intake on during the movement
  chassis.pid_wait_until(10_in);
  Intakekill();                           // turns intake off when the robot has moved 10 inches
  chassis.pid_wait();                     // wait
  chassis.pid_turn_relative_set(90, 60);  // turn 90 degrees from current angle at 127 speed
  chassis.pid_wait();                     // wait

  // Absolute Movements:
  // These are oriented to the field itself, and are generally more accurate/consistent than relative movements.
  // https://path.jerryio.com/
  chassis.pid_odom_set(10_in, 127, false);         // move forward 10 inches at 127 speed
  chassis.pid_odom_set({{10, 10}, rev, 127});      // move to the point (10, 10) at 127 speed
  chassis.pid_odom_set({{10, 10, 90}, rev, 127});  // move to the point (10, 10), ending at absolute 90 angle
  chassis.pid_odom_set({{{-9, 50}, fwd, 127},
                        {{-9, 60, 0}, fwd, 127}},
                       false); //This will go through each point in the list sequentially
  chassis.pid_swing_set(ez::LEFT_SWING, 90, 127); // swerve left 90 degrees at 127 speed
  chassis.pid_swing_set(ez::LEFT_SWING, 90, 80, 20, false); // makes wider arc 
  // Example of absolute movement used in an auto ;) (Comment out the rest of the auto and run this to see it in action)
    chassis.odom_xyt_set(0, 0, 0);  // set the initial position of the robot to (0, 0) at 0 degrees
    chassis.pid_odom_set({{0, 15}, fwd, 127}, false); // move to the point (0, 15) at 127 speed
    chassis.pid_wait_quick_chain(); // wait for the movement to finish
    chassis.pid_turn_set(180, 127);
    chassis.pid_wait_quick_chain(); // wait for the turn to finish
    chassis.pid_odom_set({{0, 0}, rev, 127}); // move to the point (0, 0) at 127 speed
    autoIntake();                             // turns intake on during the movement
    chassis.pid_wait_until(10_in); //waits to reach 10 inches along the movement
    Intakekill();  //turns off intake
    chassis.pid_wait_quick_chain(); // wait for the movement to finish
    chassis.pid_turn_set(180, 127);
    chassis.pid_wait_quick_chain(); // wait for the movement to finish
  
    // As you can see, these autos achieve the same thing in slightly different ways.

  //Here are some quick notes:
  // 1. Slow down the drivetrain while intaking or doing something important. Full speed when just getting point A -> point B. Good general rule 
  // 2. Use pid_wait_quick() or pid_wait_quick_chain() when possible to save time. The accuracy pid_wait(); provides is unneccesary for vex applications
  // 3. make sure you use a plastic field to make autos for states/worlds. our garage field is poo poo :(
  // 4. If you use odom, use jerrypath.io for general points ->
  //  -> then read the brain display for where the robot thinks it is and fine tune using that. you can find this display in auto colorsort funct
  // 5. Please for the love of GOD use odometry pods. It makes life much easier. Just ask Aadit if you don't believe me.
  // 6. If you have any code issues, feel free to reach out to me. Good Luck in Push Back and y'alls senior year. Be better than we were. Bye!
}