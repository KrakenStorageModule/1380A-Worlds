#include "subsystems.hpp"

#include "autons.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

pros::Motor intake(11, pros::MotorGears::blue);
pros::Motor lb(-8, pros::MotorGears::red);

// pistons
pros::adi::DigitalOut mogo('C');          // mogo clamp
pros::adi::DigitalOut rightDoinker('A');  // Right Doinker
pros::adi::DigitalOut leftDoinker('D');   // Left Doinker
pros::adi::DigitalOut intakeLift('B');    // Front Stage Intake Lift -> for autos mostly

// toggles
bool mogoToggle = false;        // toggle for mogo clamp
bool rightToggle = false;       // toggle for right doinker
bool leftToggle = false;        // toggle for left doinker
bool intakeLiftToggle = false;  // toggle for intake lift

// sensors
pros::Optical vision(7);     // color sensor
pros::Rotation lbSensor(3);  // lady brown rot sensor

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);  // controller

// PISTON CODE
void pneumaticDriverControl() {
  // mogo
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
    mogoToggle = !mogoToggle;
    mogo.set_value(mogoToggle);
  }
  // right doinker
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    rightToggle = !rightToggle;
    rightDoinker.set_value(rightToggle);
  }

  // left doinker
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    leftToggle = !leftToggle;
    leftDoinker.set_value(leftToggle);
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    intakeLiftToggle = !intakeLiftToggle;
    intakeLift.set_value(intakeLiftToggle);
  }
}

// LADY BROWN CODE
// U change the position of each state by changing the number in states[numstates] = {};
const int numStates = 3;
// make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {14500, 18500, 30000};  // Bigger number = closer to stowed
int currState = 0;                              // current state (index of the array)
int target = 14500;                             // this must be the same as whatever the stow state is!
// PID constants => should be left alone
double kP = 1.3;  //"Gas" pedal
double kI = 0.0;  // no touch
double kD = 0;    // Adds more "slow-down" towards the end of a motion

// PID variables
double error = 0;        // difference between target and current position
double prevError = 0;    // previous error
double armIntegral = 0;  // sum of all errors
double derivative = 0;   // difference between current and previous error
double output = 0;       // final output to the lady brown
bool tippingVar = true;  // toggle for tipping
bool untipVar = true;    // toggle for untipping
int descore = 0;         // bool for descore positions
// moves the state back 1
void backState() {
  currState = (currState - 1 + numStates) % numStates;
  target = states[currState];
  // setting them to true "resets" the toggles

  untipVar = true;
  tippingVar = true;
  descore = 0;
}

// moves the state forward 1
void nextState() {
  currState = (currState + 1) % numStates;
  target = states[currState];
  // setting them to true "resets" the toggles
  untipVar = true;
  tippingVar = true;
  descore = 0;
}

// short lady brown functions
// lowest position -> used for untipping
void untipState() {
  // this works as a toggle for the untip
  untipVar = !untipVar;
  // resets the tipping toggle
  tippingVar = true;
  descore = 0;
  if (untipVar == false) {
    target = 40000;
  } else {
    // makes the arm go to stow after hitting the untip button again
    target = states[0];
  }
}

// slightly lower than scoring -> tipping mogos
void tippingState() {
  // this works as a toggle for the tipping
  tippingVar = !tippingVar;
  //"resets" the untip toggle
  untipVar = true;
  descore = 0;
  if (tippingVar == false) {
    target = 34000;
  } else {
    // makes the arm go to stow after hitting the tipping button again
    target = states[0];
  }
}
void descoreState() {
  untipVar = true;
  tippingVar = true;

  descore++;
  if (descore > 6) {
    target = states[0];
    descore = 0;
  } else if (descore == 1) {
    target = 29500;
  } else if (descore == 2) {
    target = 31000;
  } else if (descore == 3) {
    target = 32000;
  } else if (descore == 4) {
    target = 33100;
  } else if (descore == 5) {
    target = 34400;
  } else if (descore == 6) {
    target = 35200;
  }
}

// Actual logic for lady brown
void armDriver() {
  while (true) {
    // basically the button logic
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      nextState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      backState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      tippingState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      untipState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      descoreState();
    }
    // PID calculations
    error = target - lbSensor.get_position();                        // difference between target and current position
    armIntegral += error;                                            // sum of all errors
    derivative = error - prevError;                                  // difference between current and previous error
    output = (kP * error) + (kI * armIntegral) + (kD * derivative);  // final output to the lady brown

    lb.move_voltage(output);  // actually move the arm

    prevError = error;
    pros::delay(20);
  }
}

bool intakeLockingOverride = false;  // this is used to override the intake to allow colorsort/antijam
void intakeDriver() {
  if (!intakeLockingOverride) {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      if (currState == 1) {
        intake.move_voltage(-8000);  // slow down intake for lady brown to work
      } else {
        intake.move_voltage(-12000);  // intake
      }
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      outtake();  // stop
    } else {
      Intakekill();
      // stop
    }
  }
}
// intake
void antiJamDriverControl() {
  const int checkInterval = 5;  // ms per loop
  const int stallTime = 300;    // ms of sustained low velocity to count as jam
  const int stallTicks = stallTime / checkInterval;

  int stallCounter = 0;
  int cooldown = 0;

  while (true) {
    if (intakeState == 1 && sortingBool == false && intakeLockingOverride == false && currState != 1) {
      // If velocity is very low, count it as a potential jam
      if (std::abs(intake.get_actual_velocity()) <= 10 && cooldown == 0) {
        stallCounter++;
        if (stallCounter >= stallTicks) {
          // Jam confirmed, do anti-jam action
          intakeLockingOverride = true;  // set the override to true
          outtake();
          pros::delay(100);               // reverse for 100ms
          intakeLockingOverride = false;  // set the override to true
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
// this is the display for temp
pros::MotorGroup left_drive({-19, -17, 18});
pros::MotorGroup right_drive({12, 13, -14});
void tempDisplay() {
  // Variables for temperature readings
  int avgTempLeft = 0;
  int avgTempRight = 0;
  int avgTempIntake = 0;
  int returning = 0;
  int avgTempLb = 0;
  int avgTempTotal = 0;
  int batteryLevel = ((pros::battery::get_capacity()) / 1100) * 100;
  while (true) {
    // Averaging each dt half for left and right drive motors
    avgTempLeft = (left_drive.get_temperature(0) +
                   left_drive.get_temperature(1) +
                   left_drive.get_temperature(2)) /
                  3;

    avgTempRight = (right_drive.get_temperature(0) +
                    right_drive.get_temperature(1) +
                    right_drive.get_temperature(2)) /
                   3;
    returning = (avgTempLeft + avgTempRight) / 2; //avg
    returning = (returning * 1.8) + 32;  // Convert to Fahrenheit
    // Averaging the intake and lady brown motor temperatures
    avgTempIntake = intake.get_temperature();
    avgTempLb = lb.get_temperature();

    // Convert to F while averaging both sides

    // Convert temperatures to string and display
    controller.set_text(0, 0, "Drive: " + std::to_string(int(returning)) + "F " + "  " + "Intake: " +
                         std::to_string(int(avgTempIntake)) + "F " + "  " + "LB: " +
                         std::to_string(int(avgTempLb)) + "F ");
    // controller.set_text(1, 0, "Right: " + std::to_string(int(avgTempRight)) + "F ");
    // controller.set_text(2, 0, "Intake: " + std::to_string(int(avgTempIntake)) + "F ");
    // controller.set_text(3, 0, "LB: " + std::to_string(int(avgTempLb)) + "F ");
    // controller.set_text(4, 0, "Rings Ejected: " + std::to_string(int(ringsEjected)));
    // controller.set_text(5, 0, "Battery: " + std::to_string(int(batteryLevel)) + "% ");
    pros::delay(50);  // Small delay to avoid overload
  }
}

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-19, -17, 18},  // Left Chassis Ports (negative port will reverse it!)
    {12, 13, -14},   // Right Chassis Ports (negative port will reverse it!)

    6,                                         // IMU Port
    3.25,                                      // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);                                      // Wheel RPM = cartridge * (motor gear / wheel gear)
ez::tracking_wheel horiz_tracker(2, 2, -2.5);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(14, 2, 0);     // This tracking wheel is parallel to the drive wheels
