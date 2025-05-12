#include "subsystems.hpp"

#include "autons.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

// motors
pros::Motor intake(11, pros::MotorGears::blue);
pros::Motor lb(-8, pros::MotorGears::red);

// pistons
pros::adi::DigitalOut mogo('C');          // mogo clamp
pros::adi::DigitalOut rightDoinker('A');  // Right Doinker
pros::adi::DigitalOut leftDoinker('D');   // Left Doinker
pros::adi::DigitalOut intakeLift('B');    // Front Stage Intake Lift -> for autos mostly

// toggle booleans
bool mogoToggle = false;        // toggle for mogo clamp
bool rightToggle = false;       // toggle for right doinker
bool leftToggle = false;        // toggle for left doinker
bool intakeLiftToggle = false;  // toggle for intake lift
bool tippingVar = true;         // toggle for tipping
bool untipVar = true;           // toggle for untipping

// Random variables go here
double error = 0;                    // difference between target and current position
double prevError = 0;                // previous error
double armIntegral = 0;              // sum of all errors
double derivative = 0;               // difference between current and previous error
double output = 0;                   // final output to the lady brown
int descore = 0;                     // bool for descore positions
bool intakeLockingOverride = false;  // this is used to override the intake to allow colorsort/antijam to work

// sensors
pros::Optical vision(7);           // color sensor
pros::Rotation lbSensor(3);        // lady brown rot sensor
pros::Distance distanceSensor(4);  // intake stop distance sensor

// controller(s)
pros::Controller controller(pros::E_CONTROLLER_MASTER);  // controller

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// DRIVER CONTROL CODE GOES HERE

// piston driver control code, works through toggles
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
  // intake lift
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    intakeLiftToggle = !intakeLiftToggle;
    intakeLift.set_value(intakeLiftToggle);
  }
}

// USE THIS AS AN EXAMPLE FOR THINGS LIKE ARMS/LIFTS
// This is a "PID" Loop.

// LADY BROWN CODE
// U change the position of each state by changing the number in states[numstates] = {};
const int numStates = 3;
// make sure these are in centidegrees (1 degree = 100 centidegrees) -> take the number from the rotation sensor and multiply by 100
int states[numStates] = {12500, 17450, 30000};  // Bigger number = further from stowed (depends on sensor orientation)
int currState = 0;                              // current state (index of the array)
int target = 12500;                             // this must be the same as whatever the starting state is!
// PID constants => control how the arm moves
double kP = 1.3;  //"Gas" pedal
double kI = 0.0;  // no touch pls
double kD = 0;    // Adds more "slow-down" towards the end of a motion

// moves the state back 1
void backState() {
  currState = (currState - 1 + numStates) % numStates;  // wrap around to the end of the array
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
  if (descore == 0) {
    target = 28300;
    descore = 1;
  } else {
    target = states[0];
    descore = 0;
  }
}

// Actual competition-used logic for lady brown
void armDriver() {
  // this is in a while loop because it needs to run constantly
  //  this should be called in a task to run the arm
  while (true) {
    // X moves the arm to the next state
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      nextState();
      // this is the button to move the arm to the previous state
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      backState();
      // these are essentially templates for adding extra for extra states
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      tippingState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
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

    prevError = error;  // set the previous error to the current error
    pros::delay(20);    // delay to avoid CPU overload
  }
}

// This is a more "optimal" version of the control loop that uses Bang-Bang to get within a range of the target
// and then uses pid to get to target accurately
// I was lowk too lazy and time pressed to use this in comp cuz it requires more tuning.
void arm() {
  // this is in a while loop because it needs to run constantly
  //  this should be called in a task to run the arm
  while (true) {
    // X moves the arm to the next state
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      nextState();
      // this is the button to move the arm to the previous state
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      backState();
      // these are essentially templates for adding extra for extra states
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      tippingState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      untipState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      descoreState();
    }
    // PID calculations
    error = target - lbSensor.get_position();                        // difference between target and current position
    armIntegral += error;                                            // sum of all errors
    derivative = error - prevError;                                  // difference between current and previous error
    output = (kP * error) + (kI * armIntegral) + (kD * derivative);  // final output to the lady brown

    // this is the "Bang-Bang" Portion
    // If the arm is not within a range of the target, it sends full voltage to the arm
    if (error > 500) {            // Checks if outside of Bang-Bang Exit Range
      if (error > 0) {            // Checks if error is (+) or (-)
        lb.move_voltage(120000);  // Full Speed in direction of target. Might need to make (-) depending on motor orientation
      } else {
        lb.move_voltage(-120000);  // Full Speed in direction of target. Might need to make (+) depending on motor orientation
      }
    } else {
      lb.move_voltage(output);  // actually move the arm
    }
    prevError = error;  // set the previous error to the current error
    pros::delay(20);    // delay to avoid CPU overload
  }
}

// Intake Driver Control Function
void intakeDriver() {
  if (!intakeLockingOverride) {  // checks to see if the intake is being overridden by the antijam or colorsort. Can be removed if not running either
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      autoIntake();  // intake (uses the autonomous function to avoid confusion + for ease of use)
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      outtake();  // outtake (uses the autonomous function to avoid confusion + for ease of use)
    } else {
      Intakekill();  // stop the intake (uses the autonomous function to avoid confusion + for ease of use)
    }
  }
}

// antijam code for driver control
void antiJamDriverControl() {
  const int checkInterval = 20;                      // ms per loop
  const int stallTime = 300;                         // ms of sustained low velocity to count as jam
  const int stallTicks = stallTime / checkInterval;  // allows u to play with loop time and stall time without changing how the code runs

  int stallCounter = 0;  // essentially a counter for how long the intake has been jammed
  int cooldown = 0;      // stops antijam from running constantly due to motors needing to accelerate after antijamming

  while (true) {
    if (intakeState == 1 && sortingBool == false && currState != 1) {
      // checks for: intake is intaking, not colorsorting, and lady brown is not in the loading state
      //  If velocity is very low, count it as a potential jam
      if (std::abs(intake.get_actual_velocity()) <= 10 && cooldown == 0) {
        stallCounter++;  // increment the stall counter if the intake is jammed
        if (stallCounter >= stallTicks) {
          // Jam confirmed, do anti-jam action
          intakeLockingOverride = true;    // set the override to true
          outtake();                       // reverses to unjam the intake
          pros::delay(100);                // reverse for 100ms
          intakeLockingOverride = false;   // set the override to true, locking out the driver from accessing the intake
          stallCounter = 0;                // resets the stall counter after successfully unjamming the intake
          cooldown = 300 / checkInterval;  // cooldown for 300ms. allows the intake to get back up to speed
        }
      } else {
        stallCounter = 0;  // reset the stall counter if the intake is not jammed
      }
    } else {
      stallCounter = 0;  // resets the stall counter if the conditions are incorrect for it to start counting
    }

    if (cooldown > 0) cooldown--;  // decrement the cooldown counter
    pros::delay(checkInterval);
  }
}

void colorSortDriverControl() {
  const int ejectRotation = 20;  // degrees to spin intake to eject (tune this!)
  bool ejecting = false;         // bool to check if the intake is ejecting a ring
  int ejectTarget = 0;           // target position for the intake to eject the ring

  if (currState != 1 && intakeState == 1) {  // checks if the intake is intaking and the lady brown is not in the loading state
    if (!ejecting && isWrongRing() && vision.get_proximity() > 100) {
      ejecting = true;                                      // ejecting bool is set to true, allowing the sorting to throw rings
      sortingBool = true;                                   // sorting bool is set to true, overriding the antijam code
      ejectTarget = intake.get_position() - ejectRotation;  // sets the target position for the intake to eject the ring
      // NOTE: it is (-), but sometimes it is (+) depending on the orientation of the intake motor
    } else if (ejecting) {
      // Eject only based on motor position
      if (intake.get_position() <= ejectTarget) {
        sortingBool = true;            // sorting bool is set to true, overriding the antijam code
        intakeLockingOverride = true;  // set the override to true
        Intakekill();                  // stop the intake
        pros::delay(200);              // Adjustable timeout, just how long the sorting should wait before resuming intaking
        autoIntake();                  // intake again after the ring is ejected
        ejecting = false;              // stops the ejecting from happening again until boundary conditions are met
        ringsEjected++;                // increment the number of rings ejected, can be displayed to debug.
      }
    }
  } else {
    intakeLockingOverride = false;  // returns control to driver
    sortingBool = false;            // allows antijam to run again
    ejecting = false;               // no longer ejecting a ring
  }

  pros::delay(20);  // delay to avoid CPU overload
}

// initialize the motors for the drivetrain temperature display. NOT RELATED TO DRIVING AROUND OR AUTOS, THAT IS IN CHASSIS BELOW!!!
pros::MotorGroup left_drive({-19, -17, 18});
pros::MotorGroup right_drive({12, 13, -14});

// controller display for drive temperature
void tempDisplay() {
  //  Variables for temperature readings
  int avgTempLeft = 0;
  int avgTempRight = 0;
  int avgTempIntake = 0;
  int returning = 0;
  int avgTempLb = 0;
  int avgTempTotal = 0;
  int batteryLevel = ((pros::battery::get_capacity()) / 1100) * 100;  // more accurate battery level than the default display
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
    returning = (avgTempLeft + avgTempRight) / 2;  // avg
    returning = (returning * 1.8) + 32;            // Convert to Fahrenheit
    // Averaging the intake and lady brown motor temperatures + convert to F
    avgTempIntake = (1.8 * intake.get_temperature()) + 32;
    avgTempLb = (1.8 * lb.get_temperature()) + 32;

    // Convert to F while averaging both sides

    // Convert temperatures to string and display
    controller.set_text(0, 0, "DT: " + std::to_string(int(returning)));

    pros::delay(50);  // delay to avoid cpu/controller screen overload
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
                                               // if not running odom pods, comment out the next 2 lines
ez::tracking_wheel horiz_tracker(2, 2, -2.5);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(14, 2, 0);     // This tracking wheel is parallel to the drive wheels
//Quick Note: To get distance_to_center, just grab a measuring tape and measure how far from the center of mass/gravity the wheels are. 
//You can be an inch or two off without anything bad happening.