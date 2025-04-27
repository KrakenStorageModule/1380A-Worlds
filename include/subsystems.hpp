#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "subsystems.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

extern pros::Motor intake; // intake motor
extern pros::Motor lb; // lady brown motor

extern pros::adi::DigitalOut mogo; // mogo piston
extern pros::adi::DigitalOut rightDoinker; // right doinker piston
extern pros::adi::DigitalOut leftDoinker; // left doinker piston
extern pros::adi::DigitalOut intakeLift; // front stage intake lift piston

extern pros::Optical vision; // color sensor
extern pros::Rotation lbSensor; // lady brown rot sensor

extern pros::Controller controller; // controller

extern bool mogoToggle; // toggle for mogo clamp
extern bool rightToggle; // toggle for right doinker
extern bool leftToggle; // toggle for left doinker
extern bool intakeLiftToggle; // toggle for intake lift

extern bool untipVar; // untip toggle
extern bool tippingVar; // tipping toggle

extern int target; // target position for lady brown
extern double error; // error for PID
extern int currState; // current state for lady brown
extern const int numStates; // number of states for lady brown
extern double output; // output for PID

void armDriver();
void intakeDriver();
void pneumaticDriverControl();
void backState();
void nextState();
void untipState();
void tippingState();
void intakeExtrasDriver();
void tempDisplay();
extern bool intakeLockingOverride; // this is used to override the intake to allow colorsort/antijam
void antiJamDriverControl();

