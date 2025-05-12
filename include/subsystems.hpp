//Quick Note -> This is where you initialize robot hardware stuff. You define the stuff here in subsystems.cpp
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "subsystems.hpp"

//Don't Remove This! It just lets you set up the drivetrain and related sensors in the subsystems file. I thought it was cleaner this way
extern Drive chassis;

//Motors/Motor Groups Go Here

extern pros::Motor intake; // intake motor
extern pros::Motor lb; // lady brown motor

//Pistons Go Here
extern pros::adi::DigitalOut mogo; // mogo piston
extern pros::adi::DigitalOut rightDoinker; // right doinker piston
extern pros::adi::DigitalOut leftDoinker; // left doinker piston
extern pros::adi::DigitalOut intakeLift; // front stage intake lift piston

//Sensors Go Here
extern pros::Optical vision; // color sensor
extern pros::Rotation lbSensor; // lady brown rot sensor
extern pros::Distance distanceSensor; // intake stop distance sensor
extern pros::Controller controller; // controller

//Toggle Variables Go Here
extern bool mogoToggle; // toggle for mogo clamp
extern bool rightToggle; // toggle for right doinker
extern bool leftToggle; // toggle for left doinker
extern bool intakeLiftToggle; // toggle for intake lift
extern bool untipVar; // untip toggle
extern bool tippingVar; // tipping toggle


//Random Variables that multiple functions have to access goes here
extern int target; // target position for lady brown
extern double error; // error for PID
extern int currState; // current state for lady brown
extern const int numStates; // number of states for lady brown
extern double output; // output for PID
extern bool intakeLockingOverride; // this is used to override the intake to allow colorsort/antijam
extern int states[];


//Function initializations go here
void armDriver();
void intakeDriver();
void pneumaticDriverControl();
void backState();
void nextState();
void untipState();
void tippingState();
void intakeExtrasDriver();
void tempDisplay();
void antiJamDriverControl();
void  colorSortDriverControl();
