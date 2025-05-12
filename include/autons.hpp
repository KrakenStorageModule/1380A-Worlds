//Quick Note -> This is where you initialize auto stuff. You define the stuff here in autons.cpp
#pragma once

#include <string>
#include "autons.hpp"
#include "subsystems.hpp"

//DONT TOUCH THIS
void default_constants();
//Put your helper functions here
void autoIntake();
void autoOuttake();
void intakeKill();
void intakeExtrasAuto();
void autonmogo();
void autonRightDoinker();
void autonLeftDoinker();
void autonIntakeLift();
void antiJam();
void colorSort();
extern bool isWrongRing();
void autoIntake();
void outtake();
void Intakekill();  

//Put your actual auto routes here
void NegativeRedSafeElim();
void NegativeRedSafeQual();
void PositiveBlueSafeQual();
void RedSAWP();
void NegativeBlueQual();
void NegativeBlueElim();
void PositiveRedQual();
void PositiveRedElim();
void PositiveBlueElim();
void NegativeBlueQual();
void exampleMovements();
void RedRushNeg();


//Put your global variables here
extern int color;
extern int ringsEjected;
extern int intakeState;  // 0 = off, 1 = intake, 2 = outtake
extern bool sortingBool;  // true if sorting is active