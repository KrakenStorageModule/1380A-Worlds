#pragma once

#include <string>
#include "autons.hpp"
#include "subsystems.hpp"
void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

void autoIntake();
void autoOuttake();
void intakeKill();
void intakeExtrasAuto();
void autonmogo();
void autonRightDoinker();
void autonLeftDoinker();
void autonIntakeLift();

void NegativeRedSafeElim();
void NegativeRedSafeQual();

void PositiveRedSafeQual();
void RedSAWP();

void antiJam();
void colorSort();

void autoIntake();
void outtake();
void Intakekill();  

void RedRushNeg();
extern int color;
extern int ringsEjected;
extern int intakeState;  // 0 = off, 1 = intake, 2 = outtake
extern bool sortingBool;  // true if sorting is active