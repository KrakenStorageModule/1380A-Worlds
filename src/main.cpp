#include "main.h"

#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

// IMPORTANT WEBSITES!!!
//  https://ez-robotics.github.io/EZ-Template/ -> IMPORTANT WEBSITE FOR EZ-TEMPLATE
//  https://path.jerryio.com/ -> IMPORTANT WEBSITE FOR PATHING
// https://pros.cs.purdue.edu/v5/pros-4/api.html -> API. This is all the commands you can use for PROS. Read it. It helps.

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  chassis.pid_targets_reset();   // Resets PID targets to 0
  chassis.drive_imu_reset();     // Reset gyro position to 0
  chassis.drive_sensor_reset();  // Reset drive sensors to 0
  pros::delay(500);              // Stop the user from doing anything while legacy ports configure

  // Configure your motor brake modes here.
  intake.set_brake_mode(MOTOR_BRAKE_COAST);  // Intake motor brake mode
  lb.set_brake_mode(MOTOR_BRAKE_HOLD);       // Lady Brown motor brake mode

  // Configure your chassis controls -> NO TOUCH!
  chassis.opcontrol_curve_buttons_toggle(false);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp! -> NO TOUCH!
  default_constants();

  // Autonomous Selector using LLEMU
  // {"Screen Name", FunctionName} <- Format for adding a new auton
  ez::as::auton_selector.autons_add({{"EXAMPLES", exampleMovements},
                                     {"Red Negative Elim (No Rush) [1+6]", NegativeRedSafeElim},
                                     {"Red Negative Qual (No Rush) [1+6]", NegativeRedSafeQual},
                                     {"Blue Positive Qual (No Rush) [1+5]", PositiveBlueSafeQual},
                                     {"Blue Negative Qual (No Rush) [1+5]", NegativeBlueQual},

                                     {"Red Positive Qual (No Rush) [1+5]", PositiveRedQual},
                                     {"Red Positive Elim (No Rush) [1+5]", PositiveRedElim},
                                     {"Blue Positive Elim (No Rush) [1+5]", PositiveBlueElim}

  });  

  // Initialize chassis and auton selector -> NO TOUCH!!!
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  //This is useless unless you want a piston to close/open when the robot is disabled
  //this can be useful for last second hangs like Over Under, where you could drift into the hang bar -> 
  //and the bot would go up AFTER the match ended
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  //Pretty Much useless NGL
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  controller.clear();  // Clear the controller screen. Please avoid touch unless u mess w/controller display :)

  //Feel free to remove this if you don't use an optical sensor
  vision.set_integration_time(5);  // Set the integration time for the vision sensor
  vision.set_led_pwm(100); //Turns the optical sensor's lights on.

  //NO TOUCH!!!
  chassis.pid_targets_reset();             // Resets PID targets to 0
  chassis.drive_imu_reset();               // Reset gyro position to 0
  chassis.drive_sensor_reset();            // Reset drive sensors to 0

  //This is just what lets the controller temp display run during auto. Feel free to remove if desired.
  pros::Task controllerTask(tempDisplay);  // start the controller task

  // Configure your motor brake modes here. Feel free to alter
  intake.set_brake_mode(MOTOR_BRAKE_COAST);   // Intake motor brake mode
  lb.set_brake_mode(MOTOR_BRAKE_HOLD);        // Lady Brown motor brake mode

  //NO TOUCH!!!
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
//DRIVER CONTROL CODE HERE
void opcontrol() {
  // This is preference to what you like to drive on. I suggest no touch
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  // Configure your motor brake modes. Feel free to alter
  intake.set_brake_mode(MOTOR_BRAKE_COAST);  // Intake motor brake mode
  lb.set_brake_mode(MOTOR_BRAKE_HOLD);       // Lady Brown motor brake mode
  vision.set_integration_time(3); //feel free to alter
  vision.set_led_pwm(100);  //feel free to alter
  
  //you can alter this section no issue
  // lady brown callback
  pros::Task armTask(armDriver);                // start the arm task
  //^this is run as a task because it needs to be called in autos and in driver control (and run throughout the ENTIRE match)
  pros::Task intakeTask(antiJamDriverControl);  // start the antijam task
  //^this is run as a task because it operates on it's own time and would stop the rest of the controls from running if called in the while loop

  //feel free to alter this chunk
  // controller task
  pros::Task controllerTask(tempDisplay);  // start the controller task
  //^Run as a task because of the slow update time (controller can only update every 50 ms)
  controller.clear(); //clears controller screen to let display run
  currState = 0; //this sets index to 0
  target = states[currState]; //this actually tells the lady brown to move to stowed
  while (true) {
    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Split Arcade
    //These next 2 lines are controller options 
    // chassis.opcontrol_tank(); //Tank Control
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Single Stick Arcade

    //You can alter the next 2 chunks 
    // intake functions
    intakeDriver(); //This is not run as tasks because they do not need their own timescale

    // piston driver code
    pneumaticDriverControl(); //This is not run as tasks because they do not need their own timescale

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
