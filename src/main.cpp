#include "main.h"
#include "lemlib/api.hpp"
using namespace pros;

/* ----- Device setup ------ */

// Controller
Controller controller_1(E_CONTROLLER_MASTER);

// Motor groups
MotorGroup motor_group_1({-1,-2,-3},MotorGear::blue);
MotorGroup motor_group_2({4,5,6},MotorGear::blue);

// Intake
Motor intake(8,MotorGear::blue);

// Outtake
Motor outtake(9,MotorGear::blue);

// Inertial sensor
IMU inertial_1(7);

// Descore
adi::Pneumatics descore('A',false);

/* ----- Autonomous setup ------ */

// Drivetrain
lemlib::Drivetrain drivetrain(
	&motor_group_1, // Left motor group
	&motor_group_2, // Right motor group
	12, // Track width
	3.0, // Wheel diameter
	600, // RPM
	2 // Drift
);

// Lateral controller (linear motion)
lemlib::ControllerSettings lateral_controller(
	1.5, // kP
	0, // kI
	8, // kD
	3, // Anti-windup (counteracts kI)
	1, // Small error
	100, // Small time
	3, // Large error
	500, // Large time
	5 // Slew (acceleration)
);

// Angular controller (angular motion)
lemlib::ControllerSettings angular_controller(
	2.0, // kP
	0, // kI
	15, // kD
	3, // Anti-windup (counteracts kI)
	1, // Small error
	100, // Small time
	3, // Large error
	500, // Large time
	10 // Slew (acceleration)
);

// Odometry sensors
lemlib::OdomSensors sensors(
	/* The first 4 arguments are nullptr because 
	they are meant for dead wheels, which we don't 
	have */
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&inertial_1 // Inertial sensor
);

// Chassis
lemlib::Chassis chassis(
	drivetrain,
	lateral_controller,
	angular_controller,
	sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Setup screen for displaying text
	lcd::initialize();
	// Calibrate chassis
	chassis.calibrate();
	while (inertial_1.is_calibrating()) {
		lcd::set_text(1,"Calibrating...");
		delay(20);
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
const double offset = 9.5; // Offsets inertial sensor's distance from the front
void autonomous() {
	lcd::clear();
	lcd::set_text(1,"Autonomous");
	chassis.setPose(0,-offset,0);
	intake.move(127);
	chassis.moveToPoint(0,57 - offset,2500);
	chassis.waitUntilDone();
	intake.move(0);
	chassis.moveToPoint(0,26 - offset,2000);
	chassis.waitUntilDone();
	chassis.turnToHeading(90,1200);
	chassis.waitUntilDone();
	chassis.moveToPoint(35 - offset,26 - offset,2000);
	chassis.waitUntilDone();
	chassis.turnToHeading(0,1200);
	chassis.waitUntilDone();
	chassis.moveToPoint(35 - offset,49 - offset,2000);
	chassis.waitUntilDone();
	outtake.move(127);
	delay(1500);
	outtake.move(0);
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
 
void opcontrol() {
	lcd::clear();
	lcd::set_text(1,"Opcontrol");
	while (true) {
		// Instructions for moving robot
		int back_forth = controller_1.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int left_right = controller_1.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(back_forth,left_right);

		// Intake
		if (controller_1.get_digital(DIGITAL_R1)) {
			intake.move(127);
		}

		else if (controller_1.get_digital(DIGITAL_R2)) {
			intake.move(-127);
		}

		else {
			intake.move(0);
		}

		// Outtake
		if (controller_1.get_digital(DIGITAL_L1)) {
			outtake.move(127);
		}

		else if (controller_1.get_digital(DIGITAL_L2)) {
			outtake.move(-127);
		}

		else {
			outtake.move(0);
		}

		// Descore
		if (controller_1.get_digital_new_press(DIGITAL_UP)) {
			descore.toggle();
		}

		delay(20);
	}
}