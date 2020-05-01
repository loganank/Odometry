#include "main.h"
using namespace okapi;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
	Motor frontLeftMotor(1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor frontRightMotor(2, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backRightMotor(3, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backLeftMotor(4, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

	ADIEncoder leftEncoder('C', 'D');
	ADIEncoder rightEncoder('A', 'B');
	ADIEncoder middleEncoder('E', 'F');

	std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
  .withMotors(frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor)
  .withMaxVelocity(150)
  .withSensors(leftEncoder, rightEncoder, middleEncoder)
	.withGains(
			{0.0035, 0, 0}, // Distance controller gains
			{0.003, 0, 0}, // Turn controller gains
			{0.002, 0, 0.00006}  // Angle controller gains (helps drive straight)
	)
	.withDimensions(AbstractMotor::gearset::green, {{3.5_in, 23.5_in}, imev5BlueTPR})
	.withOdometry() // use the same scales as the chassis (above)
	.buildOdometry(); // build an odometry chassis

	std::shared_ptr<okapi::XDriveModel> driveTrain = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());

	chassis->moveDistance(0_in);
  // assigning the chassis to a X-drive model
  driveTrain->setBrakeMode(AbstractMotor::brakeMode::hold);
	Controller master;

	while (true) {
			driveTrain->xArcade(master.getAnalog(ControllerAnalog::leftX),
	   	master.getAnalog(ControllerAnalog::leftY),
	    master.getAnalog(ControllerAnalog::rightX),
			0.3);

    pros::delay(20);
	}
}
