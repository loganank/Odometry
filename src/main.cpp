#include "main.h"
using namespace okapi;
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
.withDimensions(AbstractMotor::gearset::green, {{4.0_in, 9.0_in}, imev5GreenTPR})
.withOdometry() // use the same scales as the chassis (above)
.buildOdometry(); // build an odometry chassis

std::shared_ptr<okapi::XDriveModel> driveTrain = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());

Controller master;

void initialize() {//runs when program starts
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}

void disabled() {}//runs when disabled

void competition_initialize() {}//after initialize before auton

void autonomous() {}//after start comp

void opcontrol() {//after auton
	chassis->moveDistance(0_in);
	// assigning the chassis to a X-drive model
	driveTrain->setBrakeMode(AbstractMotor::brakeMode::hold);
	while (true) {
			driveTrain->xArcade(master.getAnalog(ControllerAnalog::leftX),
	   	master.getAnalog(ControllerAnalog::leftY),
	    master.getAnalog(ControllerAnalog::rightX),
			0.3);

    pros::delay(20);
	}
}
