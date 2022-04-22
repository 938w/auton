#include "main.h"
#include "okapi/api.hpp"
#include "odomDebug/odomDebug.hpp"

using namespace okapi::literals;
using namespace okapi;

// robot config
// motors
int8_t LEFT_MOTOR1_PORT = -16;
int8_t LEFT_MOTOR2_PORT = 13;
int8_t LEFT_MOTOR3_PORT = -4;
int8_t RIGHT_MOTOR1_PORT = 7;
int8_t RIGHT_MOTOR2_PORT = -10;
int8_t RIGHT_MOTOR3_PORT = 6;

// motor groups
okapi::MotorGroup leftMotors({okapi::Motor(LEFT_MOTOR1_PORT), okapi::Motor(LEFT_MOTOR2_PORT), okapi::Motor(LEFT_MOTOR3_PORT)});
okapi::MotorGroup rightMotors({okapi::Motor(RIGHT_MOTOR1_PORT), okapi::Motor(RIGHT_MOTOR2_PORT), okapi::Motor(RIGHT_MOTOR3_PORT)});

// inertial
int8_t INERTIAL_PORT = 20;

// 4bar
int8_t CLAMP_PORT = 'C';
int8_t GOALCOVER_PORT = 'D';
int8_t FourBAR_PORT = 21;

// piston mogo mech, mogo: mobile goal
int8_t MOGO_MECH_PORT = 'B';

// ring intake port
int8_t RINGINTAKE_PORT = 19;

// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// pistons clamp
pros::ADIDigitalOut fourbarClamp(CLAMP_PORT, true);
pros::ADIDigitalOut mogoMech(MOGO_MECH_PORT, true);
pros::ADIDigitalOut goalCover(GOALCOVER_PORT, false);

// fourbar motor
// port, green, reverse or not
pros::Motor fourbar(FourBAR_PORT, pros::E_MOTOR_GEARSET_18, false);

// ring intake motor
pros::Motor ringintake(RINGINTAKE_PORT, pros::E_MOTOR_GEARSET_06, false);

// tracking wheels
std::shared_ptr<ContinuousRotarySensor> leftEncoder = std::make_shared<ADIEncoder>('G', 'H', false);
std::shared_ptr<ContinuousRotarySensor> rightEncoder = std::make_shared<ADIEncoder>('E', 'F', true);
std::shared_ptr<ContinuousRotarySensor> middleEncoder = std::make_shared<RotationSensor>(20, true);

// Odom Debug
OdomDebug modom(lv_scr_act(), LV_COLOR_ORANGE);

// drivetrain can be used in both drive control and auton
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
// https://github.com/nickmertin/5225A-2017-2018
// https://github.com/theol0403/odomDebug <-- helps to display debug info
auto driveTrain = okapi::ChassisControllerBuilder()
					  .withMotors(leftMotors, rightMotors)
					  // Green gearset, 4 inch wheel diameter, 10 inch wheel track
					  .withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR})
					  // filter
					  .withDerivativeFilters(
						  std::make_unique<AverageFilter<3>>() // Distance controller filter
						  )
					  // pid
					  .withGains(
						  // without the I and D, 0.00064 works for p
						  {0.000515, 0, 0},	   // Distance controller gains [devartive make more wobble] [integral make it autocorrect to point faster]
						  {0.0015, 0, 0.0001}, // Turn controller gains
						  {0, 0, 0.0000}	   // Angle controller gains (helps drive straight)
						  )
					  // can add rotation sensor and encoder here
					  .withSensors(
						  leftEncoder,
						  rightEncoder,
						  middleEncoder)
					  //   // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
					  //   // specify the middle encoder distance (2/25 in) and diameter (3.25 in)
					  //   //.withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
					  .withOdometry({{2.75_in, 9.75_in, -2.25_in, 3.25_in}, quadEncoderTPR})
					  .buildOdometry();

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	std::cout << "Setting OKAPI log level \n";
	okapi::Logger::setDefaultLogger(
		std::make_shared<okapi::Logger>(
			okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
			"/ser/sout",										// Output to the PROS terminal
			okapi::Logger::LogLevel::info						// Show info, errors and warnings -- warn, debug, info
			));
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

void resetSensors()
{
	// reset sensors and reset odometry
	std::cout << "Odometry reset \n";
	leftEncoder->reset();
	rightEncoder->reset();
	middleEncoder->reset();
	driveTrain->setState({0_in, 0_in, 0_deg});
}

void odometryUpdate(void *ignore)
{
	while (true)
	{ // display odomDebug in inch and degree
		auto odomState = driveTrain->getState();
		modom.setData({odomState.x.convert(inch), odomState.y.convert(inch), odomState.theta.convert(degree)}, {leftEncoder->get(), rightEncoder->get(), middleEncoder->get()});

		// refresh every second
		pros::delay(1000);
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
 *
 *   `ChassisScales scales({4_in, 11.5_in}, imev5GreenTPR); // imev5GreenTPR for a green gearset`
 *
 *                             Wheel diameter
 *
 *                              +-+      Center of rotation
 *                              | |      |
 *                              v v      +----------+ Length to middle wheel
 *                                       |          | from center of rotation
 *                     +--->    ===      |      === |
 *                     |         +       v       +  |
 *                     |        ++---------------++ |
 *                     |        |                 | v
 *       Wheel track   |        |                 |
 *                     |        |        x        |+|  <-- Middle wheel
 *                     |        |                 |
 *                     |        |                 |
 *                     |        ++---------------++
 *                     |         +               +
 *                     +--->    ===             ===
 * https://www.vexforum.com/uploads/default/optimized/3X/7/a/7ae613aac9a3a9bc0cd2428fd4c0ebe42d20abd0_2_667x500.jpeg
 */
void opcontrol()
{
	modom.setResetCallback(resetSensors);

	pros::Task odometryTask(odometryUpdate);

	leftEncoder->reset();
	rightEncoder->reset();
	middleEncoder->reset();
	driveTrain->setState({12_in, 108_in, 0_deg});

	// start your oauton code here

	driveTrain->driveToPoint({36_in, 108_in}); /*
	 driveTrain->turnToPoint({36_in, 0_in});
	  driveTrain->driveToPoint({36_in, 120_in}, true);
	  mogoMech.set_value(false);
	 driveTrain->driveToPoint({36_in, 108_in});
	 driveTrain->driveToPoint({72_in, 108_in});
	 fourbarClamp.set_value(false);
	 driveTrain->driveToPoint({12_in, 108_in}, true);*/

	// sample of piston state set
	// fourbarClamp.set_value(false);

	// methods of drivetrain
	// https://pros.cs.purdue.edu/v5/okapi/api/chassis/controller/abstract-chassis-controller.html
	// odometry methods
	// https://okapilib.github.io/OkapiLib/classokapi_1_1OdomChassisController.html

	while (true)
	{
		// tank drive

		// 4bar

		// 4bar clamp

		// back tilt

		// convey

		// 10ms delay for changes
		pros::delay(10);
	}
}
