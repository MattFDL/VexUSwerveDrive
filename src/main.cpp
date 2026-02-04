#include "main.h"
#include "odometry.cpp"
#include <string>
#include "PIDController.cpp"
#include "Autonomous/Path.cpp"
#include "Autonomous/Point2D.cpp"
#include "Autonomous/PathFollower.cpp"
#include "Autonomous/AutoBuilder.cpp"
#include "Autonomous/Command.cpp"
#include <functional>
#include "Autonomous/Autos.cpp"
#include <cmath>
#include <utility>

#include "drivers.h"
#include "Swerve/SwerveModule.cpp"
#include "Swerve/SwerveDrive.cpp"

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
pros::Rotation forwardRot(5); // changed to port 6
// pros::Rotation sidewaysRot(-6);
pros::IMU imu(6); // TODO get the correct port number

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({4, -3, -2, -1}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({10, -7, 9, 8});
pros::MotorGroup intake_mg({-13, -12, -11});

PneumaticCylinder lift('e');
PneumaticCylinder holder('g');
PneumaticCylinder rake('f');
PneumaticCylinder dscore('h');

driverControls driver(master, left_mg, right_mg, intake_mg, 0, lift, rake, holder, dscore);

odometry odom(forwardRot, imu);

Autos autos = Autos();
PathFollower follower(odom, right_mg, left_mg);

pros::Motor fr_bottom_motor(9);
pros::Motor fr_top_motor(10);
pros::Rotation fr_encoder(-8);

pros::Motor fl_bottom_motor(9);
pros::Motor fl_top_motor(10);
pros::Rotation fl_encoder(-8);

pros::Motor br_bottom_motor(9);
pros::Motor br_top_motor(10);
pros::Rotation br_encoder(-8);

pros::Motor bl_bottom_motor(9);
pros::Motor bl_top_motor(10);
pros::Rotation bl_encoder(-8);

void initialize()
{
	pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Innit: %f", 1);
	odom.reset_sensors();
	//odom.set_start_position(71.97111, 25.95749, 90); // 25.36158, 12.67665, 270); //56.27433, 12.68827
	odom.set_start_position(0, 0, 90); // 25.36158, 12.67665, 270); //56.27433, 12.68827

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

// void odometry_thread() {
// 	while (true) {

// 		pros::delay(10);
// 	}

// }; TODO Threading later :----D

void driveCharacterizationTest(PathFollower &follower)
{
	static int loopCount = 1;
	static double totalVelCalculations = 0;
	static double totalVoltsCalculations = 0;
	static double step = 0.005;
	static bool rejectKs = false;
	follower.driveMotor(step * loopCount, step * loopCount);
	if (odom.velocity > 0.1 && !(rejectKs))
	{
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "KS: %f", step * loopCount);
		rejectKs = true;
	}

	totalVelCalculations = totalVelCalculations + odom.velocity;
	totalVoltsCalculations = totalVoltsCalculations + (step * loopCount);

	pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "KV Calculation: %f", totalVoltsCalculations / totalVelCalculations);
	pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "KV Calculation Time Instant: %f", (step * loopCount) / odom.velocity);
	pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "Volts: %f", step * loopCount);
	loopCount += 1;
}

void autonomous()
{

	autos.generateAutoRightSide();
	//autos.generateAutoLeftSide();
//	AutoBuilder builder = autos.getAutoLeftSide(follower,intake_mg,rake,dscore,holder,lift);
	AutoBuilder builder = autos.getAutoRightSide(follower,intake_mg,rake,dscore,holder,lift);

	while (true)
	{
		odom.calculate_postition();
		builder.run_commands();

		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "X: %f", odom.position_x);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Y: %f", odom.position_y);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 3, "Heading: %f", odom.rotation);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 4, "Angular Velocty: %f", odom.angular_velocity);
		//  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Angular: %f", follower.test1_ang);
		//  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "Vel Left: %f", follower.testx_vel);
		//  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Vel Right: %f", follower.testy_vel);
		//  pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "Heading Error: %f", (follower.heading_error_test / M_PI) * 180);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 5, "Velocity: %f", odom.velocity);

		pros::delay(10); // Run for 20 ms then update
						 // // TODO: change everything (that you can) to references
	}
}

void opcontrol()
{
	// PathFollower follower(odom, right_mg, left_mg);
	// Autos autos(follower, intake_mg);
	// AutoBuilder builder = autos.getTestAuto();

	SwerveModule mod_fr(fr_top_motor, fr_bottom_motor, fr_encoder, 5, -5, 0);
	SwerveModule mod_fl(fl_top_motor, fl_bottom_motor, fl_encoder, 5, 5, 1);
	SwerveModule mod_br(br_top_motor, br_bottom_motor, br_encoder, -5, -5, 2);
	SwerveModule mod_bl(bl_top_motor, bl_bottom_motor, bl_encoder, -5, 5, 3);
	SwerveDrive drive_train(mod_fr, mod_fl, mod_br, mod_bl);


	//SwerveDrive drive = SwerveDrive();
	double angle = 0;
	while (true)
	{
		// bool right_a=master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		// if (right_a) {
		// 	builder.run_commands();
		// }
		// driver.handleInputs();
		// odom.calculate_postition();

		double left_Y=master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    	double left_X=master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		double right_x=master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);


		// double temp = (180 * (std::atan2(left_Y, left_X) / M_PI)) - 90;
		// double temp2 = right_y*50;
		// if (temp < -90 || temp > -90) {
		// 	angle = temp;
		// }
		
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Velocity Set: %f", temp2);

		// std::pair<double,double> state(angle, temp2);

		//mod1.set_state(state);
		drive_train.drive_robot_orientated(left_Y, left_X, right_x);
		

		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "X: %f", odom.position_x);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Y: %f", odom.position_y);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 3, "Heading: %f", odom.rotation);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 5, "Velocity: %f", odom.velocity);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Ticks: %f", odom.ticks);



		pros::delay(10);
	}
}
