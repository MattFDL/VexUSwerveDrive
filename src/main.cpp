#include "main.h"
#include "odometry.cpp"
#include <string>
#include "PIDController.cpp"
#include "Autonomous/Path.cpp"
#include "Autonomous/Point2D.cpp"
#include "Autonomous/PathFollower.cpp"

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
pros::Rotation forwardRot(4);
pros::Rotation sidewaysRot(-14);
pros::IMU imu(17); // TODO get the correct port number

odometry odom(forwardRot, sidewaysRot, imu);

std::atomic<std::array<double, 3>> myAtomicStdArray;

// void odometry_thread() {
// 	while (true) {

// 		pros::delay(10);
// 	}

// }; TODO Threading later :----D
//Path path(Point2D(0, 0), Point2D(10, -10), Point2D(20, 10), Point2D(30, 0));
Path path(Point2D(0, 0), Point2D(40, 0), Point2D(40, 30), Point2D(0, 30));
Path path2(Point2D(0, 30), Point2D(40, 30), Point2D(40, 0), Point2D(0, 0));


void test_function()
{
	pros::screen::draw_circle(10, 10, 5);
	pros::screen::draw_circle(30, 100, 5);
	pros::screen::draw_circle(100, 200, 5);
	pros::screen::draw_circle(200, 10, 5);
	for (Point2D num : path.curvePoints)
	{
		int x = (int)num.x;
		int y = (int)num.y;
		pros::screen::draw_pixel(x, y);
	}
	//pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Length: %f", path.curve_length);
}

void opcontrol()
{

	// pros::Task odom_task(odometry_thread); Threading for later ;) :O

	odom.reset_sensors();
	//odom.set_start_position(0, 0, 180);

	PIDController pid(80.0, 35.0, 0.0); // 20.0
	pid.enableContinuousInput(-180, 180);

	pid.setErrorTolerance(2.0);

	pid.setIzone(30);
	pid.setMaxMinI(4000, -4000);

	path.setStepCount(100);
	path.generateCubicPathEquation();
	path.resizeCurve();

	path2.setStepCount(100);
	path2.generateCubicPathEquation();
	path2.resizeCurve();

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({-10, -9, 2, -1});	  // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({20, 19, -12, 11}); // Creates a motor group with forwards port 5 and reversed ports 4 & 6

	PathFollower follower(odom, right_mg, left_mg);
	follower.setPath(path);

	PathFollower follower2(odom, right_mg, left_mg);
	follower2.setPath(path2);
	

	while (true)
	{

		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// // Arcade control scheme
		odom.calculate_postition();

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			if (!follower.pathFinished) {
				follower.followPath();
				pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 10, "PathFinished: %f", follower.pathFinished);
			} else {
				follower2.followPathBackwards();
			}			
		}
		else
		{
			left_mg.move_voltage(0);
		    right_mg.move_voltage(0);
		}
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Turn: %i", turn);

		// int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		// int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

		// left_mg.move(dir + turn);                      // Sets left motor voltage
		// right_mg.move(dir - turn);                     // Sets right motor voltage
		// // left_mg.move_voltage(turn);
		// right_mg.move_voltage(-turn);

		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "X: %f", odom.position_x);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Y: %f", odom.position_y);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 3, "Heading: %f", odom.rotation);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 4, "Angular Velocty: %f", odom.angular_velocity);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Angular: %f", follower.test1_ang);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "Vel Left: %f", follower.testx_vel);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Vel Right: %f", follower.testy_vel);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "Heading Error: %f", (follower.heading_error_test / M_PI) * 180);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 5, "Velocity: %f", odom.velocity);

		test_function();

		pros::delay(10); // Run for 20 ms then update

		// TODO: change everything (that you can) to references
	}
}

