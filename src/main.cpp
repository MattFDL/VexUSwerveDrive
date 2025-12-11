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

#include "drivers.h"
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
pros::Rotation forwardRot(-5); // changed to port 6
// pros::Rotation sidewaysRot(-6);
pros::IMU imu(6); // TODO get the correct port number



pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({4,-3,-2,-1});	  // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({10, -7, 9,8});
pros::MotorGroup intake_mg({-13,-12,-11});

PneumaticCylinder lift('e');
PneumaticCylinder holder('h');
PneumaticCylinder rake('f');
PneumaticCylinder dscore('g');


driverControls driver(master,left_mg,right_mg,intake_mg,0,lift,rake,holder,dscore);


odometry odom(forwardRot, imu);

void initialize()
{
	pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Innit: %f", 1);
	odom.reset_sensors();
	odom.set_start_position(57.14732, 15.58576, 90); // 25.36158, 12.67665, 270); //56.27433, 12.68827
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

std::atomic<std::array<double, 3>> myAtomicStdArray;

// void odometry_thread() {
// 	while (true) {

// 		pros::delay(10);
// 	}

// }; TODO Threading later :----D
// Path path(Point2D(0, 0), Point2D(10, -10), Point2D(20, 10), Point2D(30, 0));
// Path path(Point2D(0, 0), Point2D(40, 0), Point2D(40, 30), Point2D(0, 30));
// Path path2(Point2D(0, 30), Point2D(40, 30), Point2D(40, 0), Point2D(0, 0));

Path path0(Point2D(54.41873, 16.5433),Point2D(54.66749, 46.97948),Point2D(23.09941, 42.71954),Point2D(24.91983, 16.46387));
Path path1(Point2D(24.91983, 16.46387),Point2D(25.08891, 28.65559));
Path path2(Point2D(25.08891, 28.65559),Point2D(25.0889, 39.11803));
Path path3(Point2D(25.0889, 39.11803),Point2D(42.61952, 6.623184),Point2D(37.10316, 112.6259),Point2D(76.78007, 85.47221));


void test_function()
{
	// pros::screen::draw_circle(10, 10, 5);
	// pros::screen::draw_circle(30, 100, 5);
	// pros::screen::draw_circle(100, 200, 5);
	// pros::screen::draw_circle(200, 10, 5);
	for (Point2D num : path0.curvePoints)
	{
		int x = (int)num.x;
		int y = (int)num.y;
		pros::screen::draw_pixel(x, y);
	}
	// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Length: %f", path.curve_length);
}

std::function<bool()> waitCommand(double delay_secs)
{
	const uint32_t start_time = pros::millis();
	auto lambda_wait = [delay_secs, start_time]()
	{
		double current_time_from_start = (pros::millis() - start_time) / 1000.0;
		if (current_time_from_start >= delay_secs)
		{
			return true;
		}
		else
		{
			return false;
		}
	};
	return lambda_wait;
}

std::function<bool()> setPathFollower(PathFollower &f, Path &p)
{
	auto path_set = [&f, &p]()
	{
		f.setPath(p);
		return true;
	};
	return path_set;
}
std::function<bool()> follow_path(PathFollower &f, double vel, bool backwards = false)
{

	auto path_follower = [&f, backwards, vel]()
	{
		bool b = f.followPath(backwards, vel);
		return b;
	};
	return path_follower;
}
std::function<bool()> rotate_to_degrees(PathFollower &f, double deg)
{
	auto rotaton_follower = [&f, deg]()
	{
		bool b = f.rotateToDegrees(deg);
		return b;
	};
	return rotaton_follower;
}
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



void autonomous() {
	PathFollower follower(odom, right_mg, left_mg);
	Autos autos(follower);
	AutoBuilder builder = autos.getAutoRightSide();

	while (true)
	{
		odom.calculate_postition();
		builder.run_commands();

		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "X: %f", odom.position_x);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Y: %f", odom.position_y);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 3, "Heading: %f", odom.rotation);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 4, "Angular Velocty: %f", odom.angular_velocity);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Angular: %f", follower.test1_ang);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "Vel Left: %f", follower.testx_vel);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Vel Right: %f", follower.testy_vel);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "Heading Error: %f", (follower.heading_error_test / M_PI) * 180);
		pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 5, "Velocity: %f", odom.velocity);

		pros::delay(10); // Run for 20 ms then update
		// TODO: change everything (that you can) to references
	}
}



void opcontrol()
{
	while(true){
		driver.handleInputs();
	}
	
}
