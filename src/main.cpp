#include "main.h"
#include <string>
#include <functional>
#include <cmath>
#include <utility>

#include "SwerveModule.h"
#include "SwerveDrive.h"

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
// pros::Rotation forwardRot(5); // changed to port 6
// // pros::Rotation sidewaysRot(-6);
pros::IMU imu(15); // TODO get the correct port number

pros::Controller master(pros::E_CONTROLLER_MASTER);

// PneumaticCylinder lift('e');
// PneumaticCylinder holder('g');
// PneumaticCylinder rake('f');
// PneumaticCylinder dscore('h');

// driverControls driver(master, left_mg, right_mg, intake_mg, 0, lift, rake, holder, dscore);

// odometry odom(forwardRot, imu);

// Autos autos = Autos();
// PathFollower follower(odom, right_mg, left_mg);

pros::Motor fr_bottom_motor(9);
pros::Motor fr_top_motor(10);
pros::Rotation fr_encoder(-8);

pros::Motor fl_bottom_motor(2);
pros::Motor fl_top_motor(1);
pros::Rotation fl_encoder(-3);

pros::Motor br_bottom_motor(20);
pros::Motor br_top_motor(19);
pros::Rotation br_encoder(-18);

pros::Motor bl_bottom_motor(12);
pros::Motor bl_top_motor(11);
pros::Rotation bl_encoder(-13);

void initialize()
{
	pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Innit: %f", 1);
}

void disabled() {}

void competition_initialize() {}


void autonomous()
{}

void opcontrol()
{

	SwerveModule mod_fr(fr_top_motor, fr_bottom_motor, fr_encoder, 5, -5, 0);
	SwerveModule mod_fl(fl_top_motor, fl_bottom_motor, fl_encoder, 5, 5, 1);
	SwerveModule mod_br(br_top_motor, br_bottom_motor, br_encoder, -5, -5, 2);
	SwerveModule mod_bl(bl_top_motor, bl_bottom_motor, bl_encoder, -5, 5, 3);
	SwerveDrive drive_train(mod_fr, mod_fl, mod_br, mod_bl, imu);
	drive_train.reset_sensors();
	double angle = 0;
	while (true)
	{
		double left_Y=master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    	double left_X=master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		double right_x=master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		double right_y=master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);


		// MODULE TEST CODE
		// double temp = (180 * (std::atan2(left_Y, left_X) / M_PI)) - 90;
		// double temp2 = right_y;
		// if (temp < -90 || temp > -90) {
		// 	angle = temp;
		// }
		// std::pair<double,double> state(angle, temp2);
		//mod_bl.set_state(state);

		drive_train.drive_field_orientated(left_Y, -left_X, right_x);
		
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "X: %f", odom.position_x);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Y: %f", odom.position_y);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 3, "Heading: %f", odom.rotation);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 5, "Velocity: %f", odom.velocity);
		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Ticks: %f", odom.ticks);

		pros::delay(20);
	}
}
