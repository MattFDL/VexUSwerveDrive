#include "main.h"
#include <string>
#include <functional>
#include <cmath>
#include <utility>

#include "SwerveModule.h"
#include "SwerveDrive.h"
#include "Odometry.h"
#include "Autos.h"
#include "AutoBuilder.h"
#include "intake.h"

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
pros::IMU imu(12); // TODO get the correct port number

pros::Controller master(pros::E_CONTROLLER_MASTER);


// PneumaticCylinder match_start_end_game('a');
// PneumaticCylinder flap('h');
// PneumaticCylinder matchLoad('d');
// PneumaticCylinder descore('b');

PneumaticCylinder lift('e');
PneumaticCylinder descore('b');
// PneumaticCylinder match_start_end_game('a');
// PneumaticCylinder flap('h');

PneumaticCylinder match_start_end_game('h');
PneumaticCylinder flap('f');
PneumaticCylinder matchLoad('d');


// driverControls driver(master, left_mg, right_mg, intake_mg, 0, lift, rake, holder, dscore);

// odometry odom(forwardRot, imu);

// Autos autos = Autos();
// PathFollower follower(odom, right_mg, left_mg);

pros::Motor fr_bottom_motor(6);
pros::Motor fr_top_motor(5);
pros::Rotation fr_encoder(-7);

pros::Motor fl_bottom_motor(15);
pros::Motor fl_top_motor(16);
pros::Rotation fl_encoder(-17); // 11

pros::Motor br_bottom_motor(9);
pros::Motor br_top_motor(8);
pros::Rotation br_encoder(-10);

pros::Motor bl_bottom_motor(19);
pros::Motor bl_top_motor(20);
pros::Rotation bl_encoder(-18);

pros::Rotation horizonal(13);
pros::Rotation vertical(-11);

pros::Motor rollers(-1);
pros::Motor lever(-2);

Odometry odom(horizonal, vertical, imu);

double x_pos = 5;//4.625; // inches forward
double y_pos = 5;	  // inches left/right
SwerveModule mod_fr(fr_top_motor, fr_bottom_motor, fr_encoder, x_pos, -y_pos, 0, false);
SwerveModule mod_fl(fl_top_motor, fl_bottom_motor, fl_encoder, x_pos, y_pos, 1, false);
SwerveModule mod_br(br_top_motor, br_bottom_motor, br_encoder, -x_pos, -y_pos, 2, false);
SwerveModule mod_bl(bl_top_motor, bl_bottom_motor, bl_encoder, -x_pos, y_pos, 3, true); //no match load

// SwerveModule mod_fr(fr_top_motor, fr_bottom_motor, fr_encoder, x_pos, -y_pos, 0, false);
// SwerveModule mod_fl(fl_top_motor, fl_bottom_motor, fl_encoder, x_pos, y_pos, 1, true);
// SwerveModule mod_br(br_top_motor, br_bottom_motor, br_encoder, -x_pos, -y_pos, 2, false);
// SwerveModule mod_bl(bl_top_motor, bl_bottom_motor, bl_encoder, -x_pos, y_pos, 3, true);

SwerveDrive drive_train(mod_fr, mod_fl, mod_br, mod_bl, odom);
Autos autos = Autos();

Intake intake = Intake(rollers, lever, lift, flap);
//AutoBuilder builder = autos.matchLoadAuto(drive_train, odom, intake, matchLoad);
AutoBuilder builder = autos.noMatchLoadAuto(drive_train, odom, intake);

// SET THIS DEPENDING ON THE ROBOT
bool noMatchLoad = true;

void initialize()
{
	pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Innit: %f", 1);
	odom.reset_sensors();

	match_start_end_game.extend();
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{

	match_start_end_game.extend();

	while (true)
	{
		odom.calculate_position();
		//builder.run_commands();
		if (!drive_train.auto_in_use)
		{
			drive_train.stop_motors();
		}
		intake.handle_intake();
		pros::delay(20);
	}
}

void handle_inputs()
{
	double left_Y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	double left_X = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	double right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	double right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	// drive_train.drive_field_orientated(left_Y, -left_X, right_x);

	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		intake.lift_lever();
	} // R2 Outtake

	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		intake.run_rollers();
	} // L2 Intake
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
	{
		intake.toggle_lift();
	} // R1 Lift
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		match_start_end_game.toggle();
	} //  DOWN match endgame
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
	{
		intake.stop_rollers();
	} // B stop
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		intake.reverse_rollers();
	} // Y reverse rollers

	if (noMatchLoad)
	{
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			descore.toggle();
		} // Descore for one robot, flap toggle for the other one
	}
	else
	{
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			flap.toggle();
		}
	}
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
	{
		matchLoad.toggle();
	} //
}

void opcontrol()
{

	match_start_end_game.extend();

	while (true)
	{
		odom.calculate_position();
		double left_Y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double left_X = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		double right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		double right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		drive_train.drive_field_orientated(left_Y, -left_X, right_x);		

		intake.handle_intake();
		handle_inputs();

		// pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Ticks: %f", odom.ticks);
		pros::delay(20);
	}
}
