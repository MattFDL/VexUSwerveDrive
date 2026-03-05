#include "Odometry.h"

Odometry::Odometry(pros::Rotation &hr, pros::Rotation &vr, pros::IMU &i) : horizontal_rotation(hr), vertical_rotation(vr), imu(i) {

}
void Odometry::reset_sensors() {
    horizontal_rotation.reset(); //* DO NOT PUT IN CONSTRUCTOR PLEASE THIS CRASHES THINGS
    horizontal_rotation.reset_position();
    vertical_rotation.reset();
    vertical_rotation.reset_position();
    imu.set_heading(180);
    imu.reset(true); //blocking
}
void Odometry::calculate_position() {
    double horizontal_inches_count = (static_cast<double>(horizontal_rotation.get_position()) / HORIZONTAL_SENSOR_TICKS_TO_INCHES);
    double vertical_inches_count = (static_cast<double>(vertical_rotation.get_position()) / VERTICAL_SENSOR_TICKS_TO_INCHES);

    double deltaX = vertical_inches_count - vertical_inches_previous;
    double deltaY = horizontal_inches_count - horizontal_inches_previous;

    double heading = get_imu_reading() * (M_PI / 180);

    position_x = position_x + ((deltaX * cos(heading)) + (deltaY * sin(heading)));
    position_y = position_y - ((deltaY * cos(heading)) - (deltaX * sin(heading)));
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 0, "Position X: %f", position_x);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Position Y: %f", position_y);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Heading: %f", heading);

    vertical_inches_previous = vertical_inches_count;
    horizontal_inches_previous = horizontal_inches_count;
    
}
double Odometry::get_imu_reading() {
    double reading_adjusted = (180 - imu.get_heading());
    return reading_adjusted;
}
double Odometry::get_position_x() {
    return position_x;
}
double Odometry::get_position_y() {
    return position_y;
}
void Odometry::set_starting_position(Pose p) {
    position_x = p.x;
    position_y = p.y;
    imu.set_heading((180 - p.heading));
}