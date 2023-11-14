#include <iostream>
#include <cstdio>
#include <thread>
#include <vex.h>

vex::motor left_motor_1(vex::PORT3, vex::gearSetting::ratio6_1, true);
vex::motor left_motor_2_top(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor left_motor_2_bottom(vex::PORT2, vex::gearSetting::ratio6_1, true);

vex::motor right_motor_1(vex::PORT6, vex::gearSetting::ratio6_1, false);
vex::motor right_motor_2_top(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor right_motor_2_bottom(vex::PORT5, vex::gearSetting::ratio6_1, false);

vex::motor_group left(left_motor_2_top, left_motor_2_bottom, left_motor_1);
vex::motor_group right(right_motor_2_top, right_motor_2_bottom, right_motor_1);
vex::brain brain;
vex::controller controller;

#define BASE_WIDTH 31.7 // centimeters
#define WHEEL_RADIUS 4.15 // centimeters

double x_position, y_position, rotation_value;
double x() { return x_position; }
double y() { return y_position; }
double to_rad(double degree_measure);
double to_deg(double radian_measure);
double rotation() { return to_deg(rotation_value); }
void track(void);
void pos_read(void);

int main(int argc, const char * argv[]) {
    vex::thread tracking(track);
    while (true) {
        brain.Screen.print("%.2lf %.2lf %.2lf", x(), y(), rotation());
    }
    tracking.join();
}


double to_rad(double degree_measure) {
    static const constexpr double conversion = 3.14159265358979323846 / 180.0;
    return degree_measure * conversion;
}
double to_deg(double radian_measure) {
    static const constexpr double conversion = 180.0 / 3.14159265358979323846;
    return radian_measure * conversion;
}
void track(void) {
    const constexpr double base_width = BASE_WIDTH;
    const constexpr double wheel_radius = WHEEL_RADIUS;
    const constexpr double degree_radian = 3.14159265358979323846 / 180.0;
    double left_position, right_position;
    double last_left_position, last_right_position;
    double delta_left_position, delta_right_position;
    double delta_rotation;
    double center_radius, local_x_translation, local_offset;
    auto get_left = [] () -> double { return (left_motor_2_bottom.position(vex::deg) + left_motor_1.position(vex::deg)) * 0.5 * degree_radian * wheel_radius; }; // factor halved (unsure on source of doubling currently)
    auto get_right = [] () -> double { return (right_motor_2_bottom.position(vex::deg) + right_motor_1.position(vex::deg)) * 0.5 * degree_radian * wheel_radius; }; // factor halved
    x_position = 0.0;
    y_position = 0.0;
    rotation_value = 0.0;
    last_left_position = get_left();
    last_right_position = get_right();
    while (true) {
        left_position = get_left();
        right_position = get_right();
        delta_left_position = left_position - last_left_position;
        delta_right_position = right_position - last_right_position;
        last_left_position = left_position;
        last_right_position = right_position;
        delta_rotation = (delta_right_position - delta_left_position) / base_width;
        local_offset = rotation_value + delta_rotation * 0.5;
        if (delta_rotation != 0.0) {
            center_radius = delta_left_position / delta_rotation + base_width * 0.5;
            local_x_translation = 2.0 * sin(delta_rotation * 0.5) * center_radius;
        } else {
            local_x_translation = delta_left_position;
        }
        x_position += local_x_translation * cos(local_offset);
        y_position += local_x_translation * sin(local_offset);
        rotation_value += delta_rotation;
    }
}
void pos_read(void) {
    while (true) {
        vexDelay(2000);
        std::cout << x() << ' ' << y() << ' ' << rotation() << '\n';
        brain.Screen.print("%.2lf %.2lf %.2lf", x(), y(), rotation());
    }
}