#include <iostream>
#include <cstdio>
#include <thread>
#include <vex.h>
#include <pid.hpp>

/*

control period: add callback functions for automatic actions

*/

vex::motor left_motor_1(vex::PORT3, vex::gearSetting::ratio6_1, true);
vex::motor left_motor_2_top(vex::PORT9, vex::gearSetting::ratio6_1, false);
vex::motor left_motor_2_bottom(vex::PORT2, vex::gearSetting::ratio6_1, true);

vex::motor right_motor_1(vex::PORT6, vex::gearSetting::ratio6_1, false);
vex::motor right_motor_2_top(vex::PORT8, vex::gearSetting::ratio6_1, true);
vex::motor right_motor_2_bottom(vex::PORT5, vex::gearSetting::ratio6_1, false);

vex::motor_group left(left_motor_2_top, left_motor_2_bottom, left_motor_1);
vex::motor_group right(right_motor_2_top, right_motor_2_bottom, right_motor_1);

vex::brain brain;
vex::controller controller;

vex::pneumatics pneumatics(brain.ThreeWirePort.A);

#define BASE_WIDTH 31.7 // centimeters
#define WHEEL_RADIUS 4.15 // centimeters
#define PID_TURN_PARAMS 13.5, 0.01, 0.8, 0.999 // parameter pack (kp, ki, kd, ir)
#define PID_LINEAR_PARAMS 1.25, 0.0002, 0.05, 0.999 // parameter pack (kp, ki, kd, ir)

double x_position, y_position, rotation_value;
double x() { return x_position; }
double y() { return y_position; }
double to_rad(double degree_measure);
double to_deg(double radian_measure);
double rotation() { return to_deg(rotation_value); }
int track();
void set_heading(double heading, double rot_tolerance = 0.0174532925199);
void move(double x, double y, double dist_tolerance = 0.5, double rot_tolerance = 0.0174532925199);
int display();
int controlling();

int main(int argc, const char * argv[]) {
    vex::thread tracking(track);
    // vex::thread control(controlling);
    vex::thread displaying(display);
    // brain.Screen.print("Hi\n");

    // int n = 5;
    // while (n--) {
    //     set_heading(to_rad(90));
    //     vexDelay(3000);
    //     set_heading(to_rad(0));
    //     vexDelay(3000);
    // }
    vexDelay(3000);
    move(10, 0, 0.5, 0.0174532925199);
    vexDelay(3000);
    move(40, 30, 2.5, 0.0154532925199);
    // move(-50, 0);
    tracking.join();
}

int controlling() {
    auto toggle_pneumatics = [] () -> void {
        static bool on = false;
        if (on) {
            pneumatics.close();
            on = false;
        } else {
            pneumatics.open();
            on = true;
        }
    };
    while (true) {
        double fwdpower = controller.Axis3.position();
        double turnpower = controller.Axis1.position();
        left.spin(vex::fwd, fwdpower + turnpower, vex::pct);
        right.spin(vex::fwd, fwdpower - turnpower, vex::pct);
        if (controller.ButtonL1.pressing()) {
            pneumatics.open();
        } else {
            pneumatics.close();
        }
        controller.ButtonL1.pressed(toggle_pneumatics);
    }
    vex::this_thread::sleep_for(10);
}

double to_rad(double degree_measure) {
    static const constexpr double conversion = 3.14159265358979323846 / 180.0;
    return degree_measure * conversion;
}
double to_deg(double radian_measure) {
    static const constexpr double conversion = 180.0 / 3.14159265358979323846;
    return radian_measure * conversion;
}
void set_heading(double heading, double rot_tolerance) {
    auto nearest_coterminal = [] (double rotation, double heading) -> double {
        static double pi2 = M_PI * 2, divpi2 = 1 / pi2;
        return floor((rotation - heading + M_PI) * divpi2) * pi2 + heading;
    };
    double target_rotation = nearest_coterminal(to_rad(rotation()), heading);
    PID turn(PID_TURN_PARAMS);
    turn.init(to_rad(rotation()), target_rotation);
    while (fabs(to_rad(rotation()) - target_rotation) > rot_tolerance) {
        double turn_power = turn.output(to_rad(rotation()));
        // double turn_power = 0;
        right.spin(vex::fwd, turn_power, vex::pct);
        left.spin(vex::fwd, -turn_power, vex::pct);
    }
    left.stop();
    right.stop();
}
void move(double target_x, double target_y, double dist_tolerance, double rot_tolerance) {
    auto distance = [] (double x, double y) -> double {
        return pow(x * x + y * y, 0.5);
    };
    auto nearest_coterminal = [] (double rotation, double heading) -> double {
        static double pi2 = M_PI * 2, divpi2 = 1 / pi2;
        return floor((rotation - heading + M_PI) * divpi2) * pi2 + heading;
    };
    auto get_heading = [] (double x, double y) -> double {
        double heading = atan(y / x);
        if ((x < 0 and y > 0) or (x < 0 and y < 0)) {
            return heading + M_PI;
        } else if (x > 0 and y < 0) {
            return heading + M_PI_2;
        } else {
            return heading;
        }
    };
    double relative_x = target_x - x(), relative_y = target_y - y();
    double target_rotation = nearest_coterminal(to_rad(rotation()), get_heading(relative_x, relative_y));
    set_heading(get_heading(relative_x, relative_y), rot_tolerance);
    vexDelay(1000);
    PID turn(PID_TURN_PARAMS), forward(PID_LINEAR_PARAMS);
    turn.init(get_heading(relative_x, relative_y), target_rotation);
    forward.init(-distance(relative_x, relative_y), 0);
    while (distance(relative_x, relative_y) > dist_tolerance) {
        double fwd_power = forward.output(-distance(relative_x, relative_y));
        // double turn_power = turn.output(to_rad(rotation()));
        double turn_power = 0;
        left.spin(vex::fwd, fwd_power - turn_power, vex::pct);
        right.spin(vex::fwd, fwd_power + turn_power, vex::pct);
        relative_x = target_x - x(), relative_y = target_y - y();
        turn.change_target(nearest_coterminal(to_rad(rotation()), get_heading(relative_x, relative_y)));
    }
    left.stop();
    right.stop();
}

int display() {
    while (true) {
        brain.Screen.clearScreen();
        brain.Screen.setCursor(1, 1);
        brain.Screen.print("%.2f %.2f %.2f", x(), y(), rotation());
        vex::this_thread::sleep_for(10);
    }
}


int track() {
    static const constexpr double base_width = BASE_WIDTH;
    static const constexpr double wheel_radius = WHEEL_RADIUS;
    static const constexpr double degree_radian = 3.14159265358979323846 / 180.0;
    double left_position, right_position;
    double last_left_position, last_right_position;
    double delta_left_position, delta_right_position;
    double delta_rotation;
    double center_radius, local_x_translation, local_offset;
    auto get_left = [] () -> double {
        return (
            (left_motor_2_bottom.position(vex::deg) + left_motor_1.position(vex::deg)) * 0.5 * degree_radian * wheel_radius + 1.84
        ) / 1.68;
    }; // cursed correction factors
    auto get_right = [] () -> double {
        return (
            (right_motor_2_bottom.position(vex::deg) + right_motor_1.position(vex::deg)) * 0.5 * degree_radian * wheel_radius + 1.84
        ) / 1.68;
    }; // cursed correction factors
    // auto get_left = [] () -> double { return ((left_motor_2_bottom.position(vex::deg) + left_motor_1.position(vex::deg)) * 0.5 * degree_radian * wheel_radius); }; // cursed correction factors
    // auto get_right = [] () -> double { return ((right_motor_2_bottom.position(vex::deg) + right_motor_1.position(vex::deg)) * 0.5 * degree_radian * wheel_radius); }; // cursed correction factors
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
        vex::this_thread::sleep_for(10);
    }
    return 0;
}