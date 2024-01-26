#include <iostream>
#include <cstdio>
#include <thread>
#include <vex.h>
#include <pid.hpp>

// DEVICES

    vex::brain brain;
    vex::controller controller;

    vex::pneumatics front_wings(brain.ThreeWirePort.A);
    vex::pneumatics lift(brain.ThreeWirePort.B);
    vex::pneumatics back_wings(brain.ThreeWirePort.C);
    vex::pneumatics elevation(brain.ThreeWirePort.D);

    vex::motor catapult(vex::PORT19, vex::gearSetting::ratio36_1, false);
    vex::motor intake(vex::PORT13, vex::gearSetting::ratio6_1, true);

    vex::motor left_motor_1(vex::PORT3, vex::gearSetting::ratio6_1, false);
    vex::motor left_motor_2(vex::PORT9, vex::gearSetting::ratio6_1, false);
    vex::motor left_motor_3(vex::PORT8, vex::gearSetting::ratio6_1, false);

    vex::motor right_motor_1(vex::PORT11, vex::gearSetting::ratio6_1, true);
    vex::motor right_motor_2(vex::PORT10, vex::gearSetting::ratio6_1, true);
    vex::motor right_motor_3(vex::PORT5, vex::gearSetting::ratio6_1, true);

    vex::motor_group left(left_motor_1, left_motor_2, left_motor_3);
    vex::motor_group right(right_motor_1, right_motor_2, right_motor_3);

// unit conversions
const double rad_to_deg = 180 / M_PI;
const double deg_to_rad = 1 / rad_to_deg;
#define DEG * deg_to_rad
#define RAD * rad_to_deg

// drivetrain parameters
#define BASE_WIDTH 31.7 // centimeters
#define WHEEL_RADIUS 4.15 // centimeters
#define PID_TURN_PARAMS 6.5, 0.01, 0.8, 0.999 // parameter pack (kp, ki, kd, ir)
#define PID_LINEAR_PARAMS 1.25, 0.0002, 0.05, 0.999 // parameter pack (kp, ki, kd, ir)

// odometry and motion functions
double x_position, y_position, rotation_value;
double x() { return x_position; }
double y() { return y_position; }
double rotation() { return rotation_value; }
int track();
void set_heading(double heading, double rot_tolerance = 0.0174532925199);
void move(double x, double y, double dist_tolerance = 3, double rot_tolerance = 0.0174532925199);
int display();
int controlling();
void init();
void turn(double angle);
void fwd(double dist);
void bwd(double dist);
double powercap(double power);
void autonomous();

int main(int argc, const char * argv[]) {
    init();
    vex::thread tracking(track);
    vex::thread displaying(display);
    autonomous();
    vex::thread control(controlling);
}

void autonomous() {
    front_wings.open();
    vexDelay(200);
    left.spin(vex::fwd, 100, vex::pct);
    right.spin(vex::fwd, 80, vex::pct);
    vexDelay(1000);
}

// Set the heading to a specified value (accounts for coterminality)
void set_heading(double heading, double rot_tolerance) {
    auto nearest_coterminal = [] (double rotation, double heading) -> double {
        static double pi2 = M_PI * 2, divpi2 = 1 / pi2;
        return floor((rotation - heading + M_PI) * divpi2) * pi2 + heading;
    };
    double target_rotation = nearest_coterminal(rotation(), heading);
    PID turn(PID_TURN_PARAMS);
    turn.init(rotation(), target_rotation);
    while (fabs(rotation() - target_rotation) > rot_tolerance) {
        double turn_power = powercap(turn.output(rotation()));
        right.spin(vex::fwd, turn_power, vex::pct);
        left.spin(vex::fwd, -turn_power, vex::pct);
        vex::this_thread::sleep_for(1);
    }
    left.stop();
    right.stop();
    vexDelay(100);
}
// Move the robot to (`target_x`, `target_y`) in Cartesian coordinates
void move(double target_x, double target_y, double dist_tolerance, double rot_tolerance) {
    auto distance = [] (double x, double y) -> double {
        return pow(x * x + y * y, 0.5);
    };
    auto nearest_coterminal = [] (double rotation, double heading) -> double {
        static double pi2 = M_PI * 2, divpi2 = 1 / pi2;
        return floor((rotation - heading + M_PI) * divpi2) * pi2 + heading;
    };
    auto get_heading = [] (double x, double y) -> double {
        if (x == 0) {
            if (y < 0) {
                return -M_PI_2;
            } else {
                return M_PI_2;
            }
        }
        double heading = atan(y / x);
        if (y == 0 and x < 0) {
            return M_PI;
        }
        if ((x < 0 and y > 0) or (x < 0 and y < 0)) {
            return heading + M_PI;
        } else if (x > 0 and y < 0) {
            return heading + M_PI_2;
        } else {
            return heading;
        }
    };
    double relative_x = target_x - x(), relative_y = target_y - y();
    double target_rotation = nearest_coterminal(rotation(), get_heading(relative_x, relative_y));
    double adjustment_dampening = 0.05;
    if (fabs(target_rotation - rotation()) <= M_PI_2) {
        set_heading(target_rotation, rot_tolerance);
        PID turn(PID_TURN_PARAMS), forward(PID_LINEAR_PARAMS);
        turn.init(rotation(), target_rotation);
        forward.init(-distance(relative_x, relative_y), 0);
        while (distance(relative_x, relative_y) > dist_tolerance) {
            double fwd_power = powercap(forward.output(-distance(relative_x, relative_y)));
            double turn_power = turn.output(rotation()) * adjustment_dampening;
            left.spin(vex::fwd, fwd_power - turn_power, vex::pct);
            right.spin(vex::fwd, fwd_power + turn_power, vex::pct);
            relative_x = target_x - x(), relative_y = target_y - y();
            turn.change_target(nearest_coterminal(rotation(), get_heading(relative_x, relative_y)));
            vex::this_thread::sleep_for(10);
        }
    } else {
        set_heading(target_rotation + M_PI, rot_tolerance);
        PID turn(PID_TURN_PARAMS), backward(PID_LINEAR_PARAMS);
        turn.init(rotation(), target_rotation + M_PI);
        backward.init(distance(relative_x, relative_y), 0);
        while (distance(relative_x, relative_y) > dist_tolerance) {
            double bwd_power = powercap(backward.output(distance(relative_x, relative_y)));
            double turn_power = turn.output(rotation()) * adjustment_dampening;
            left.spin(vex::fwd, bwd_power - turn_power, vex::pct);
            right.spin(vex::fwd, bwd_power + turn_power, vex::pct);
            relative_x = target_x - x(), relative_y = target_y - y();
            turn.change_target(nearest_coterminal(rotation(), get_heading(relative_x, relative_y) + M_PI));
            vex::this_thread::sleep_for(10);
        }
    }
    left.stop();
    right.stop();
    vexDelay(100);
}


// Turn `angle` radians
void turn(double angle) {
    set_heading(rotation() + angle);
}

// Move `dist` centimeters forward at current heading
void fwd(double dist) {
    double theta = rotation();
    move(x() + dist * cos(theta), y() + dist * sin(theta));
}

// Move `dist` centimeters backward at current heading
void bwd(double dist) {
    double theta = rotation() + M_PI;
    move(x() + dist * cos(theta), y() + dist * sin(theta));
}

/*
Initialization function for encoders:
 - left motors set to zero
 - right motors set to zero
*/
void init() {
    left_motor_1.resetPosition();
    left_motor_2.resetPosition();
    left_motor_3.resetPosition();
    right_motor_1.resetPosition();
    right_motor_2.resetPosition();
    right_motor_3.resetPosition();
}

/*
Stops devices:
 - left motor group
 - right motor group
 - intake motor
 - catapult motor
*/
void stop() {
    left.stop();
    right.stop();
    intake.stop();
    catapult.stop();
}

/*
Power processing function which limits `power` such that it satisfies:
0.25 ≤ |power| ≤ 20.00
*/
double powercap(double power) {
    bool neg = false;
    if (power < 0) {
        neg = true;
    }
    power = fabs(power);
    power = std::max(0.25, power);
    power = std::min(10.0, power);
    if (neg) {
        power = -power;
    }
    return power;
}

/*
Control period function.
Controller binds:
 - Axis 3 -> Forward/Backward power
 - Axis 1 -> Turning
 - L1 -> Wings (toggle)
 - L2 -> Catapult (hold)
 - R1 -> Intake out (hold)
 - R2 -> Intake in (hold)
 - A -> Lift (toggle)
 - B -> Elevation (toggle)
*/
int controlling() {
    bool lastL1State = 0;
    bool toggleWings = 0;

    bool lastAState = 0;
    bool toggleLift = 0;

    bool lastBState = 0;
    bool toggleElevation = 0;

    while (true) {
        double fwdpower = controller.Axis3.position();
        double turnpower = controller.Axis1.position() * 0.4;
        left.spin(vex::fwd, fwdpower + turnpower, vex::pct);
        right.spin(vex::fwd, fwdpower - turnpower, vex::pct);

        //toggle wings
        if(controller.ButtonL1.pressing() != lastL1State && lastL1State != 1){
            if(toggleWings){
                toggleWings = 0;
            } else{
                toggleWings = 1;
            }
        }
        if (toggleWings) {
            front_wings.open();
        } else {
            front_wings.close();
        }
        lastL1State = controller.ButtonL1.pressing();

        // toggleLift
        if(controller.ButtonA.pressing() != lastAState && lastAState != 1){
            if(toggleLift){
                toggleLift = 0;
            } else{
                toggleLift = 1;
            }
        }
        if (toggleLift) {
            lift.open();

        } else {
            lift.close();
        }
        lastAState = controller.ButtonA.pressing();

        // toggleLift
        if(controller.ButtonB.pressing() != lastBState && lastBState != 1){
            if(toggleElevation){
                toggleElevation = 0;
            } else{
                toggleElevation = 1;
            }
        }
        if (toggleElevation) {
            elevation.open();

        } else {
            elevation.close();
        }
        lastBState = controller.ButtonB.pressing();

        if(controller.ButtonL2.pressing()) {
            catapult.spin(vex::fwd, 100, vex::pct);
        }
        else {
            catapult.spin(vex::fwd, 0, vex::pct);
        }

        // intake
        if(controller.ButtonR2.pressing()) {
            intake.spin(vex::fwd, -100, vex::pct);
        }

        if(controller.ButtonR1.pressing()) {
            intake.spin(vex::fwd, 100, vex::pct);
        }
 
        if(!controller.ButtonR2.pressing() && !controller.ButtonR1.pressing()) {
            intake.spin(vex::fwd, 0, vex::pct);
        }
        
        vex::this_thread::sleep_for(10);
    }
    return 0;
}

// Display debug information
int display() {
    while (true) {
        brain.Screen.clearScreen();
        brain.Screen.setCursor(1, 1);
        brain.Screen.print("%.2f %.2f %.2f", x(), y(), rotation());
        controller.Screen.clearScreen();
        controller.Screen.setCursor(1, 1);
        controller.Screen.print("%.2f %.2f %.2f", x(), y(), rotation());
        printf("%.2f %.2f %.2f\n", x(), y(), rotation());

        brain.Screen.setCursor(2, 1);
        brain.Screen.print(
            "Left: %.2f %.2f %.2f", 
            left_motor_1.position(vex::deg),
            left_motor_2.position(vex::deg),
            left_motor_3.position(vex::deg)
        );
        brain.Screen.setCursor(3, 1);
        brain.Screen.print(
            "Right: %.2f %.2f %.2f",
            right_motor_1.position(vex::deg),
            right_motor_2.position(vex::deg),
            right_motor_3.position(vex::deg)
        );

        printf("Left: %.2f %.2f %.2f\n", 
            left_motor_1.position(vex::deg),
            left_motor_2.position(vex::deg),
            left_motor_3.position(vex::deg));
        printf("Right: %.2f %.2f %.2f\n\n",
            right_motor_1.position(vex::deg),
            right_motor_2.position(vex::deg),
            right_motor_3.position(vex::deg));

        vex::this_thread::sleep_for(100);
    }
}

// Odometry tracking function (runs in a background thread)
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
            (left_motor_3.position(vex::deg) + left_motor_2.position(vex::deg) + left_motor_1.position(vex::deg)) * 0.3333333333 * degree_radian * wheel_radius + 1.84
        ) / 1.68;
    }; // cursed correction factors
    auto get_right = [] () -> double {
        return (
            (right_motor_3.position(vex::deg) + right_motor_2.position(vex::deg) + right_motor_1.position(vex::deg)) * 0.3333333333 * degree_radian * wheel_radius + 1.84
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
        vex::this_thread::sleep_for(1);
    }
    return 0;
}