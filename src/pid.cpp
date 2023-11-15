#include <pid.hpp>

PID::PID(double kp, double ki, double kd, double ir) {
    proportionality_constant = kp;
    integral_constant = ki;
    derivative_constant = kd;
    integral_ratio = ir;
    error_sum = 0.0;
}

void PID::change_target(double target) {
    target_value = target;
}

void PID::init(double current_value, double target) {
    target_value = target;
    last_error = target - current_value;
}

double PID::output(double current_value) {
    error = target_value - current_value;
    error_sum = error_sum * integral_ratio + error;
    proportional_component = error * proportionality_constant;
    integral_component = error_sum * integral_constant;
    derivative_component = (error - last_error) * derivative_constant;
    last_error = error;
    return proportional_component + integral_component + derivative_component;
}