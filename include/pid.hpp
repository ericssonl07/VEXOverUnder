#ifndef PID_HPP
#define PID_HPP

class PID {
    private:
        double proportionality_constant, integral_constant, derivative_constant, integral_ratio;
        double proportional_component, integral_component, derivative_component;
        double target_value;
        double current_value;
        double error;
        double error_sum;
        double last_error;
    public:
        PID(double kp = 1.0, double ki = 0.0, double kd = 0.0, double ir = 0.999);
        void change_target(double target);
        void init(double current_value, double target);
        double output(double current_value);
};

#endif