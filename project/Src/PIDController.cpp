//
// Created by Michael R. Shannon on 3/8/19.
// Adapted from: https://github.com/sinaaghli/spirit/blob/master/src/spPID.cpp
//

#include <chrono>
#include "PIDController.h"

namespace slc {

    /** Construct a PID controller with zero gains.
     *
     */
    PIDController::PIDController()
    {
        proportional_gain_ = 0.0;
        integral_gain_ = 0.0;
        derivative_gain_ = 0.0;
        integral_value_ = 0.0;
    }

    /** Construct a PID controller.
     *
     * @param proportional_gain proportional gain of controller
     * @param integral_gain integral gain of controller
     * @param derivative_gain derivative gain of controller
     */
    PIDController::PIDController(double proportional_gain,
                                 double integral_gain,
                                 double derivative_gain)
            : proportional_gain_(proportional_gain),
              integral_gain_(integral_gain),
              derivative_gain_(derivative_gain)
    {
        integral_value_ = 0.0;
    }

    /** Set the proportional gain.
     *
     * @param gain new proportional gain of the controller
     */
    void PIDController::set_proportional_gain(double gain)
    {
        proportional_gain_ = gain;
    }

    /** Set the integral gain.
     *
     * @param gain new integral gain of the controller
     */
    void PIDController::set_integral_gain(double gain)
    {
        integral_gain_ = gain;
    }

    /** Set the derivative gain.
     *
     * @param gain new derivative gain of the controller
     */
    void PIDController::set_derivative_gain(double gain)
    {
        derivative_gain_ = gain;
    }

    /** Set the target value.
     *
     * @param target target value
     */
    void PIDController::set_target(double target)
    {
        target_ = target;
    }

    /** Set controller input, the current value.
     *
     * @param input current value
     */
    void PIDController::set_input(double input)
    {
        previous_error_time_ = current_error_time_;
        current_error_time_ = std::chrono::high_resolution_clock::now();
        previous_error_ = current_error_;
        current_error_ = target_ - input;
    }

    /** Get controller output, control signal.
     *
     * @return output control signal
     */
    double PIDController::get_output()
    {
        double delta_time =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                        previous_error_time_ - current_error_time_).count() *
                        0.000001;
        double delta_error = current_error_ - previous_error_;
        double proportional = proportional_gain_ * current_error_;
        double derivative = derivative_gain_ * (delta_error / delta_time);
        integral_value_ += previous_error_ * delta_time;
        double integral = integral_gain_ * integral_value_;
        return proportional + integral + derivative;
    }

}

