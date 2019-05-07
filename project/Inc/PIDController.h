//
// Created by Michael R. Shannon on 3/8/19.
// Adapted from: https://github.com/sinaaghli/spirit/blob/master/include
//                  /spirit/Controllers/spPID.h
//

#include <chrono>

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

namespace slc {

    /** A simple PID controller.
     *
     */
    class PIDController
    {
    public:
        PIDController();

        PIDController(float proportional_gain,
                      float integral_gain,
                      float derivative_gain);

        void set_proportional_gain(float gain);

        void set_integral_gain(float gain);

        void set_derivative_gain(float gain);

        void set_target(float target);

        void set_input(float input);

        float get_target();

        float get_output();

    private:
        float proportional_gain_;
        float integral_gain_;
        float derivative_gain_;
        float integral_value_;
        float target_;
        float previous_error_ = 0;
        float current_error_ = 0;
        std::chrono::high_resolution_clock ::time_point previous_error_time_;
        std::chrono::high_resolution_clock ::time_point current_error_time_;
    };

}

#endif //PIDCONTROLLER_H
