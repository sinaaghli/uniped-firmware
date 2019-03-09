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

        PIDController(double proportional_gain,
                      double integral_gain,
                      double derivative_gain);

        void set_proportional_gain(double gain);

        void set_integral_gain(double gain);

        void set_derivative_gain(double gain);

        void set_target(double target);

        void set_input(double input);

//    void set_plant_errorSetPlantError(double error/*desired-current*/);

        double get_output();

    private:
        double proportional_gain_;
        double integral_gain_;
        double derivative_gain_;
        double integral_value_;
        double target_;
        double previous_error_;
        double current_error_;
        std::chrono::high_resolution_clock::time_point previous_error_time_;
        std::chrono::high_resolution_clock::time_point current_error_time_;
    };

}

#endif //PIDCONTROLLER_H
