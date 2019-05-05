//
// Created by Michael R. Shannon on 5/4/19.
//

#include <utility>

#include "PWM.h"
#include "GPIO.h"
#include "Motor.h"

namespace slc {

    Motor::Motor(PWM enable, GPIO c, GPIO d)
    : enable_(std::move(enable)), c_(std::move(c)), d_(std::move(d))
    {
        drift();
    }

    void Motor::forward(int power)
    {
        c_.write(true);
        d_.write(false);
        enable_.set_duty_cycle(static_cast<float>(power)/100.0f);
    }

    void Motor::reverse(int power)
    {
        c_.write(false);
        d_.write(true);
        enable_.set_duty_cycle(static_cast<float>(power)/100.0f);
    }

    void Motor::stop(int power)
    {
        c_.write(false);
        d_.write(false);
        enable_.set_duty_cycle(static_cast<float>(power)/100.0f);
    }

    void Motor::drift()
    {
        enable_.set_duty_cycle(0.0);
        c_.write(false);
        d_.write(false);
    }
}
