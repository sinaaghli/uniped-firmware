//
// Created by Michael R. Shannon on 5/4/19.
//

#include <utility>

#include "PWM.h"
#include "GPIO.h"
#include "Motor.h"

namespace slc {

    /** Construct a DC motor object.
     *
     * @param enable PWM object connected to the enable pin the H bridge
     * @param c input 1 of the H bridge
     * @param d input 2 of the H bridge
     */
    Motor::Motor(PWM enable, GPIO c, GPIO d)
    : enable_(std::move(enable)), c_(std::move(c)), d_(std::move(d))
    {
        drift();
    }

    /** Turn on motor in the forward direction.
     *
     * @param power power between 0 (min) and 100 (max)
     */
    void Motor::forward(int power)
    {
        c_.write(true);
        d_.write(false);
        enable_.set_duty_cycle(static_cast<float>(power)/100.0f);
    }

    /** Turn on motor in the reverse direction.
     *
     * @param power power between 0 (min) and 100 (max)
     */
    void Motor::reverse(int power)
    {
        c_.write(false);
        d_.write(true);
        enable_.set_duty_cycle(static_cast<float>(power)/100.0f);
    }

    /** Enable breaking mode.
     *
     * @param power power between 0 (min) and 100 (max)
     */
    void Motor::stop(int power)
    {
        c_.write(false);
        d_.write(false);
        enable_.set_duty_cycle(static_cast<float>(power)/100.0f);
    }

    /** Enable free floating mode.
     *
     */
    void Motor::drift()
    {
        enable_.set_duty_cycle(0.0);
        c_.write(false);
        d_.write(false);
    }
}
