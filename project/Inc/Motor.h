//
// Created by Michael R. Shannon on 5/4/19.
//

#ifndef MOTOR_H
#define MOTOR_H

#include <memory>

#include "PWM.h"
#include "GPIO.h"

namespace slc {

    /** Regular motor.
     *
     * Allows control of a DC motor attached to an H bridge.
     *
     */
    class Motor
    {
    public:

        Motor(PWM enable, GPIO c, GPIO d);

        virtual void forward(int power);

        virtual void reverse(int power);

        virtual void stop(int power);

        virtual void drift();

    private:
        PWM enable_;
        GPIO c_;
        GPIO d_;

    };

}

#endif //MOTOR_H
