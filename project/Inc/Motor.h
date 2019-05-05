//
// Created by Michael R. Shannon on 5/4/19.
//

#ifndef MOTOR_H
#define MOTOR_H

#include <memory>

#include "PWM.h"
#include "GPIO.h"

namespace slc {

    class Motor
    {
    public:

        Motor(PWM enable, GPIO c, GPIO d);

        void forward(int power = 100);

        void reverse(int power = 100);

        void stop(int power = 100);

        void drift();

    private:
        PWM enable_;
        GPIO c_;
        GPIO d_;

    };

}

#endif //MOTOR_H
