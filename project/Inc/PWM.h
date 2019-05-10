//
// Created by Michael R. Shannon on 5/4/19.
//

#ifndef PWM_H
#define PWM_H

#include <stm32f4xx_hal_tim.h>

namespace slc {

    /** Object oriented interface to a PWM output.
     *
     */
    class PWM
    {
    public:
        PWM(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t period);
        void set_duty_cycle(float duty_cycle);

    private:
        TIM_HandleTypeDef *htim_;
        uint32_t channel_;
        uint32_t period_;

    };

}

#endif //PWM_H
