//
// Created by Michael R. Shannon on 5/4/19.
//

#include <cstdint>

#include "PWM.h"


namespace slc {

    PWM::PWM(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t period)
    : htim_(htim), channel_(channel), period_(period)
    {
        set_duty_cycle(0);
    }

    void PWM::set_duty_cycle(float duty_cycle)
    {
        TIM_OC_InitTypeDef sConfigOC = {0};
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = static_cast<uint32_t>(period_ * duty_cycle);
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        HAL_TIM_PWM_ConfigChannel(htim_, &sConfigOC, channel_);
        HAL_TIM_PWM_Start(htim_, channel_);
    }

}
