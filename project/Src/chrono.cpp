//
// Created by Michael R. Shannon on 3/8/19.
// Based on: https://forum.atollic.com/viewtopic.php?t=923#p1877
//

#include <chrono>
#include <stm32f4xx_hal.h>

extern TIM_HandleTypeDef htim5;

namespace std::chrono {

    steady_clock::time_point steady_clock::now() noexcept
    {
        // will rollover after 49.7 days
        return time_point(milliseconds(HAL_GetTick()));
    }

    high_resolution_clock::time_point high_resolution_clock::now() noexcept
    {
        // will rollover after 1 hour, 11 minutes, and 58 seconds
        return time_point(microseconds(__HAL_TIM_GetCounter(&htim5)));
    }

}
