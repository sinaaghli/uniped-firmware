//
// Created by Michael R. Shannon on 5/4/19.
//

#include <stm32f4xx_hal_gpio.h>

#include "GPIO.h"

namespace slc {

    GPIO::GPIO(GPIO_TypeDef *GPIOx, uint16_t pin)
    : GPIOx_(GPIOx), pin_(pin)
    {
    }

    bool GPIO::read() const
    {
       return HAL_GPIO_ReadPin(GPIOx_, pin_) == GPIO_PIN_SET;
    }

    void GPIO::write(bool value)
    {
        HAL_GPIO_WritePin(GPIOx_, pin_, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void GPIO::toggle()
    {
        HAL_GPIO_TogglePin(GPIOx_, pin_);
    }

}
