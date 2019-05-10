//
// Created by Michael R. Shannon on 5/4/19.
//

#include <stm32f4xx_hal_gpio.h>

#include "GPIO.h"

namespace slc {

    /** Create GPIO object.
     *
     * @param GPIOx pointer to GPIO port structure
     * @param pin GPIO pin define
     */
    GPIO::GPIO(GPIO_TypeDef *GPIOx, uint16_t pin)
    : GPIOx_(GPIOx), pin_(pin)
    {
    }

    /** Read the value of the pin.
     *
     * @retval true the pin is logic high
     * @retval false the pin is logic low
     */
    bool GPIO::read() const
    {
       return HAL_GPIO_ReadPin(GPIOx_, pin_) == GPIO_PIN_SET;
    }

    /** Write to the pin.
     *
     * @param value pin value to write, false for logic low, true for logic high
     */
    void GPIO::write(bool value)
    {
        HAL_GPIO_WritePin(GPIOx_, pin_, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /** Toggle the pin.
     *
     */
    void GPIO::toggle()
    {
        HAL_GPIO_TogglePin(GPIOx_, pin_);
    }

}
