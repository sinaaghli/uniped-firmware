//
// Created by Michael R. Shannon on 5/4/19.
//

#ifndef GPIO_H
#define GPIO_H

#include <cstdint>

#include "stm32f407xx.h"

namespace slc {

    class GPIO
    {
    public:
        GPIO(GPIO_TypeDef *GPIOx, uint16_t pin);
        bool read() const;
        void write(bool value);
        void toggle();

    private:
        GPIO_TypeDef *GPIOx_ = nullptr;
        uint16_t pin_;
    };

}

#endif //GPIO_H
