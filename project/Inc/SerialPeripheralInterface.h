//
// Created by Michael R. Shannon on 4/2/19.
//

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "SerialPeripheralInterface_it.h"
#include "NonCopyable.h"
#include "Status.h"

#ifndef SERIALPERIPHERALINTERFACE_H
#define SERIALPERIPHERALINTERFACE_H

namespace slc {

    class SerialPeripheralInterface : public NonCopyable
    {
        friend void ::spi_rx_complete_it(SPI_HandleTypeDef *spi);

    public:

        SerialPeripheralInterface(
                SPI_HandleTypeDef *spi,
                GPIO_TypeDef *chip_select_port, uint16_t chip_select_pin);

        ~SerialPeripheralInterface();

        void register_callback(std::function<void(void *, size_t)> callback);

        bool unregister_callback();

        Status read(void *buffer, size_t bytes,
                std::atomic_bool *complete_flag = nullptr);

        bool ready() const;

        bool busy() const;

        SerialPeripheralInterface(SerialPeripheralInterface &&) = default;
        SerialPeripheralInterface &operator=(
                SerialPeripheralInterface &&) = default;

    private:
        SPI_HandleTypeDef *spi_;
        GPIO_TypeDef *chip_select_port_;
        uint16_t chip_select_pin_;
        std::optional<std::function<void(void *, size_t)>> callback_;
        std::atomic_bool *complete_flag_ = nullptr;
        void *buffer_ = nullptr;
        size_t bytes_ = 0;

        void complete_read_();
    };


}


#endif //SERIALPERIPHERALINTERFACE_H
