//
// Created by Michael R. Shannon on 4/2/19.
//

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <utility>
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "Status.h"
#include "SerialPeripheralInterface.h"


namespace slc {

    std::map<SPI_HandleTypeDef *, SerialPeripheralInterface *> spi_objects_;

    SerialPeripheralInterface::SerialPeripheralInterface(
            SPI_HandleTypeDef *spi,
            GPIO_TypeDef *chip_select_port, uint16_t chip_select_pin)
            : spi_(spi), chip_select_port_(chip_select_port),
              chip_select_pin_(chip_select_pin)
    {
        // register object
        spi_objects_[spi_] = this;
    }

    SerialPeripheralInterface::~SerialPeripheralInterface()
    {
        // unregister object
        spi_objects_.erase(spi_);
    }

    void SerialPeripheralInterface::register_callback(
            std::function<void(void *, size_t)> callback)
    {
        callback_ = callback;
    }

    bool SerialPeripheralInterface::unregister_callback()
    {
        bool previous = callback_.has_value();
        callback_ = {};
        return previous;
    }

    Status SerialPeripheralInterface::read(
            void *buffer, size_t bytes, std::atomic_bool *complete_flag)
    {
        HAL_StatusTypeDef status = HAL_OK;

        // return error if busy
        if (!ready())
        {
            return Status::failed;
        }

        // select chip
        HAL_GPIO_WritePin(chip_select_port_, chip_select_pin_, GPIO_PIN_RESET);

        if (complete_flag == nullptr)
        {
            // blocking receive
            status = HAL_SPI_Receive(
                    spi_, static_cast<uint8_t *>(buffer),
                    static_cast<uint16_t>(bytes), 10000);
            complete_read_();
        }
        else
        {
            // receive with interrupt
            complete_flag_ = complete_flag;
            *complete_flag_ = false;
            buffer_ = buffer;
            bytes_ = bytes;
            status = HAL_SPI_Receive_IT(
                    spi_, static_cast<uint8_t *>(buffer),
                    static_cast<uint16_t>(bytes));
        }

        switch (status)
        {
            case HAL_OK:
                if (complete_flag == nullptr)
                {
                    return Status::success;
                }
                return Status::working;
            default:
                return Status::failed;
        }
    }

    bool SerialPeripheralInterface::ready() const
    {
        return spi_->State == HAL_SPI_STATE_READY;
    }

    bool SerialPeripheralInterface::busy() const
    {
        return !ready();
    }

    void SerialPeripheralInterface::complete_read_()
    {
        // disable chip select
        HAL_GPIO_WritePin(chip_select_port_, chip_select_pin_, GPIO_PIN_SET);
        if (callback_.has_value())
        {
            // buffer_ is no longer changing outside of context
            callback_.value()(const_cast<void *>(buffer_), bytes_);
        }
    }

}


extern "C" void spi_rx_complete_it(SPI_HandleTypeDef *spi)
{
    slc::SerialPeripheralInterface *spi_object = nullptr;
    try
    {
        spi_object = slc::spi_objects_.at(spi);
    }
    catch (const std::out_of_range &)
    {
        return;
    }

    if (spi_object == nullptr)
    {
        return;
    }

    spi_object->complete_flag_->store(true);
    spi_object->complete_read_();
}
