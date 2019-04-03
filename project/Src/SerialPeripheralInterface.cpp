//
// Created by Michael R. Shannon on 4/2/19.
//

#include <cstdint>
#include <functional>
#include <map>
#include <utility>
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
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

    bool SerialPeripheralInterface::read(
            void *buffer, size_t bytes, bool *complete_flag)
    {
        HAL_StatusTypeDef status = HAL_OK;

        // return error if busy
        if (!ready())
        {
            return false;
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

        return status == HAL_OK;
    }

    bool SerialPeripheralInterface::ready()
    {
        return spi_->State == HAL_SPI_STATE_READY;
    }

    void SerialPeripheralInterface::complete_read_()
    {
        // disable chip select
        HAL_GPIO_WritePin(chip_select_port_, chip_select_pin_, GPIO_PIN_SET);
        if (callback_.has_value())
        {
            callback_.value()(buffer_, bytes_);
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

    *(spi_object->complete_flag_) = true;
    spi_object->complete_read_();
}
