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

    // internal list of instances
    std::map<SPI_HandleTypeDef *, SerialPeripheralInterface *> spi_objects_;

    /** Create a serial peripheral interface object.
     *
     * @param spi pointer to the underlying SPI structure
     * @param chip_select_port pointer to GPIO port structure of chip select pin
     * @param chip_select_pin GPIO pin define of chip select pin
     */
    SerialPeripheralInterface::SerialPeripheralInterface(
            SPI_HandleTypeDef *spi,
            GPIO_TypeDef *chip_select_port, uint16_t chip_select_pin)
            : spi_(spi), chip_select_port_(chip_select_port),
              chip_select_pin_(chip_select_pin)
    {
        // TODO: change this to use the GPIO class
        // register object
        spi_objects_[spi_] = this;
    }

    /** Destructor, removes the instance from the internal list of instances.
     *
     */
    SerialPeripheralInterface::~SerialPeripheralInterface()
    {
        // unregister object
        spi_objects_.erase(spi_);
    }

    /** Register a function to call when data is recieved.
     *
     * @param callback callback function, it will be given the location of a
     *                 buffer of data and the length of the data in that
     *                 buffer, upon return the buffer is invalid
     */
    void SerialPeripheralInterface::register_callback(
            std::function<void(void *, size_t)> callback)
    {
        callback_ = callback;
    }

    /** Remove the callback.
     *
     * @return true if there was a callback, false otherwise
     */
    bool SerialPeripheralInterface::unregister_callback()
    {
        bool previous = callback_.has_value();
        callback_ = {};
        return previous;
    }

    /** Read the SPI device.
     *
     * @param buffer a buffer to write the received data into
     * @param bytes maximum number of bytes to read
     * @param complete_flag pointer to an atomic bool to set to true when the
     *                      read is complete, places the SPI device in
     *                      non-blocking mode if given
     * @return status of SPI device, success, failed, or working (if used in
     *         non-blocking mode)
     */
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

    /** Is the SPI device ready.
     *
     * @return true if the SPI device can be read from
     */
    bool SerialPeripheralInterface::ready() const
    {
        return spi_->State == HAL_SPI_STATE_READY;
    }

    /** Is the SPI device busy.
     *
     * @return true if the SPI device is currently reading data.
     */
    bool SerialPeripheralInterface::busy() const
    {
        return !ready();
    }

    /** End a SPI transfer, deselects chip and calls callback if any.
     *
     * For internal use only.
     *
     */
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
