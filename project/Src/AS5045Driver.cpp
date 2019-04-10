//
// Created by Michael R. Shannon on 4/5/19.
//

#include <cstdint>
#include <memory>
#include "Status.h"
#include "AS5045Driver.h"


namespace slc {

    AS5045Driver::AS5045Driver(
            SerialPeripheralInterface spi, size_t encoders_)
            : spi_(std::move(spi)), encoders(encoders_),
              status_(Status::idle),
              buffer_length_((bits_per_encoder * encoders + 7) / 8),
              spi_buffer_(std::make_unique<uint8_t[]>(buffer_length_)),
              sample_buffer_(std::make_unique<uint8_t[]>(buffer_length_))
    {
    }

    Status AS5045Driver::sample(bool blocking)
    {
        swap_if_complete_();

        if (!spi_.ready())
        {
            return Status::failed;
        }

        spi_read_complete_ = false;

        if (blocking)
        {
            status_ = spi_.read(spi_buffer_.get(), buffer_length_);
            if (status_ == Status::success)
            {
                swap_buffers_();
            }
            return status_;
        }

        status_ = spi_.read(spi_buffer_.get(), buffer_length_, &spi_read_complete_);
        return status_;
    }

    std::size_t AS5045Driver::sample_count() const
    {
        swap_if_complete_();
        return sample_count_;
    }

    Status AS5045Driver::status() const
    {
        swap_if_complete_();
        return status_;
    }

    bool AS5045Driver::busy() const
    {
        return spi_.busy();
    }

    uint8_t *AS5045Driver::buffer()
    {
        swap_if_complete_();

        if (sample_count_ == 0)
        {
            return nullptr;
        }

        return sample_buffer_.get();
    }

    void AS5045Driver::swap_buffers_() const
    {
        sample_buffer_.swap(spi_buffer_);
        ++sample_count_;
    }

    void AS5045Driver::swap_if_complete_() const
    {
        bool complete = true;
        if (spi_read_complete_.compare_exchange_strong(complete, false))
        {
            swap_buffers_();
        }
    }


}
