//
// Created by Michael R. Shannon on 4/5/19.
//

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>
#include "Status.h"
#include "Sensor.h"
#include "SerialPeripheralInterface.h"

#ifndef AS5045DRIVER_H
#define AS5045DRIVER_H


namespace slc {

    class AS5045Driver : public virtual Sensor
    {
    public:
        static const size_t bits_per_encoder = 19;
        const size_t encoders;

        explicit AS5045Driver(
                SerialPeripheralInterface spi,
                size_t encoders = 1);

        Status sample(bool blocking) override;

        std::size_t sample_count() const override;

        Status status() const override;

        bool busy() const override;

        std::pair<size_t, uint32_t> data(size_t encoder = 0);

    private:
        SerialPeripheralInterface spi_;
        Status status_;
        mutable size_t sample_count_ = 0;
        mutable std::atomic_bool spi_read_complete_ = false;
        const size_t buffer_length_;  // must be before buffers
        mutable std::unique_ptr<uint8_t[]> spi_buffer_;
        mutable std::unique_ptr<uint8_t[]> sample_buffer_;

        void swap_buffers_() const;

        void swap_if_complete_() const;

    };

}


#endif //AS5045DRIVER_H
