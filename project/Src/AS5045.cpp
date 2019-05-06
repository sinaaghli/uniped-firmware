//
// Created by Michael R. Shannon on 4/4/19.
//

#include <functional>
#include <utility>
#include <sstream>
#include "tools.h"
#include "Status.h"
#include "AS5045.h"


namespace slc {


    AS5045::AS5045(std::shared_ptr<AS5045Driver> driver,
                   size_t index, bool reversed, int zero_offset)
            : AngularEncoder(AS5045::positions_per_revolution, reversed, zero_offset),
              index_(index), driver_(std::move(driver))
    {
        if (!driver_)
        {
            throw std::invalid_argument("'driver' cannot be null");
        }

        if (index_ >= driver_->encoders)
        {
            std::ostringstream message;
            message << "'index' = " << index
                    << " must be less than the number of encoders ("
                    << driver_->encoders << ") in the driver";
            throw std::out_of_range(message.str());
        }
    }

    std::shared_ptr<AS5045Driver> AS5045::driver()
    {
        return driver_;
    }

    Status AS5045::sample(bool blocking)
    {
        status_ = driver_->sample(blocking);
        return status_;
    }

    Status AS5045::status() const
    {
        check_for_new_();
        return status_;
    }

    size_t AS5045::sample_count() const
    {
        check_for_new_();
        return sample_count_;
    }

    bool AS5045::busy() const
    {
        return driver_->busy();
    }

    std::pair<size_t, int> AS5045::raw_value() const
    {
        check_for_new_();
        return {sample_count_, raw_position_};
    }

    void AS5045::check_for_new_() const
    {
        if (driver_->sample_count() > sample_count_)
        {
            auto [count, data] = driver_->data(index_);

            if (((data & 0x38U) != 0x20) || tools::parity(data))
            {
                status_ = Status::failed;
                return;  // latest sample is invalid
            }

            sample_count_ = count;
            raw_position_ = static_cast<uint16_t>((data >> 6U) & 0xFFFU);
            status_ = Status::success;
        }
    }

}
