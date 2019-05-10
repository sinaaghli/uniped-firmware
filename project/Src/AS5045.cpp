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


    /** Construct sensor.
     *
     * @param driver backend AS5045Driver representing the daisy chain
     * @param index index (begining at 0) of this sensor in the daisy chain
     * @param reversed set to true to reverse the encoder direction,
     *                 defaults to false
     * @param zero_offset encoder tick to make the zero angle
     */
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

    /** Get the underlying driver.
     *
     * @return underlying driver of the daisy chain
     */
    std::shared_ptr<AS5045Driver> AS5045::driver()
    {
        return driver_;
    }

    /** Trigger a sample.
     *
     * This actually triggers a sampling of the underlying daisy chain driver
     * and thus only needs to be called for one AS5045 encoder in the the chain.
     *
     * @param blocking true for blocking, false for non-blocking
     * @return sensor status, either success or failed
     */
    Status AS5045::sample(bool blocking)
    {
        status_ = driver_->sample(blocking);
        return status_;
    }

    /** Get status of sensor.
     *
     * @return sensor status, usually success or failed, or working if used
     *                in non-blocking mode and the next sample is not ready yet
     */
    Status AS5045::status() const
    {
        check_for_new_();
        return status_;
    }

    /** Get the current sample count.
     *
     * @return current sample number
     */
    size_t AS5045::sample_count() const
    {
        check_for_new_();
        return sample_count_;
    }

    /** Determine if the underlying driver is busy.
     *
     * When busy a non-blocking sampling is currently working.
     *
     * @return true if busy, false if not
     */
    bool AS5045::busy() const
    {
        return driver_->busy();
    }

    /** Get the raw encoder value (0 to 4096).
     *
     * @return pair of sample count and raw value
     */
    std::pair<size_t, int> AS5045::raw_value() const
    {
        check_for_new_();
        return {sample_count_, raw_position_};
    }

    /** Check for new sample data.
     *
     * For internal use only.
     *
     * This sets the raw_position_ attribute if the driver has a new encoder
     * sample available.
     */
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
