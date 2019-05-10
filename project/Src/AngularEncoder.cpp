//
// Created by Michael R. Shannon on 4/3/19.
//

#include <cstdlib>
#include <cmath>
#include <chrono>

#include "tools.h"
#include "AngularEncoder.h"


namespace slc {

    /** Construct angular encoder, only for use by subclass.
     *
     * @param revolution_size number of encoder ticks per revolution
     * @param reversed set to true to reverse the encoder direction, does
     *                 not effect raw_value, defaults to false
     * @param zero_offset encoder tick to make the zero angle
     */
    AngularEncoder::AngularEncoder(
            unsigned int revolution_size,
            bool reversed, int zero_offset)
            : revolution_size_(revolution_size),
              reversed_(reversed), zero_offset_(zero_offset)
    {
    }

    /** Calibrate the encoder to use a zero offset.
     *
     * @param position current position (in encoder ticks), defaults to 0
     */
    void AngularEncoder::calibrate(int position)
    {
        zero_offset_ = raw_value().second - position;
        revolutions_ = 0;
    }

    /** Get the zero offset in encoder ticks.
     *
     * @return zero offset in encoder ticks
     */
    int AngularEncoder::zero_offset() const
    {
        return zero_offset_;
    }

    /** Get the current calibrated position (in encoder ticks).
     *
     * @return pair of sample count and calibrated position
     */
    std::pair<size_t, int> AngularEncoder::position() const
    {
        auto new_time = std::chrono::high_resolution_clock::now();
        auto[count, pos] = raw_value();
        if (count > 1)
        {
            if (std::abs(pos - old_raw_value_) >= revolution_size_/2)
            {
                revolutions_ += pos < old_raw_value_? 1 : -1;
            }
            float delta_time =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                            new_time - old_time_).count() * 0.000060f;
            rpm_ = (static_cast<float>(pos - old_raw_value_)/revolution_size_
                    )/delta_time;
        }
        old_raw_value_ = pos;
        old_time_ = new_time;
        pos += revolution_size_ * revolutions_;
        pos = pos - zero_offset_;
        if (reversed_)
        {
            pos = -pos;
        }
        return {count, pos};
    }

    /** Get number of encoder ticks per 360 degree revolution.
     *
     * @return ticks per revolution
     */
    unsigned int AngularEncoder::revolution_size() const
    {
        return revolution_size_;
    }

    /** Get the current continuous angle of the encoder in degrees.
     *
     * @return pair of sample count and angle (continuous) in degrees
     */
    std::pair<size_t, float> AngularEncoder::degrees() const
    {
        auto[count, pos] = position();
        return {count, resolution_degrees() * pos};
    }

    /** Get the number of degrees per encoder tick.
     *
     * @return degrees per encoder tick
     */
    float AngularEncoder::resolution_degrees() const
    {
        return 360.0f / revolution_size_;
    }

    /** Get the current continuous angle of the encoder in radians.
     *
     * @return pair of sample count and angle (continuous) in radians
     */
    std::pair<size_t, float> AngularEncoder::radians() const
    {
        auto[count, pos] = position();
        return {count, resolution_radians() * pos};
    }

    /** Get the number of radians per encoder tick.
     *
     * @return radians per encoder tick
     */
    float AngularEncoder::resolution_radians() const
    {
        return (2.0f * 3.14159265358979323846f) / revolution_size_;
    }

    /** Get current RPM.
     *
     * @return pair of sample count and angular velocity in RPM
     */
    std::pair<size_t, float> AngularEncoder::rpm() const
    {
        // TODO: this does not work well
        return {sample_count(), rpm_};
    }

}
