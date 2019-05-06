//
// Created by Michael R. Shannon on 4/3/19.
//

#include <cstdlib>
#include <cmath>
#include <AngularEncoder.h>

#include "tools.h"
#include "AngularEncoder.h"


namespace slc {

    AngularEncoder::AngularEncoder(
            unsigned int positions_per_revolution,
            bool reversed, int zero_offset)
            : revolution_size_(positions_per_revolution),
              reversed_(reversed), zero_offset_(zero_offset)
    {
    }

    void AngularEncoder::calibrate(int position)
    {
        zero_offset_ = raw_value().second - position;
        revolutions_ = 0;
    }

    int AngularEncoder::zero_offset() const
    {
        return zero_offset_;
    }

    std::pair<size_t, int> AngularEncoder::position() const
    {
        auto[count, pos] = raw_value();
        if (count > 1 && std::abs(pos - old_raw_value_) >= revolution_size_/2)
        {
            revolutions_ += pos < old_raw_value_? 1 : -1;
        }
        old_raw_value_ = pos;
        pos += revolution_size_ * revolutions_;
        pos = pos - zero_offset_;
        if (reversed_)
        {
            pos = -pos;
        }
        return {count, pos};
    }

    unsigned int AngularEncoder::revolution_size() const
    {
        return revolution_size_;
    }

    std::pair<size_t, float> AngularEncoder::degrees() const
    {
        auto[count, pos] = position();
        return {count, resolution_degrees() * pos};
    }

    float AngularEncoder::resolution_degrees() const
    {
        return 360.0f / revolution_size_;
    }

    std::pair<size_t, float> AngularEncoder::radians() const
    {
        auto[count, pos] = position();
        return {count, resolution_radians() * pos};
    }

    float AngularEncoder::resolution_radians() const
    {
        return (2.0f * 3.14159265358979323846f) / revolution_size_;
    }

}
