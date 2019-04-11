//
// Created by Michael R. Shannon on 4/3/19.
//

#include <cmath>
#include "tools.h"
#include "AngularEncoder.h"


namespace slc {

    AngularEncoder::AngularEncoder(
            unsigned int positions_per_revolution,
            bool reversed, int zero_offset)
            : positions_per_revolution_(positions_per_revolution),
            reversed_(reversed), zero_offset_(zero_offset)
    {
    }

    void AngularEncoder::calibrate(int position)
    {
        zero_offset_ = raw_position().second - position;
    }

    int AngularEncoder::zero_offset() const
    {
        return zero_offset_;
    }

    std::pair<size_t, int> AngularEncoder::position() const
    {
        auto [count, raw_pos] = raw_position();
        auto pos = raw_pos - zero_offset_;
        if (reversed_)
        {
            pos = positions_per_revolution_ - pos;
        }
        pos = tools::pmod(pos, static_cast<int>(positions_per_revolution_));
        return {count, pos};
    }

    unsigned int AngularEncoder::postions_per_revolution() const
    {
        return positions_per_revolution_;
    }

    std::pair<size_t, float> AngularEncoder::degrees() const
    {
        auto [count, pos] = position();
        return {count, resolution_degrees() * pos};
    }

    float AngularEncoder::resolution_degrees() const
    {
        return 360.0f/positions_per_revolution_;
    }

    std::pair<size_t, float> AngularEncoder::radians() const
    {
        auto [count, pos] = position();
        return {count, resolution_radians() * pos};
    }

    float AngularEncoder::resolution_radians() const
    {
        return (2.0f * 3.14159265358979323846f) / positions_per_revolution_;
    }

}
