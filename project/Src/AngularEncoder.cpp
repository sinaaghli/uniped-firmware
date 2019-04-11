//
// Created by Michael R. Shannon on 4/3/19.
//

#include <cmath>
#include "AngularEncoder.h"


namespace slc {

    AngularEncoder::AngularEncoder(float resolution, int zero_offset)
    {
        resolution_ = resolution;
        zero_offset_ = zero_offset;
    }

    void AngularEncoder::calibrate(int position)
    {
        zero_offset_ = raw_position().second - position;
    }

    std::pair<size_t, int> AngularEncoder::position() const
    {
        auto [count, raw_pos] = raw_position();
        return {count, raw_pos - zero_offset_};
    }

    int AngularEncoder::zero_offset() const
    {
        return zero_offset_;
    }

    std::pair<size_t, float> AngularEncoder::degrees() const
    {
        auto [count, pos] = position();
        return {count, resolution_degrees() * pos};
    }

    float AngularEncoder::resolution_degrees() const
    {
        return resolution_;
    }

    std::pair<size_t, float> AngularEncoder::radians() const
    {
        auto [count, pos] = position();
        return {count, resolution_radians() * pos};
    }

    float AngularEncoder::resolution_radians() const
    {
        return resolution_ * 3.14159265358979323846f / 180.0f;
    }

}
