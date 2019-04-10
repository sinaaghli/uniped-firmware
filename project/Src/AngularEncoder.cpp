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
        zero_offset_ = raw_position() - position;
    }

    int AngularEncoder::position() const
    {
        return raw_position() - zero_offset_;
    }

    int AngularEncoder::zero_offset() const
    {
        return zero_offset_;
    }

    float AngularEncoder::degrees() const
    {
        return resolution_degrees() * position();
    }

    float AngularEncoder::resolution_degrees() const
    {
        return resolution_;
    }

    float AngularEncoder::radians() const
    {
        return resolution_radians() * position();
    }

    float AngularEncoder::resolution_radians() const
    {
        return resolution_ * 3.14159265358979323846f / 180.0f;
    }

}
