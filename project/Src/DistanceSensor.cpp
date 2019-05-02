//
// Created by Michael R. Shannon on 5/2/19.
//

#include "DistanceSensor.h"


namespace slc {

    DistanceSensor::DistanceSensor(float zero_offset)
    : zero_offset_(zero_offset)
    {
    }

    float DistanceSensor::calibrate(float meters)
    {
        zero_offset_ = absolute_meters() - meters;
    }

    float DistanceSensor::absolute_inches() const
    {
        return absolute_meters() * 1000.0f / 2.54f;
    }

    float DistanceSensor::meters() const
    {
        return absolute_meters() - zero_offset_;
    }

    float DistanceSensor::inches() const
    {
        return meters() * 1000.0f / 2.54f;
    }

}
