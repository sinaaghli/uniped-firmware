//
// Created by Michael R. Shannon on 5/2/19.
//

#include "DistanceSensor.h"


namespace slc {

    /** Construct distance sensor.
     *
     * For use only by sublcass.
     *
     * @param zero_offset zero offset in meters
     */
    DistanceSensor::DistanceSensor(float zero_offset)
    : zero_offset_(zero_offset)
    {
    }

    /** Calibrate sensor, sets the value of the current distance.
     *
     * @param meters new value of current distance, in meters, defaults to 0
     */
    void DistanceSensor::calibrate(float meters)
    {
        zero_offset_ = absolute_meters() - meters;
    }

    /** Get absolyute, uncalibrated distance in inches.
     *
     * @return uncalibrated distance in inches
     */
    float DistanceSensor::absolute_inches() const
    {
        return absolute_meters() * 1000.0f / 2.54f;
    }

    /** Get calibrated distance in meters.
     *
     * @return calibrated distance in meters
     */
    float DistanceSensor::meters() const
    {
        return absolute_meters() - zero_offset_;
    }

    /** Get calibrated distance in inches.
     *
     * @return calibrated distance in inches
     */
    float DistanceSensor::inches() const
    {
        return meters() * 1000.0f / 2.54f;
    }

}
