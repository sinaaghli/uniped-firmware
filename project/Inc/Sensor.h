//
// Created by Michael R. Shannon on 4/9/19.
//

#include <cstdint>
#include <cstddef>
#include "Status.h"

#ifndef SENSOR_H
#define SENSOR_H


namespace slc {

    /** Sensor interface.
     *
     */
    class Sensor
    {
    public:
        /** Trigger a sample of the sensor.
         *
         * If in blocking mode the new sensor value will be available upon
         * return.
         *
         * @param blocking true for blocking sample or false for non blocking
         * @return sensor status
         */
        virtual Status sample(bool blocking) = 0;

        /** Get the sensor status.
         *
         * @return status of the sensor
         */
        virtual Status status() const = 0;

        /** Get the number of the current sample.
         *
         * This is the number of samples taken with the sensor since the last
         * reset.
         *
         * @return current sample number
         */
        virtual size_t sample_count() const = 0;

        /** Determine if the sensor is busy.
         *
         * When not busy the sample method can be called to sample a new value.
         *
         * @return true if busy, false if not
         */
        virtual bool busy() const = 0;

    };

}


#endif //SENSOR_H
