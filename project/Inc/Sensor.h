//
// Created by Michael R. Shannon on 4/9/19.
//

#include <cstdint>
#include <cstddef>
#include "Status.h"

#ifndef SENSOR_H
#define SENSOR_H


namespace slc {

    class Sensor
    {
    public:
        virtual Status sample(bool blocking) = 0;

        virtual Status status() const = 0;

        virtual size_t sample_count() const = 0;

        virtual bool busy() const = 0;

    };

}


#endif //SENSOR_H
