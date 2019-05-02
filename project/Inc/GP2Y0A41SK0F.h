//
// Created by Michael R. Shannon on 5/2/19.
//

#ifndef GP2Y0A41SK0F_H
#define GP2Y0A41SK0F_H

#include <cstdint>

#include "DistanceSensor.h"

namespace slc {

    class GP2Y0A41SK0F : public DistanceSensor
    {
    public:
        explicit GP2Y0A41SK0F(
                uint16_t *raw_value,
                float reference_voltage = 3.0,
                float zero_offset = 0.0);

        float absolute_meters() const override;

    private:
        uint16_t *raw_value_ = nullptr;
        float reference_voltage_;

    };

}

#endif //GP2Y0A41SK0F_H
