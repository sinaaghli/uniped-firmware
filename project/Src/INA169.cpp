//
// Created by Michael R. Shannon on 5/2/19.
//

#include "INA169.h"

namespace slc {

    INA169::INA169(uint16_t *raw_value, float reference_voltage,
                   float shunt, float output)
            : raw_value_(raw_value), reference_voltage_(reference_voltage),
              shunt_(shunt), output_(output)
    {
    }

    float INA169::amps() const
    {
        float voltage = (*raw_value_) * reference_voltage_ / 4096.0f;
        return (voltage * 1000.0f) / (shunt_ * output_);
    }

}
