//
// Created by Michael R. Shannon on 5/2/19.
//

#ifndef INA169_H
#define INA169_H

#include <cstdint>

#include <CurrentSensor.h>

namespace slc {

    class INA169 : public CurrentSensor
    {
    public:
        explicit INA169(uint16_t *raw_value, float reference_voltage=3.0,
                        float shunt=10.0f, float output=10000.0f);

        float amps() const override;
    private:
        uint16_t *raw_value_ = nullptr;
        float reference_voltage_;
        float shunt_;
        float output_;

    };

}

#endif //INA169_H
