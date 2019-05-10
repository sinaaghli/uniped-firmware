//
// Created by Michael R. Shannon on 5/2/19.
//

#include <cmath>
#include <cstdint>

#include "GP2Y0A41SK0F.h"


namespace slc {

    /** Construct the optical distance sensor.
     *
     * @param raw_value pointer to raw value location in memory
     *                  (from 12-bit ADC)
     * @param reference_voltage reference voltage of ADC, defaults to 3.0 volts
     * @param zero_offset zero offset to use
     */
    GP2Y0A41SK0F::GP2Y0A41SK0F(
            uint16_t *raw_value, float reference_voltage, float zero_offset)
    : DistanceSensor(zero_offset),
    reference_voltage_(reference_voltage), raw_value_(raw_value)
    {
    }

    /** Get absolute, uncalibrated distance in meters.
     *
     * @return uncalibrated distance in meters
     */
    float GP2Y0A41SK0F::absolute_meters() const
    {
        float voltage = (*raw_value_)*reference_voltage_/4096.0f;
        // determined from Tools/fit_distance_voltage.py
        float centimeters =
                powf(15.3504349560002f/(voltage + 0.14812358860854632f),
                        1.2379065907953468f);
        return centimeters/100.0f;
    }

}
