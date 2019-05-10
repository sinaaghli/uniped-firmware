//
// Created by Michael R. Shannon on 5/2/19.
//

#include "INA169.h"

namespace slc {

    /** Construct INA169 current sensor object.
     *
     * If using the SparkFun breakout board only the first argument is required.
     *
     * @param raw_value pointer to raw value location in memory
     *                  (from 12-bit ADC)
     * @param reference_voltage reference voltage of ADC, defaults to 3.0 volts
     * @param shunt shunt resistor value in ohms, defaults to 10 ohms
     * @param output output resistor value in ohms, defaults to 10 kilo-ohms
     */
    INA169::INA169(uint16_t *raw_value, float reference_voltage,
                   float shunt, float output)
            : raw_value_(raw_value), reference_voltage_(reference_voltage),
              shunt_(shunt), output_(output)
    {
    }

    /** Get the current in amperes.
     *
     * @return current in amperes
     */
    float INA169::amps() const
    {
        float voltage = (*raw_value_) * reference_voltage_ / 4096.0f;
        return (voltage * 1000.0f) / (shunt_ * output_);
    }

}
