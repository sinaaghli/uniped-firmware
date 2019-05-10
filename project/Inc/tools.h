//
// Created by Michael R. Shannon on 4/10/19.
//

#include <cstdint>
#include <cstddef>

#ifndef TOOLS_H
#define TOOLS_H


namespace slc::tools {

    uint8_t offset_byte(uint8_t *buffer, size_t bit_offset);

    /** Determine bit parity of data in integer width
     *
     * @tparam T type of integer to get parity of
     * @param integer data to get parity of
     * @return true if odd parity, false if even parity
     */
    template<typename T>
    bool parity(T integer)
    {
        bool parity = false;
        while (integer)
        {
            parity = !parity;
            integer = integer & (integer - 1);
        }
        return parity;
    }

    /** Always positive modulus.
     *
     * @tparam T integer type to compute modulus for
     * @param numerator number to take modulus of
     * @param denominator modulus number
     * @return positive modulus
     */
    template<typename T>
    T pmod(T numerator, T denominator)
    {
        return (numerator % denominator + denominator) % denominator;
    }

}


#endif //TOOLS_H
