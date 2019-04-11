//
// Created by Michael R. Shannon on 4/10/19.
//

#include <cstdint>
#include <cstddef>

#ifndef TOOLS_H
#define TOOLS_H


namespace slc::tools {

    uint8_t offset_byte(uint8_t *buffer, size_t bit_offset);

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

}


#endif //TOOLS_H
