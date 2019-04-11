//
// Created by Michael R. Shannon on 4/10/19.
//

#include "tools.h"

namespace slc::tools {

    uint8_t offset_byte(uint8_t *buffer, size_t bit_offset)
    {
        size_t byte_offset = bit_offset / 8;
        bit_offset = bit_offset % 8;
        return (buffer[byte_offset] << bit_offset) |
               (buffer[byte_offset + 1] >> (8 - bit_offset));
    }

}

