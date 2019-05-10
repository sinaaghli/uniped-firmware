//
// Created by Michael R. Shannon on 4/10/19.
//

#include "tools.h"

namespace slc::tools {

    /** Get a byte at an arbitrary bit offset into the given buffer.
     *
     * @param buffer buffer to retrieve byte from
     * @param bit_offset bit offset to retrieve byte at
     * @return byte at the given offset
     */
    uint8_t offset_byte(uint8_t *buffer, size_t bit_offset)
    {
        size_t byte_offset = bit_offset / 8;
        bit_offset = bit_offset % 8;
        return (buffer[byte_offset] << bit_offset) |
               (buffer[byte_offset + 1] >> (8 - bit_offset));
    }

}

