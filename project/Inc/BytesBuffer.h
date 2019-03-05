//
// Created by mrshannon on 3/4/19.
//

#include <cstddef>
#include <cstdint>

#ifndef BYTESBUFFER_H
#define BYTESBUFFER_H

namespace slc {

    /** Circular bytes buffer.
     *
     */
    class BytesBuffer
    {
    public:
        explicit BytesBuffer(size_t capacity);

        ~BytesBuffer();

        size_t write(const uint8_t source[], size_t count);

        size_t read(uint8_t dest[], size_t count);

        void reset();

        bool empty() const;

        bool full() const;

        size_t capacity() const;

        size_t size() const;

    private:
        size_t head_ = 0;
        size_t tail_ = 0;
        const size_t capacity_;
        uint8_t *buffer_;
    };

}

#endif //BYTESBUFFER_H
