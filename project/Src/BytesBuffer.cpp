//
// Created by mrshannon on 3/4/19.
//

#include <cstring>
#include <algorithm>

#include "IRQLock.h"
#include <BytesBuffer.h>


namespace slc {

    /** Construct a bytes buffer of a given size.
     *
     * @param capacity maximum number of elements to store
     */
    BytesBuffer::BytesBuffer(size_t capacity) : capacity_(capacity)
    {
        buffer_ = new uint8_t[capacity_];
    }

    BytesBuffer::~BytesBuffer()
    {
        delete buffer_;
    }

    /** Write bytes to the buffer.
     *
     * If the buffer is full then only a portion or possibly none of the given
     * bytes will be written.
     *
     * @param source buffer to read the bytes from
     * @param count number of bytes to write
     * @return number of bytes written, less than count if buffer becomes filled
     */
    size_t BytesBuffer::write(const uint8_t *source, size_t count)
    {
        IRQLock lock;
        size_t written = 0;
        size_t to_write = 0;
        while ((count > 0) && !full_())
        {
            if (tail_ == 0)
            {
                to_write = std::min(count, capacity_ - head_ - 1);
            }
            else if (head_ < tail_)
            {
                to_write = std::min(count, tail_ - head_ - 1);
            }
            else
            {
                to_write = std::min(count, capacity_ - head_);
            }
            memcpy(&buffer_[head_], source, to_write);
            head_ = (head_ + to_write) % capacity_;
            written += to_write;
            count -= to_write;
            source += to_write;
        }
        return written;
    }

    /** Read bytes from the buffer.
     *
     * @param dest buffer to write bytes into
     * @param count number of bytes to read
     * @return number of bytes read, less than count if buffer becomes empty
     */
    size_t BytesBuffer::read(uint8_t *dest, size_t count)
    {
        IRQLock lock;
        size_t read = 0;
        size_t to_read = 0;
        while ((count > 0) && !empty_())
        {
            if (head_ < tail_)
            {
                to_read = std::min(count, capacity_ - tail_);
            }
            else
            {
                to_read = std::min(count, head_ - tail_);
            }
            memcpy(dest, &buffer_[tail_], to_read);
            tail_ = (tail_ + to_read) % capacity_;
            read += to_read;
            count -= to_read;
            dest += to_read;
        }
        return read;
    }

    /** Reset the buffer, erasing it.
     *
     */
    void BytesBuffer::reset()
    {
        IRQLock lock;
        head_ = 0;
        tail_ = 0;
    }

    bool BytesBuffer::empty_() const
    {
        return head_ == tail_;
    }

    /** Determine if the buffer is empty.
     *
     * @retval true: buffer is empty
     * @retval false: buffer is not empty
     */
    bool BytesBuffer::empty() const
    {
        IRQLock lock;
        return empty_();
    }

    bool BytesBuffer::full_() const
    {
        return (head_ + 1) % capacity_ == tail_;
    }

    /** Determine if the buffer is full.
     *
     * @retval true: buffer is full
     * @retval false: buffer is not full
     */
    bool BytesBuffer::full() const
    {
        IRQLock lock;
        return full_();
    }

    /** Get maximum capacity of the buffer.
     *
     * @return maximum number of elements in buffer
     */
    size_t BytesBuffer::capacity() const
    {
        IRQLock lock;
        return capacity_;
    }

    /** Get number of elements in the buffer.
     *
     * @return number of elements in the buffer
     */
    size_t BytesBuffer::size() const
    {
        IRQLock lock;
        if (head_ < tail_)
        {
            return capacity_ - tail_ + head_;
        }
        return head_ - tail_;
    }

}
