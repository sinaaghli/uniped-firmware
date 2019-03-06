//
// Created by Michael R. Shannon on 3/4/19.
//

#include <cstddef>
#include <cstdint>

#ifndef CHARACTERDEVICE_H
#define CHARACTERDEVICE_H

namespace slc {

    /** Character device interface.
     *
     */
    class CharacterDevice
    {
    public:
        /** Determine if the device can be written to.
         *
         * @retval true: can write to the device
         * @retval false: cannot write to the device
         */
        virtual bool write_ready() const = 0;

        /** Determine if the device can be read from.
         *
         * @retval true: can read from the device
         * @retval false: cannot read from the device
         */
        virtual bool read_ready() const = 0;

        /** Determine if the device is failed and should be reset.
         *
         * @retval true: device is in some failed state
         * @retval false: device is functioning properly
         */
        virtual bool failed() const = 0;

        /** Attempt to reset the device, clearing any errors.
         *
         * This will drop any partially received or transmitted packets.
         *
         */
        virtual void reset() = 0;

        /** Write bytes to the device.
         *
         * Will not block if attempting to write more bytes than possible.
         *
         * @param buffer bytes to write to the device
         * @param length number of bytes to write from the buffer
         * @return number of bytes written to the device
         */
        virtual size_t write(uint8_t buffer[], size_t length) = 0;

        /** Read bytes from the device.
         *
         * Will not block if attempting to read more bytes than necessary.
         *
         * @param buffer buffer to write bytes into
         * @param length number of bytes to read
         * @return number of bytes read from the device
         */
        virtual size_t read(uint8_t buffer[], size_t length) = 0;

        virtual ~CharacterDevice() = default;
    };

}

#endif //CHARACTERDEVICE_H
