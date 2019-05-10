//
// Created by Michael R. Shannon on 3/4/19.
//

#ifndef USBSERIAL_H
#define USBSERIAL_H

#include <cstdint>
#include <usbd_def.h>
#include "BytesBuffer.h"
#include "CharacterDevice.h"


namespace slc {

    /** USB serial device.
     *
     * Allows bidirectional communication over USB.
     *
     */
    class USBSerial : public virtual CharacterDevice
    {
    public:
        explicit USBSerial(USBD_HandleTypeDef *usb);

        bool write_ready() const override;

        bool read_ready() const override;

        bool failed() const override;

        void reset() override;

        size_t write(uint8_t buffer[], size_t length) override;

        size_t read(uint8_t buffer[], size_t length) override;

        void register_it();

        size_t recieve_it(uint8_t buffer[], size_t length);

    private:
        USBD_HandleTypeDef *usb_;
        BytesBuffer buffer_;
        bool failed_ = false;
        static const size_t BUFFER_SIZE_ = 4096;
    };

}


#endif //USBSERIAL_H
