//
// Created by Michael R. Shannon on 3/4/19.
//

#include <algorithm>
#include <cassert>
#include <USBSerial.h>

#include "usbd_cdc_if.h"
#include "USBSerial.h"
#include "USBSerial_it.h"


slc::USBSerial *usb_it_ = NULL;


namespace slc {

    /** Create USB serial device
     *
     * @param usb pointer to USB device structure
     */
    USBSerial::USBSerial(USBD_HandleTypeDef *usb)
            : usb_(usb), buffer_(BUFFER_SIZE_)
    {

    }

    /** Determine if the device is ready to be written to.
     *
     * @return true if the USB device is available for writing
     */
    bool USBSerial::write_ready() const
    {
        auto hcdc = static_cast<USBD_CDC_HandleTypeDef *>(usb_->pClassData);
        return !failed_ && (hcdc->TxState == 0);
    }

    /** Determine if the device is ready to be read from.
     *
     * @return true if the USB device is available for reading
     */
    bool USBSerial::read_ready() const
    {
        return !buffer_.empty();
    }

    /** Determine if the device is in a failed state.
     *
     * @return true if device is in a failed state, true otherwise
     */
    bool USBSerial::failed() const
    {
        return failed_;
    }

    /** Reset the device.
     *
     * This is here only to satisfy the CharacterDevice interface.
     * The USB device cannot be reset.
     *
     */
    void USBSerial::reset()
    {
        // NO RESET POSSIBLE
    }

    /** Write data to the device.
     *
     * @param buffer data to write
     * @param length number of bytes to write
     * @return number of bytes actually written.
     */
    size_t USBSerial::write(uint8_t *buffer, size_t length)
    {
        length = std::min(length, static_cast<size_t>(UINT8_MAX));
        auto length16 = static_cast<uint16_t>(length);
        auto hcdc = static_cast<USBD_CDC_HandleTypeDef *>(usb_->pClassData);
        if (hcdc->TxState != 0)
        {
            return 0;
        }
        USBD_CDC_SetTxBuffer(usb_, buffer, length16);
        auto error = USBD_CDC_TransmitPacket(usb_);
        switch (error)
        {
            case USBD_OK:
                return length;
            case USBD_FAIL:
                failed_ = true;
            case USBD_BUSY:
            default:
                return 0;
        }
    }

    /** Read bytes from device.
     *
     * @param buffer buffer to write into
     * @param length maximum number of bytes to read
     * @return number of bytes actually read
     */
    size_t USBSerial::read(uint8_t *buffer, size_t length)
    {
        return buffer_.read(buffer, length);
    }

    /** Register this instance with the interrupt handler function.
     *
     * Only one instance can be registered at a time.
     *
     */
    void USBSerial::register_it()
    {
        usb_it_ = this;
    }

    /** Called from inside the interrupt handler.
     *
     * For internal use only.
     *
     * @param buffer data received from USB device
     * @param length length of data in bytes
     * @return number of bytes successfully received
     */
    size_t USBSerial::recieve_it(uint8_t *buffer, size_t length)
    {
        return buffer_.write(buffer, length);
    }

}


extern "C" size_t usb_receive_it(uint8_t buffer[], size_t length)
{
    if (usb_it_ == nullptr)
    {
        return 0;
    }
    return usb_it_->recieve_it(buffer, length);
}
