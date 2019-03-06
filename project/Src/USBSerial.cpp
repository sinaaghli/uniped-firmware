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

    USBSerial::USBSerial(USBD_HandleTypeDef *usb)
            : usb_(usb), buffer_(BUFFER_SIZE_)
    {

    }

    bool USBSerial::write_ready() const
    {
        auto hcdc = static_cast<USBD_CDC_HandleTypeDef *>(usb_->pClassData);
        return !failed_ && (hcdc->TxState == 0);
    }

    bool USBSerial::read_ready() const
    {
        return !buffer_.empty();
    }

    bool USBSerial::failed() const
    {
        return failed_;
    }

    void USBSerial::reset()
    {
        // NO RESET POSSIBLE
    }

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

    size_t USBSerial::read(uint8_t *buffer, size_t length)
    {
        return buffer_.read(buffer, length);
    }

    void USBSerial::register_it()
    {
        usb_it_ = this;
    }

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
