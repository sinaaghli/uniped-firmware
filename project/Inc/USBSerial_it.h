//
// Created by Michael R. Shannon on 3/5/19.
//

#ifndef USBSERIAL_IT_H
#define USBSERIAL_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/** Call this inside USB RX interrupt handler.
 *
 * @param buffer buffer of received data
 * @param length length of received data
 * @return number of bytes that where successfully received
 */
size_t usb_receive_it(uint8_t buffer[], size_t length);

#ifdef __cplusplus
}
#endif

#endif //USBSERIAL_IT_H
