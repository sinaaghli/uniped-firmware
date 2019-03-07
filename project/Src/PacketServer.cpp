//
// Created by Michael R. Shannon on 3/5/19.
//

#include <cstring>
#include <functional>
#include <memory>
#include <utility>
#include <stm32f4xx_hal.h>
#include "PacketServer.h"

namespace slc {

    /** Construct a packet server.
     *
     * @param device character device to use for sending/receiving
     * @param crc handle to the CRC peripheral
     * @param magic_byte magic byte that begins every packet
     * @param max_packet_size maximum size of the data portion of a packet
     */
    PacketServer::PacketServer(
            std::unique_ptr<CharacterDevice> device,
            CRC_HandleTypeDef &crc,
            uint8_t magic_byte, size_t max_packet_size)
            : device_(std::move(device)), crc_(crc),
              magic_byte_(magic_byte),
              max_packet_size(max_packet_size)
    {
        auto buffer_size = HEADER_SIZE_ + max_packet_size +
                           padding_(max_packet_size) + CHECKSUM_SIZE_;
        tx_packet_ = new uint8_t[buffer_size];
        rx_packet_ = new uint8_t[buffer_size];
    }

    PacketServer::~PacketServer()
    {
        delete tx_packet_;
        delete rx_packet_;
    }

    /** State the process of transmitting a packet.
     *
     * @param type type of the packet
     * @param size size of the packet
     * @return buffer to write packet data to
     */
    void *PacketServer::init_packet(uint8_t type, uint16_t size)
    {
        if (size > max_packet_size)
        {
            return nullptr;
        }
        tx_packet_size_ = size;
        tx_packet_[0] = magic_byte_;
        tx_packet_[1] = type;
        memcpy(&tx_packet_[2], &tx_packet_size_, sizeof(tx_packet_size_));
        return &tx_packet_[HEADER_SIZE_];
    }

    /** Send the packet prepared with @ref init_packet.
     *
     * If @p blocking is not true then it will only send as many bytes of the
     * packet as the underlying device can accept at the moment.
     *
     * @param blocking set to true to block until the packet is sent
     * @retval true: the packet was successfully sent, always if @p blocking
     * @retval false: the packet could not be sent
     */
    bool PacketServer::send_packet(bool blocking)
    {
        // pad the packet to ALIGNMENT_
        size_t padding = padding_(HEADER_SIZE_ + tx_packet_size_);
        memset(&tx_packet_[HEADER_SIZE_ + tx_packet_size_], 0, padding);

        // append checksum
        uint32_t checksum = compute_checksum_(
                tx_packet_, HEADER_SIZE_ + tx_packet_size_ + padding);
        memcpy(&tx_packet_[HEADER_SIZE_ + tx_packet_size_],
               &checksum, sizeof(checksum));

        size_t sent = 0;
        size_t total_size = HEADER_SIZE_ + tx_packet_size_
                            + padding + CHECKSUM_SIZE_;

        // non blocking
        if (!blocking)
        {
            sent = device_->write(tx_packet_, total_size);
            return sent == total_size;
        }

        // blocking
        while (sent < total_size)
        {
            while (!device_->write_ready());
            sent += device_->write(tx_packet_, total_size - sent);
        }
        return true;
    }

    /** Register a packet @p handler for the given @p type.
     *
     * @param type packet type to handle
     * @param handler function to handle the packet
     */
    void PacketServer::register_handler(
            uint8_t type, std::function<void(void *, size_t)> handler)
    {
        auto result = handlers_.insert(std::make_pair(type, handler));
        if (!result.second)
        {
            result.first->second = handler;
        }
    }

    /** Receive a packet.
     *
     * This method should be called whenever packet handling is desired.
     *
     * If @p blocking is not true then only a part of a packet may be received
     * and will resume receiving on the next call to this method.
     *
     * @warning do not mix blocking and non blocking calls
     *
     * @param blocking set to true to block until a packet is received
     * @retval true: a packet was received and handled
     * @retval false: no packet was received, the checksum was invalid,
     *                or there was no handler
     */
    bool PacketServer::receive_packet(bool blocking)
    {
        if (blocking)
        {
            return receive_packet_blocking_();
        }
        return receive_packet_nonblocking_();
    }

    /** Receive a packet in blocking mode.
     *
     * This method should be called whenever packet handling is desired.
     *
     * @retval true: a packet was received and handled
     * @retval false: no packet was received, the checksum was invalid,
     *                or there was no handler
     */
    bool PacketServer::receive_packet_blocking_()
    {
        received_bytes_ = 0;

        // search for magic byte
        do
        {
            while (device_->read(&rx_packet_[0], 1) == 0);
        } while (rx_packet_[0] != magic_byte_);
        ++received_bytes_;

        // retrieve the header and packet size
        read_to_(HEADER_SIZE_, true);
        memcpy(&rx_packet_size_, &rx_packet_[HEADER_SIZE_ - 2],
               sizeof(rx_packet_size_));

        // retrieve the remainder of the packet
        size_t total_size = HEADER_SIZE_ + rx_packet_size_ +
                            padding_(rx_packet_size_) + CHECKSUM_SIZE_;
        read_to_(total_size, true);
        received_bytes_ = 0;

        // verify checksum and call handler
        if (!verify_rx_packet_())
        {
            return false;
        }
        return handle_rx_packet_();
    }

    /** Receive a packet in non-blocking mode.
     *
     * This method should be called whenever packet handling is desired.
     *
     * @retval true: a packet was received and handled
     * @retval false: no packet was received, the checksum was invalid,
     *                or there was no handler
     */
    bool PacketServer::receive_packet_nonblocking_()
    {
        // return immediately if device is not ready
        if (!device_->read_ready())
        {
            return false;
        }

        // search for magic byte
        if (received_bytes_ == 0)
        {
            if ((device_->read(&rx_packet_[0], 1) == 0) ||
                (rx_packet_[0] != magic_byte_))
            {
                return false;
            }
            ++received_bytes_;
        }

        // retrieve the header and packet size
        if (received_bytes_ < HEADER_SIZE_)
        {
            if (!read_to_(HEADER_SIZE_, false))
            {
                return false;
            }
            memcpy(&rx_packet_size_, &rx_packet_[HEADER_SIZE_ - 2],
                   sizeof(rx_packet_size_));
        }

        // retrieve the remainder of the packet
        size_t total_size = HEADER_SIZE_ + rx_packet_size_ +
                            padding_(rx_packet_size_) + CHECKSUM_SIZE_;
        if (!read_to_(total_size, false))
        {
            return false;
        }
        received_bytes_ = 0;

        // verify checksum and call handler
        if (!verify_rx_packet_())
        {
            return false;
        }
        return handle_rx_packet_();
    }

    /** Verify the received packet.
     *
     * @retval true: the RX packet has a valid checksum
     * @retval false: the RX packet does not have a valid checksum
     */
    bool PacketServer::verify_rx_packet_()
    {
        auto size = HEADER_SIZE_ + rx_packet_size_ + padding_(rx_packet_size_);
        uint32_t checksum;
        memcpy(&checksum, &rx_packet_[size], CHECKSUM_SIZE_);
        uint32_t computed_checksum = compute_checksum_(rx_packet_, size);
        return checksum != computed_checksum;
    }

    /** Call the handler for the received packet.
     *
     * @retval true: the RX packet was handled
     * @retval false: the RX packet was not handled
     */
    bool PacketServer::handle_rx_packet_()
    {
        if (handlers_.find(rx_packet_[1]) == handlers_.end())
        {
            return false;
        }
        handlers_[rx_packet_[1]](
                &rx_packet_[HEADER_SIZE_], rx_packet_size_);
    }

    /** Advance reading of the received packet to a given @p length.
     *
     * Unless @p blocking is set to true it may not be able to extend the
     * received packet to the requested length.
     *
     * @param length total length to extend the received packet to
     * @param blocking set to true to block until this length is achieved
     * @retval true: the received packet has been extend to @p length
     * @retval false: the received packet has not been extend to @p length
     */
    bool PacketServer::read_to_(size_t length, bool blocking)
    {
        // have already read this much
        if (received_bytes_ >= length)
        {
            return true;
        }

        // non blocking
        if (!blocking)
        {
            received_bytes_ += device_->read(
                    &rx_packet_[received_bytes_], length - received_bytes_);
            return received_bytes_ == length;
        }

        // blocking
        while ((received_bytes_ < length))
        {
            received_bytes_ += device_->read(
                    &rx_packet_[received_bytes_], length - received_bytes_);
        }
        return true;
    }

    /** Compute the CRC 32 checksum of the given buffer.
     *
     * @param buffer the buffer to compute the checksum for
     * @param length the length of the @p buffer, must be multiple of 4 bytes
     * @return 32 bit checksum of the given @p buffer
     */
    uint32_t PacketServer::compute_checksum_(void *buffer, size_t length)
    {
        return HAL_CRC_Calculate(
                &crc_, static_cast<uint32_t *>(buffer), length);
    }

    /** Compute the amount of padding required for @ref ALIGNMENT_.
     *
     * @param length length of the current data
     * @return number of bytes that must be added to the buffer for
     *         @ref ALIGNMENT_
     */
    size_t PacketServer::padding_(size_t length)
    {
        return ALIGNMENT_ * ((length + 1 - ALIGNMENT_) / ALIGNMENT_) - length;
    }

}
