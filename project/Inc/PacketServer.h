//
// Created by Michael R. Shannon on 3/5/19.
//

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <stm32f4xx_hal.h>
#include "CharacterDevice.h"

#ifndef PACKETSERVER_H
#define PACKETSERVER_H

namespace slc {

    /** Packet server.
     *
     * This class represents a packet server that can both transmit and
     * receive packets on a @ref CharacterDevice.
     *
     */
    class PacketServer
    {
    public:
        const size_t max_packet_size;

        PacketServer(std::unique_ptr<CharacterDevice> device,
                     CRC_HandleTypeDef &crc,
                     uint8_t magic_byte, size_t max_packet_size);

        ~PacketServer();

        void *init_packet(uint8_t type, uint16_t size);

        bool send_packet(bool blocking = false);

        void register_handler(
                uint8_t type, std::function<void(void *, size_t)> handler);

        bool receive_ready();

        bool receive_packet(bool blocking = false);

    private:
        static const size_t ALIGNMENT_ = 4;
        static const size_t HEADER_SIZE_ = 4;
        static const size_t CHECKSUM_SIZE_ = 4;
        std::map<uint8_t, std::function<void(void *, size_t)>> handlers_;
        std::unique_ptr<CharacterDevice> device_;
        CRC_HandleTypeDef &crc_;
        uint8_t magic_byte_;
        uint16_t tx_packet_size_ = 0;
        uint16_t rx_packet_size_ = 0;
        size_t received_bytes_ = 0;
        uint8_t *tx_packet_;
        uint8_t *rx_packet_;

        bool receive_packet_blocking_();

        bool receive_packet_nonblocking_();

        bool verify_rx_packet_();

        bool handle_rx_packet_();

        bool read_to_(size_t length, bool blocking = false);

        uint32_t compute_checksum_(void *buffer, size_t length);

        size_t padding_(size_t length);


    };

}

#endif //PACKETSERVER_H
