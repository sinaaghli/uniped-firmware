//
// Created by Michael R. Shannon on 5/7/19.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstddef>
#include <functional>
#include <vector>

#include "EncodedMotor.h"

namespace slc {

    /** Main controller class.
     *
     * This is used to control motor configuration and control from
     * incoming message packets.
     *
     */
    class Controller
    {
    public:
        size_t add_motor(std::unique_ptr<EncodedMotor> motor);
        EncodedMotor *motor(size_t id);
        void tick();
        std::function<void()> get_ticker();
        std::function<void(void *, size_t)> get_pid_handler();
        std::function<void(void *, size_t)> get_power_handler();
        std::function<void(void *, size_t)> get_angle_handler();
        std::function<void(void *, size_t)> get_speed_handler();

    private:
        std::vector<std::unique_ptr<EncodedMotor>> motors_;

        void pid_handler_(void *packet, size_t size);
        void power_handler_(void *packet, size_t size);
        void angle_handler_(void *packet, size_t size);
        void speed_handler_(void *packet, size_t size);


    };

}

#endif //CONTROLLER_H
