//
// Created by Michael R. Shannon on 5/7/19.
//

#include <functional>
#include "Controller.h"

namespace slc {

    using namespace std::placeholders;

    size_t Controller::add_motor(std::unique_ptr<EncodedMotor> motor)
    {
        motors_.push_back(std::move(motor));
        return motors_.size();
    }

    EncodedMotor *Controller::motor(size_t id)
    {
        return motors_.at(id).get();
    }

    void Controller::tick()
    {
        for (auto &motor : motors_)
        {
            motor->tick();
        }
    }

    std::function<void()> Controller::get_ticker()
    {
        return std::function<void()>();
    }

    std::function<void(void *, size_t)> Controller::get_pid_handler()
    {
        return std::bind(&Controller::pid_handler_, this, _1, _2);
    }

    std::function<void(void *, size_t)> Controller::get_power_handler()
    {
        return std::bind(&Controller::power_handler_, this, _1, _2);
    }

    std::function<void(void *, size_t)> Controller::get_angle_handler()
    {
        return std::bind(&Controller::angle_handler_, this, _1, _2);
    }

    std::function<void(void *, size_t)> Controller::get_speed_handler()
    {
        return std::bind(&Controller::speed_handler_, this, _1, _2);
    }

    typedef struct __attribute__((packed))
    {
        uint8_t motor;
        uint8_t type;  // 1 for position, 2 for velocity
        float proportional_gain;
        float integral_gain;
        float derivative_gain;
    } PIDMessage_;

#define POSITION 1
#define VELOCITY 2

    void Controller::pid_handler_(void *packet, size_t size)
    {
        if (size < sizeof(PIDMessage_))
        {
            return;
        }
        auto message = static_cast<PIDMessage_ *>(packet);
        if (message->motor >= motors_.size())
        {
            return;
        }
        PIDController *pid = nullptr;
        switch (message->type)
        {
            case POSITION:
                pid = motors_.at(message->motor)->get_position_controller();
                break;
            case VELOCITY:
                pid = motors_.at(message->motor)->get_velocity_controller();
                break;
            default:
                return;
        }
        pid->set_proportional_gain(message->proportional_gain);
        pid->set_integral_gain(message->integral_gain);
        pid->set_derivative_gain(message->derivative_gain);
    }

    typedef struct __attribute__((packed))
    {
        uint8_t motor;
        uint8_t mode;  // 1 = forward, 2 = reverse, 3 = stop, 4 = stop
        uint8_t power;
    } PowerMessage_;

#define FORWARD 1
#define REVERSE 2
#define STOP 3
#define DRIFT 4

    void Controller::power_handler_(void *packet, size_t size)
    {
        if (size < sizeof(PowerMessage_))
        {
            return;
        }
        auto message = static_cast<PowerMessage_ *>(packet);
        if (message->motor >= motors_.size())
        {
            return;
        }
        EncodedMotor *motor = motors_.at(message->motor).get();
        switch (message->mode)
        {
            case FORWARD:
                motor->forward(message->power);
                break;
            case REVERSE:
                motor->reverse(message->power);
                break;
            case STOP:
                motor->stop(message->power);
                break;
            case DRIFT:
                motor->drift();
                break;
            default:
                return;
        }
    }

    typedef struct __attribute__((packed))
    {
        uint8_t motor;
        float degrees;
    } AngleMessage_;

    void Controller::angle_handler_(void *packet, size_t size)
    {
        if (size < sizeof(AngleMessage_))
        {
            return;
        }
        auto message = static_cast<AngleMessage_ *>(packet);
        if (message->motor >= motors_.size())
        {
            return;
        }
        motors_.at(message->motor)->angle(message->degrees);
    }

    typedef struct __attribute__((packed))
    {
        uint8_t motor;
        float rpm;
    } SpeedMessage_;

    void Controller::speed_handler_(void *packet, size_t size)
    {
        if (size < sizeof(SpeedMessage_))
        {
            return;
        }
        auto message = static_cast<SpeedMessage_ *>(packet);
        if (message->motor >= motors_.size())
        {
            return;
        }
        motors_.at(message->motor)->speed(message->rpm);
    }

}
