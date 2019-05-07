//
// Created by Michael R. Shannon on 5/5/19.
//

#ifndef ENCODEDMOTOR_H
#define ENCODEDMOTOR_H

#include <memory>
#include <optional>
#include <functional>

#include "AngularEncoder.h"
#include "PIDController.h"
#include "Motor.h"

namespace slc {

    enum class EncodedMotorMode
    {
        power_forward,
        power_reverse,
        power_stop,
        power_drift,
        angle,
        speed
    };

    class EncodedMotor : public Motor
    {
    public:
        EncodedMotor(PWM enable, GPIO c, GPIO d,
                     std::shared_ptr<AngularEncoder> encoder,
                     PIDController position_pid,
                     PIDController velocity_pid);
        EncodedMotor(PWM enable, GPIO c, GPIO d,
                     std::shared_ptr<AngularEncoder> encoder,
                     PIDController position_pid,
                     PIDController velocity_pid,
                     float min_angle, float max_angle);
        void forward(int power) override;
        void reverse(int power) override;
        void stop(int power) override;
        void drift() override;
        void angle(float degrees);
        void speed(float rpm);
        void tick();
        std::function<void()> get_ticker();
        PIDController &get_position_controller();
        PIDController &get_velocity_controller();

    private:
        EncodedMotorMode mode_ = EncodedMotorMode::power_drift;
        std::shared_ptr<AngularEncoder> encoder_;
        size_t encoder_count_ = 0;
        PIDController position_pid_;
        PIDController velocity_pid_;
        std::optional<float> min_angle_;
        std::optional<float> max_angle_;

        void bounds_check();

    };

}


#endif //ENCODEDMOTOR_H
