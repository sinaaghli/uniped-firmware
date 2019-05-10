//
// Created by Michael R. Shannon on 5/5/19.
//

#include <memory>
#include <utility>
#include <optional>
#include <functional>

#include "EncodedMotor.h"

namespace slc {

    /** Construct an encoded motor without bounding angles.
     *
     * @param enable PWM object connected to the enable pin the H bridge
     * @param c input 1 of the H bridge
     * @param d input 2 of the H bridge
     * @param encoder angular encoder attached to the motor
     * @param position_pid PID controller for angle setpoints
     * @param velocity_pid PID controller for RPM setpoints
     */
    EncodedMotor::EncodedMotor(
            PWM enable, GPIO c, GPIO d,
            std::shared_ptr<AngularEncoder> encoder,
            PIDController position_pid, PIDController velocity_pid)
            : Motor(std::move(enable), std::move(c), std::move(d)),
              encoder_(std::move(encoder)),
              position_pid_(position_pid), velocity_pid_(velocity_pid)
    {
        drift();
    }

    /** Construct an encoded motor with bounding angles.
     *
     * @param enable PWM object connected to the enable pin the H bridge
     * @param c input 1 of the H bridge
     * @param d input 2 of the H bridge
     * @param encoder angular encoder attached to the motor
     * @param position_pid PID controller for angle setpoints
     * @param velocity_pid PID controller for RPM setpoints
     * @param min_angle minimum angle (in degrees) that the motor is allowed
     *                  to have, when below this the motor will stop
     * @param max_angle maximum angle (in degrees) that the motor is allowed
     *                  to have, when above this the motor will stop
     */
    EncodedMotor::EncodedMotor(
            PWM enable, GPIO c, GPIO d,
            std::shared_ptr<AngularEncoder> encoder,
            PIDController position_pid, PIDController velocity_pid,
            float min_angle, float max_angle)
            : Motor(std::move(enable), std::move(c), std::move(d)),
              encoder_(std::move(encoder)),
              position_pid_(position_pid), velocity_pid_(velocity_pid),
              min_angle_(min_angle), max_angle_(max_angle)
    {
        drift();
    }

    /** Turn on motor in the forward direction.
     *
     * Sets mode to "power".
     *
     * @param power power between 0 (min) and 100 (max)
     */
    void EncodedMotor::forward(int power)
    {
        mode_ = EncodedMotorMode::power_forward;
        Motor::forward(power);
    }

    /** Turn on motor in the reverse direction.
     *
     * Sets mode to "power" control.
     *
     * @param power power between 0 (min) and 100 (max)
     */
    void EncodedMotor::reverse(int power)
    {
        mode_ = EncodedMotorMode::power_reverse;
        Motor::reverse(power);
    }

    /** Enable breaking mode.
     *
     * Sets mode to "power" control.
     *
     * @param power power between 0 (min) and 100 (max)
     */
    void EncodedMotor::stop(int power)
    {
        mode_ = EncodedMotorMode::power_stop;
        Motor::stop(power);
    }

    /** Enable free floating mode.
     *
     * Sets mode to "power" control.
     *
     */
    void EncodedMotor::drift()
    {
        mode_ = EncodedMotorMode::power_drift;
        Motor::drift();
    }

    /** Enable angle control mode and set desired angle.
     *
     * @param degrees angle setpoint in degrees
     */
    void EncodedMotor::angle(float degrees)
    {
        mode_ = EncodedMotorMode::angle;
        if (min_angle_.has_value() && degrees < min_angle_.value())
        {
            degrees = min_angle_.value();
        }
        if (max_angle_.has_value() && degrees > max_angle_.value())
        {
            degrees = max_angle_.value();
        }
        position_pid_.set_target(degrees);
    }

    /** Enable speed control mode and set desired speed.
     *
     * @param rpm speed setpoint in RPM
     */
    void EncodedMotor::speed(float rpm)
    {
        mode_ = EncodedMotorMode::speed;
        velocity_pid_.set_target(rpm);
    }

    /** Call from tick to stop motor from going outside of bounds.
     *
     */
    void EncodedMotor::bounds_check_()
    {
        switch (mode_)
        {
            case EncodedMotorMode::power_forward:
                if (max_angle_.has_value() &&
                    encoder_->degrees().second >= max_angle_.value())
                {
                    stop(100);
                }
                break;
            case EncodedMotorMode::power_reverse:
                if (min_angle_.has_value() &&
                    encoder_->degrees().second <= min_angle_.value())
                {
                    stop(100);
                }
                break;
            case EncodedMotorMode::speed:
                if (velocity_pid_.get_target() > 0 && max_angle_.has_value() &&
                    encoder_->degrees().second >= max_angle_.value())
                {
                    stop(100);
                }
                if (velocity_pid_.get_target() < 0 && max_angle_.has_value() &&
                    encoder_->degrees().second <= min_angle_.value())
                {
                    stop(100);
                }
                break;
        }
    }

    /** Call frequently to run single iteration control loop.
     *
     */
    void EncodedMotor::tick()
    {
        std::optional<float> pid_output;
        bounds_check_();
        switch (mode_)
        {
            case EncodedMotorMode::angle:
            {
                auto[encoder_count, degrees] = encoder_->degrees();
                if (encoder_count > encoder_count_)
                {
                    encoder_count_ = encoder_count;
                    position_pid_.set_input(degrees);
                }
                pid_output = position_pid_.get_output();
                break;
            }
            case EncodedMotorMode::speed:
            {
                auto[encoder_count, rpm] = encoder_->rpm();
                if (encoder_count > encoder_count_)
                {
                    encoder_count_ = encoder_count;
                    velocity_pid_.set_input(rpm);
                }
                pid_output = velocity_pid_.get_output();
                break;
            }
        }
        if (pid_output.has_value())
        {
            int power = static_cast<int>(std::max(-100.0f, std::min(
                    100.0f, pid_output.value() * 100)));
            if (power == 0)
            {
                Motor::drift();
            }
            else if (power < 0)
            {
                Motor::reverse(-power);
            }
            else
            {
                Motor::forward(power);
            }
        }
    }

    /** Get function bound to this instance that calls tick.
     *
     * @return function bound to this instance that runs single iteration
     *         of control loop
     */
    std::function<void()> EncodedMotor::get_ticker()
    {
        return std::bind(&EncodedMotor::tick, this);
    }

    /** Get pointer of position PID controller.
     *
     * @return pointer to PID controller used for position
     */
    PIDController *EncodedMotor::get_position_controller()
    {
        return &position_pid_;
    }

    /** Get pointer of angular speed PID controller.
     *
     * @return pointer to PID controller used for angular speed
     */
    PIDController *EncodedMotor::get_velocity_controller()
    {
        return &velocity_pid_;
    }

}
