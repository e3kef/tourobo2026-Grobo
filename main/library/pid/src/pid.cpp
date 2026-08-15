#include "pid.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      prev_error_(0.0f),
      prev_prev_error_(0.0f),
      output_(0.0f),
      min_output_(-1000.0f),
      max_output_(1000.0f),
      first_update_(true)
{
}

void PIDController::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(float min_output, float max_output)
{
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::reset()
{
    prev_error_ = 0.0f;
    prev_prev_error_ = 0.0f;
    output_ = 0.0f;
    first_update_ = true;
}

float PIDController::update(float target, float current, float dt)
{
    if (dt <= 0.0f) {
        return output_;
    }

    float error = target - current;

    float delta_output;

    if (first_update_) {
        // 初回はD項を使わない
        delta_output =
            kp_ * error +
            ki_ * error * dt;

        first_update_ = false;
    }
    else {
        delta_output =
            kp_ * (error - prev_error_) +
            ki_ * error * dt +
            kd_ * (error - 2.0f * prev_error_ + prev_prev_error_) / dt;
    }

    output_ += delta_output;

    if (output_ > max_output_) {
        output_ = max_output_;
    }
    else if (output_ < min_output_) {
        output_ = min_output_;
    }

    prev_prev_error_ = prev_error_;
    prev_error_ = error;

    return output_;
}