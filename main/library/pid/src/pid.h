#pragma once

class PIDController
{
public:
    PIDController(
        float kp = 0.0f,
        float ki = 0.0f,
        float kd = 0.0f
    );

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_output, float max_output);

    void reset();

    float update(float target, float current, float dt);

private:
    float kp_;
    float ki_;
    float kd_;

    float prev_error_;
    float prev_prev_error_;

    float output_;

    float min_output_;
    float max_output_;

    bool first_update_;
};