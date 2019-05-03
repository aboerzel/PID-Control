#include "PID.h"
#include <iostream>
#include <numeric>
#include <algorithm>

PID::PID(const double kp, const double ki, const double kd)
{
    p_[0] = kp;
    p_[1] = ki;
    p_[2] = kd;
}

PID::~PID()
= default;

void PID::twiddle_one_param(const unsigned param, const double twiddle_coefficient, const double tolerance,
                            const int stabilization_steps, const int twiddle_steps)
{
    current_param_ = param;
    dp_[param] = twiddle_coefficient;

    tolerance_ = tolerance;

    stabilization_steps_ = std::max(stabilization_steps, 1);;
    twiddle_update_steps_ = twiddle_steps;

    twiddle_all_params_ = false;
    twiddle_enabled_ = true;
}

void PID::twiddle_all_params(const double dp, const double di, const double dd, const double tolerance,
                             const int stabilization_steps, const int twiddle_steps)
{
    current_param_ = 0;
    dp_[0] = dp;
    dp_[1] = di;
    dp_[2] = dd;

    tolerance_ = tolerance;

    stabilization_steps_ = std::max(stabilization_steps, 1);
    twiddle_update_steps_ = twiddle_steps;

    twiddle_all_params_ = true;
    twiddle_enabled_ = true;
}

void PID::update_error(const double error)
{
    if (update_count_ == 0)
    {
        p_error_ = error;
    }

    // Calculate errors  
    d_error_ = error - p_error_;
    i_error_ += error;
    p_error_ = error;

    printf("******************** Update %d *******************\n", update_count_);
    printf("p error    : %4.4f\n", p_error_);
    printf("i error    : %4.4f\n", i_error_);
    printf("d error    : %4.4f\n", d_error_);

    // Calculate total error
    if (update_count_ > stabilization_steps_)
    {
        total_error_ += error * error;
    }

    if (twiddle_enabled_ && update_count_ >= twiddle_update_steps_)
    {
        twiddle();
    }

    printf("Current  kp: %.08f ki: %.08f kd: %.08f\n", p_[0], p_[1], p_[2]);
    printf("Best     kp: %.08f ki: %.08f kd: %.08f\n", best_p_[0], best_p_[1], best_p_[2]);
    printf("Twiddle  dp: %.08f di: %.08f dd: %.08f\n", dp_[0], dp_[1], dp_[2]);
    printf("Twiddle tot: %.08f\n", std::accumulate(dp_.begin(), dp_.end(), 0.0));
    printf("Total error: %.3f\n", total_error_);
    printf("Best  error: %.3f\n", best_error_);

    update_count_++;
}

double PID::control()
{
    auto control = -p_[0] * p_error_ - p_[1] * i_error_ - p_[2] * d_error_;

    // Limit control between -1.0 and 1.0
    if (control > 1.0)
        control = 1.0;
    else if (control < -1.0)
        control = -1.0;

    return control;
}

void PID::twiddle()
{
    // Do nothing if twiddle disabled or finished
    if (twiddle_finished_)
    {
        printf("Twiddle finished!\n");
        return;
    }

    if (twiddle_step_ == 0)
    {
        // Start with increment
        p_[current_param_] += dp_[current_param_];
        twiddle_step_ = 1;
    }
    else if (twiddle_step_ == 1)
    {
        // Last update was an increment
        if (total_error_ < best_error_)
        {
            // Save best parameter and error and increment current parameter and increase dp
            best_error_ = total_error_;
            best_p_ = p_;
            dp_[current_param_] *= 1.1;
            p_[current_param_] += dp_[current_param_];

            // Choose next parameter
            if (twiddle_all_params_)
                if (++current_param_ == p_.size()) current_param_ = 0;
        }
        else
        {
            // Revert last increment and and go in the opposite direction
            p_[current_param_] -= 2 * dp_[current_param_];
            twiddle_step_ = 2;
        }
    }
    else if (twiddle_step_ == 2)
    {
        // Last update was an decrement
        if (total_error_ < best_error_)
        {
            // Save best parameter and error and increment current parameter and increase dp
            best_error_ = total_error_;
            best_p_ = p_;
            dp_[current_param_] *= 1.1;
        }
        else
        {
            // Revert last decrement and decrease dp
            p_[current_param_] += dp_[current_param_];
            dp_[current_param_] *= 0.9;
        }

        p_[current_param_] += dp_[current_param_];
        twiddle_step_ = 1;

        // Choose next parameter
        if (twiddle_all_params_)
            if (++current_param_ == p_.size()) current_param_ = 0;
    }

    // Check if the twiddle goal has been reached
    twiddle_finished_ = std::accumulate(dp_.begin(), dp_.end(), 0.0) < tolerance_;

    // Reset total error
    total_error_ = 0;

    // Reset update count
    update_count_ = 0;

    // Reset simulator when control coefficients are changed 
    restart_simulator();
}

void PID::restart_simulator()
{
    std::string reset_msg = "42[\"reset\",{}]";
    server_.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void PID::set_server(uWS::WebSocket<uWS::SERVER> ws)
{
    server_ = ws;
}
