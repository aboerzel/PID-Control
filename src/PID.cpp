#include "PID.h"
#include <iostream>
#include <numeric>
#include <math.h>

#define NUM_TWIDDLE_UPDATES 1000

PID::PID()
= default;

PID::~PID()
= default;

void PID::init(const double kp, const double ki, const double kd,
               const double k_cte, const double k_steer, const double k_speed,
               const bool twiddle_steer, const bool twiddle_speed)
{
    steer_p_[0] = kp;
    steer_p_[1] = ki;
    steer_p_[2] = kd;

    speed_p_[0] = k_cte;
    speed_p_[1] = k_steer;
    speed_p_[2] = k_speed;

    best_steer_p_ = steer_p_;
    best_speed_p_ = speed_p_;

    steer_dp_[0] = steer_p_[0] / 10;
    steer_dp_[1] = steer_p_[1] / 10;
    steer_dp_[2] = steer_p_[2] / 10;

    speed_dp_[0] = speed_p_[0] / 10;
    speed_dp_[1] = speed_p_[1] / 10;
    speed_dp_[2] = speed_p_[2] / 10;

    twiddle_steer_ = twiddle_steer;
    twiddle_speed_ = twiddle_speed;
}

void PID::update_error(const double cte, const double speed, const double angle)
{
    if (!is_initialized_)
    {
        p_error_ = cte;
        is_initialized_ = true;
    }

    // Calculate errors
    d_error_ = cte - p_error_;
    i_error_ += cte;
    p_error_ = cte;

    steer_error_ = fabs(angle);
    speed_error_ = fabs(speed);

    // Calculate total errors
    cte_sum_ += cte * cte;
    speed_err_sum_ += speed_error_ * (1 / (fabs(cte) + 0.5)) / 100;

    printf("Steer parameter Kp:    %.08f Ki:      %.08f Kd:      %.08f)\n", steer_p_[0], steer_p_[1], steer_p_[2]);
    printf("Best steer par. Kp:    %.08f Ki:      %.08f Kd:      %.08f)\n", best_steer_p_[0], best_steer_p_[1], best_steer_p_[2]);

    printf("Speed parameter K_cte: %.08f K_steer: %.08f K_speed: %.08f)\n", speed_p_[0], speed_p_[1], speed_p_[2]);
    printf("Best speed par. Kp:    %.08f Ki:      %.08f Kd:      %.08f)\n", best_steer_p_[0], best_steer_p_[1], best_steer_p_[2]);
}

double PID::steer_control()
{
    auto control = -steer_p_[0] * p_error_ - steer_p_[1] * i_error_ - steer_p_[2] * d_error_;

    // Limit speed control between -1.0 and 1.0
    if (control > 1.0)
        control = 1.0;
    else if (control < -1.0)
        control = -1.0;

    return control;
}

double PID::speed_control()
{
    return speed_p_[0] * (1 / (fabs(p_error_) + 0.0001)) +
           speed_p_[1] * (1 / (steer_error_ + 0.0001)) +
           speed_p_[2] * (1 / (speed_error_ + 0.0001));
}

bool PID::twiddle()
{
    // Do nothing if twiddle disabled or finished
    if (!twiddle_steer_ || twiddle_steer_finished_ && !twiddle_speed_ || twiddle_speed_finished_)
        return false;

    printf("Twiddle %d : total cte: %4.8f - total speed err: %4.8f\n", twiddle_update_count_, cte_sum_, speed_err_sum_);
    printf("Twiddle %d : best  cte: %4.8f - best  speed err: %4.8f\n", twiddle_update_count_, best_steer_err_, best_speed_err_);

    auto reset_simulator = false;

    if (twiddle_update_count_ == 2)
    {
        // Reset simulator when control coefficients are changed 
        reset_simulator = true;

        // Reset error values
        p_error_ = 0;
        i_error_ = 0;
        d_error_ = 0;

        if (!twiddle_steer_finished_)
        {
            // Check if the steer twiddle goal has been reached
            twiddle_steer_finished_ = std::accumulate(steer_dp_.begin(), steer_dp_.end(), 0.0) <= steer_tolerance_;
            if (!twiddle_steer_finished_)
            {
                // twiddle steer parameter and reset cte
                twiddle_update(twiddle_step_steer_, twiddle_steer_param_, cte_sum_, best_steer_err_, steer_p_, steer_dp_, best_steer_p_);
                cte_sum_ = 0;
            }
        }

        if (!twiddle_speed_finished_)
        {
            // Check if the speed twiddle goal has been reached
            twiddle_speed_finished_ = std::accumulate(speed_dp_.begin(), speed_dp_.end(), 0.0) <= speed_tolerance_;
            if (twiddle_speed_finished_)
            {
                // twiddle speed parameter and reset speed error
                twiddle_update(twiddle_step_speed_, twiddle_speed_param_, speed_err_sum_, best_speed_err_, speed_p_, speed_dp_, best_speed_p_);
                speed_err_sum_ = 0;
            }
        }
    }

    // Wait for NUM_TWIDDLE_UPDATES error updates before next twiddle update
    if (twiddle_update_count_ < NUM_TWIDDLE_UPDATES)
    {
        twiddle_update_count_ += 1;
    }
    else
    {
        twiddle_update_count_ = 0; // Start next twiddle update
    }

    return reset_simulator;
}

void PID::twiddle_update(unsigned& step, unsigned& param, const double err, double& best_err,
                         std::vector<double>& p, std::vector<double>& dp, std::vector<double>& best_p)
{
    if (step == 0)
    {
        // Start with increment
        p[param] += dp[param];
        step = 1;
    }
    else if (step == 1)
    {
        // Last update was an increment
        if (err < best_err)
        {
            // Save best parameter and error and increment current parameter and increase dp
            best_err = err;
            best_p = p;
            dp[param] *= 1.1;

            // Choose next parameter
            if (++param == p.size()) param = 0;
            step = 0;
        }
        else
        {
            // Revert last increment and and go in the opposite direction
            p[param] -= 2 * dp[param];
            step = 2;
        }
    }
    else if (step == 2)
    {
        // Last update was an decrement
        if (err < best_err)
        {
            // Save best parameter and error and increment current parameter and increase dp
            best_err = err;
            best_p = p;
            dp[param] *= 1.1;
        }
        else
        {
            // Revert last decrement and decrease dp
            p[param] += dp[param];
            dp[param] *= 0.9;
        }

        // Choose next parameter
        if (++param == p.size()) param = 0;
        step = 0;
    }
}
